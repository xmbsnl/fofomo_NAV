# -*- coding: utf-8 -*-
"""真机远程控制：通过 SSH 在工控机上启停机器人 / 建图 / 导航。

为什么单独一个文件
------------------
GUI 以前只能启动**本机**进程（Gazebo、自研驱动）。真机作业的正确架构是：
驱动 + 融合 + 安全层 + 导航栈全部跑在工控机上，上位机（GUI）只做监控与下发。
所以这里不复用 ManagedProcess，而是把「在工控机上起/停 launch」独立出来。

依赖
----
工控机必须已配好免密登录：
    ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519
    ssh-copy-id teamhd@192.168.2.153

实现要点
--------
1. 启动用 `setsid nohup ... &`：让进程脱离 SSH 会话，否则 SSH 一断 launch 就死。
2. 停止用 `pkill -INT -f <launch>`：ros2 launch 收到 SIGINT 会优雅地逐个停节点，
   直接 -9 会留下 dt_ros2 占着串口，下次启动报 "device busy"。
3. 所有远程命令都带超时，网络断了不能把 GUI 界面卡死。
"""
import os
import shlex
import subprocess
import threading
import time

# 工控机上拉起整个系统的 launch（见 02-hardware/dt01_ws 的 dt01_all.launch.py）
ALL_LAUNCH = 'dt01_all.launch.py'

# 判断"车上有没有在跑"用的特征串。
#
# ⚠️ 必须写成 dt01_all[.]launch.py 这种形式，不能直接写 dt01_all.launch.py：
#   pgrep -f 'dt01_all.launch.py' 是通过 ssh 远程执行的，而承载这条命令的
#   远程 shell（bash -c "pgrep -f 'dt01_all.launch.py' ..."）自身的命令行里
#   就含有这个字符串 → pgrep 会把这个 shell 自己也匹配上 → 永远返回"在运行"。
#   加方括号后，shell 命令行里是 "[.]"（不匹配正则里的 .），只有真正的
#   ros2 launch 进程（命令行含 dt01_all.launch.py）才会被匹配。
PROC_PATTERN = 'dt01_all[.]launch[.]py'

# 除了 launch 父进程，还要清理"可能被单独启动/成为孤儿"的进程。
# 场景：调试时手工 `ros2 run dt01_bringup imu_preprocess.py`，
# 它不属于任何 launch，只 pkill launch 是杀不掉的 —— 下次打开 GUI
# 会直接连上这个残留节点，表现为"一进界面就有传感器数据"。
ORPHAN_PATTERNS = [
    'imu_preprocess[.]py',
    'safety_mux[.]py',
    'chassis_bridge[.]py',
    'odom_to_tf[.]py',
    'link_watchdog[.]py',
    'async_slam_toolbox[_.]',
    'nav2_[a-z]',
    'ahrs_driver_nod[e]',
    # robot_localization 的 EKF（可执行名 ekf_node，节点名 ekf_filter_node）：
    # 以前名单里没有它 → 每次启停都杀不干净，反复几次后累积出 3 个 EKF
    # 同时发 odom->base_footprint，TF 互相打架 → 位姿/朝向跳变（2026-09-18）。
    'ekf_nod[e]',
    # 底盘驱动本体：以前名单里也没有它，杀不干净时会残留一个，
    # 重启后又起一个 → **两个 dt_ros2 同时读 /dev/ttyS7**，串口数据互相踩，
    # /odom 直接跳变（2026-09-18）。
    'dt_ros2_nod[e]',
    # 蓝海雷达驱动（2026-09-18 补）：以前名单里没有它 → 停止/关窗后雷达节点
    # 还活着，/scan 继续往上位机灌，表现为"退出界面了还能看到雷达数据"。
    'bluesea2_nod[e]',
    # robot_state_publisher（2026-09-28 补）：它在 /opt/ros 路径下，
    # 上面的 dt01_ws 兜底匹配不到 —— 历代启动残留的 RSP 僵尸进程
    # 每个都占一个 DDS 参与者槽位，攒到上限后新节点报
    # "Failed to find a free participant index"，整套导航起不来。
    'robot_state_publishe[r]',
    # 兜底：凡是从工控机 dt01_ws 装出来的节点（命令行含 dt01_ws/install/.../lib/）
    # 都算"我们的进程"。launch 以后再加新节点也不会再漏 —— 这次雷达驱动
    # 漏掉就是因为名单是逐个手写的。同样用反自匹配写法。
    'dt01_ws/instal[l]',
    # 巡线节点（2026-09-28 补）：由 GUI 部署启动的独立 python 脚本，
    # 不属于 dt01_ws install —— 不加进名单的话停止/断开后它还挂着。
    'route_follower_nod[e]',
]

# 巡线节点进程匹配串（与 ORPHAN_PATTERNS 中条目一致，反自匹配写法）
ROUTE_FOLLOWER_PROC_PATTERN = 'route_follower_nod[e]'
# 巡线节点在工控机上的路径与日志（GUI 部署时上传到 ~/route_follower_node.py）
ROUTE_FOLLOWER_REMOTE = '~/route_follower_node.py'
ROUTE_FOLLOWER_LOG = '/tmp/dt01_route_follower.log'


class RemoteHost(object):
    """封装对一台工控机的 SSH 操作。所有方法都不抛异常，失败返回 (False, 原因)。"""

    def __init__(self, user='teamhd', host='192.168.2.153',
                 ws='~/dt01_ws', maps_dir='~/dt01_maps', log=None):
        self.user = user
        self.host = host
        self.ws = ws
        self.maps_dir = maps_dir
        self._log = log or (lambda _msg: None)
        # 启动后 tail 的日志（工控机上的路径）
        self.remote_log = '/tmp/dt01_gui_launch.log'
        # 命令通道串行化：GUI 里可能有多个后台线程同时发 ssh（点「启动机器人」的
        # probe 线程 + 主线程同步地图），并发抢同一条 ControlMaster 连接时会互相
        # 排队，小命令被大传输堵住 → 第一次点击必超时、第二次就好了。
        self._ssh_lock = threading.Lock()

    # ------------------------------------------------------------------ 基础
    @property
    def target(self):
        return '%s@%s' % (self.user, self.host)

    def _ssh(self, remote_cmd, timeout=12, retry_on_timeout=True):
        """执行一条远程命令，返回 (returncode, stdout, stderr)。

        两处防超时设计（针对"每次进 GUI 第一次点启动机器人必超时"）：

        1. 命令通道串行（_ssh_lock）。多条 ssh 复用同一条 ControlMaster 连接，
           而 scp 拉地图也走这条连接 —— 大文件传输会把小命令堵在后面排队，
           于是第一次点击超时、第二次（传输已结束）就好了。
        2. 超时自动重试一次。冷启动握手或工控机瞬时高负载时偶发超时，
           重试基本必成。注意：**只有超时才重试**，远端命令返回非 0 不重试，
           避免把 start()/stop() 这类有副作用的命令执行两遍。
        """
        cmd = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10',
               '-o', 'StrictHostKeyChecking=no',
               self.target, remote_cmd]
        last_err = ''
        for attempt in (1, 2):
            waited = 0.0
            try:
                t_lock = time.time()
                with self._ssh_lock:
                    waited = time.time() - t_lock
                    if waited > 1.0:
                        # 等锁 = 有别的线程正在发 ssh。排查"第一次点击超时"时
                        # 这个数字是关键证据：等锁久说明是 GUI 内部排队，
                        # 等锁≈0 却超时说明是链路/工控机的问题。
                        self.log('[SSH] 等锁 %.1fs: %s'
                                 % (waited, remote_cmd[:50]))
                    t0 = time.time()
                    p = subprocess.run(cmd, capture_output=True, text=True,
                                       timeout=timeout)
                cost = time.time() - t0
                if cost > 2.0:
                    # 慢命令观测：正常应在 1s 内，>2s 说明在排队或链路劣化
                    self.log('[SSH] 慢命令 %.1fs: %s'
                             % (cost, remote_cmd[:60]))
                return p.returncode, p.stdout.strip(), p.stderr.strip()
            except subprocess.TimeoutExpired:
                last_err = 'SSH 超时（%ss）' % timeout
                self.log('[SSH] 超时 %.0fs（第%d次，等锁%.1fs）: %s'
                         % (timeout, attempt, waited, remote_cmd[:60]))
                if attempt == 1 and retry_on_timeout:
                    self.log('SSH 超时，自动重试一次…')
                    time.sleep(0.5)
                else:
                    # start()/stop() 这类**有副作用**的命令绝不重试：
                    # 重试就等于再执行一次启动 —— 那正是"双 SLAM 打架、
                    # 位姿跳变"的来源（2026-09-18）。
                    break
            except Exception as e:                              # noqa: BLE001
                return -2, '', str(e)
        return -1, '', last_err

    def log(self, msg):
        self._log(str(msg))

    # ------------------------------------------------------------------ 查询
    def is_reachable(self):
        """工控机能不能连上（不要求系统在跑）。

        timeout=10：首次连接可能赶上主线程在做其它 SSH 串行任务
        （如地图同步），命令要排队，6s 不够用。
        """
        rc, out, _err = self._ssh('echo ok', timeout=10)
        return rc == 0 and out == 'ok'

    def status(self):
        """一条 SSH 同时问清两件事：连得上吗？车上在跑什么？

        返回 'slam' / 'nav' / 'idle' / 'none'，**None 表示连不上**（SSH 失败或超时）。

        为什么合成一条：以前点一次「启动机器人」要发三条命令 ——
        `echo ok` 探连通 + pgrep 查模式 + pgrep 查是否在跑。其实前一条
        成功就证明连通，两条 pgrep 查的也是同一批进程。三条合成一条，
        省两次往返，也就少两次在 WiFi 上超时的机会（2026-09-18 简化）。

        'idle' = 机器人本体在跑但没在建图/导航；'none' = 连通但什么都没跑。
        """
        rc, out, _err = self._ssh(
            "if pgrep -f 'async_slam_toolbox[_.]' >/dev/null; then echo slam; "
            "elif pgrep -f 'nav2_[a-z]' >/dev/null; then echo nav; "
            "elif pgrep -f '%s' >/dev/null; then echo idle; "
            "else echo none; fi" % PROC_PATTERN,
            timeout=10)
        if rc != 0:
            return None
        out = out.strip()
        return out if out in ('slam', 'nav', 'idle', 'none') else 'none'

    def is_running(self):
        """工控机上是否已有 dt01_all 在跑。

        只看 ros2 launch 那个进程（父进程），不数子节点 —— 子节点同名进程
        有十几个，任何一个残留都会让判断失真。
        """
        rc, out, _err = self._ssh(
            "pgrep -f '%s' > /dev/null && echo yes || echo no" % PROC_PATTERN,
            timeout=8)
        return rc == 0 and out.strip() == 'yes'

    def running_pids(self):
        """返回所有匹配的 PID（排障用）。"""
        rc, out, _err = self._ssh("pgrep -f '%s'" % PROC_PATTERN, timeout=8)
        if rc != 0 or not out:
            return []
        return [int(x) for x in out.split() if x.isdigit()]

    def running_mode(self):
        """车上正在跑哪种模式：'slam' / 'nav' / None。

        用于"接管"：用户在终端手动起了建图，GUI 不必重启，直接进入
        对应模式的监控状态即可（重启反而会丢掉已建的图）。

        正则的反自匹配技巧与 PROC_PATTERN 相同 —— 这条命令通过 ssh 执行时，
        远程 shell 自己的命令行里就含这些模式串，直接写会被匹配成"永远在跑"：
          async_slam_toolbox[_.]   真实进程名后跟 '_'，shell 字面后跟 '['
          nav2_[a-z]               真实进程名后跟字母，shell 字面后跟 '['
        """
        rc, out, _err = self._ssh(
            "if pgrep -f 'async_slam_toolbox[_.]' >/dev/null; then echo slam; "
            "elif pgrep -f 'nav2_[a-z]' >/dev/null; then echo nav; "
            "else echo none; fi",
            timeout=8)
        out = out.strip()
        return out if out in ('slam', 'nav') else None

    def remote_path(self, name):
        """把地图名转成工控机上的绝对路径（slam_toolbox 保存时用它）。"""
        return os.path.join(self.maps_dir, name)

    def list_maps(self):
        """列出工控机上已保存的地图名（不含扩展名）。"""
        rc, out, _err = self._ssh(
            'ls %s/*.yaml 2>/dev/null | xargs -n1 basename 2>/dev/null' % self.maps_dir,
            timeout=8)
        if rc != 0 or not out:
            return []
        return sorted(os.path.splitext(os.path.basename(line))[0]
                      for line in out.splitlines() if line.strip())

    # ------------------------------------------------------------------ 控制
    def kill_all(self):
        """强杀工控机上整套系统 —— 启动超时/失败后收拾现场。

        用户要求：**超时就要把对应进程全部杀掉，不要影响下一次点击**。
        否则第一次超时留下的残骸会让第二次点击变成"接管"，甚至叠加出第二套
        SLAM（两套互发 map->odom TF，位姿疯狂跳变）。

        ⚠️ pkill -f 的模式必须写成反自匹配（`slam_toolbo[x]`），否则会匹配到
        执行本命令的远程 bash 自身，把自己杀掉导致 ssh 断连（exit 255）。
        """
        pats = [PROC_PATTERN] + ORPHAN_PATTERNS
        pats_str = ' '.join("'%s'" % p for p in pats)
        script = (
            "for p in %s; do pkill -INT -f \"$p\" 2>/dev/null; done; "
            "sleep 2; "
            "for p in %s; do pkill -KILL -f \"$p\" 2>/dev/null; done; "
            "echo done" % (pats_str, pats_str))
        self._ssh(script, timeout=20, retry_on_timeout=False)
        self.log('[%s] 已清理现场（强杀残留进程）' % self.target)

    def start(self, mode='idle', map_file='', extra=''):
        """在工控机上启动整个系统。

        mode:
          'slam'  机器人 + 建图
          'nav'   机器人 + 导航（必须给 map_file）
          'idle'  只起机器人（驱动+雷达+IMU+安全层），不进任何模式
                  —— 「启动机器人」按钮走这条：先有传感器数据，
                     建图/导航由用户随后点按钮再起。
        map_file: 导航模式下的地图 yaml 路径（工控机上的路径）
        返回 (bool, 说明文字)
        """
        if self.is_running():
            return False, '工控机上已经在运行了，请先停止'

        if mode == 'slam':
            launch_arg = 'mode:=slam'
        elif mode == 'nav':
            if not map_file:
                return False, '导航模式必须先指定地图'
            launch_arg = "map_file:='%s'" % map_file
        else:
            # 既不是建图也不是导航：dt01_all 只拉起机器人本体
            launch_arg = 'mode:=idle'
        if extra:
            launch_arg += ' ' + extra

        inner = (
            "cd %s && source /opt/ros/humble/setup.bash && "
            "source ./install/setup.bash && "
            "export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp "
            "CYCLONEDDS_URI=file:///home/%s/cyclonedds.xml && "
            "ros2 launch dt01_bringup %s %s"
            % (self.ws, self.user, ALL_LAUNCH, launch_arg)
        )
        # ⚠️ 重定向必须加在**整条后台子命令**上，不能只加在 ros2 launch 后面。
        # 旧写法：`cd && source && ... && setsid nohup ros2 launch >log 2>&1 </dev/null &`
        # 只有 launch 自己重定向了，而执行这条 && 链的后台 shell 仍握着 ssh 的
        # stdout 管道；ros2 launch 在管着所有节点、几小时不退出，那个 shell 就
        # 一直等它 —— ssh 于是一直空等到 20s 超时。
        # 后果三连（2026-09-18 修复）：
        #   1. 明明启动成功了，GUI 却报超时；
        #   2. 用户再点一次 → status 查到 slam → 显示"已接管"（看似要点两次）；
        #   3. 超时自动重试又真执行了一遍启动 → 两套 SLAM 互发 TF → 位姿跳变。
        # 把整条链塞进 `bash -c` 再整体重定向，ssh 立即拿到 started 返回（实测 0.6s）。
        remote = ("setsid nohup bash -c %s > %s 2>&1 < /dev/null & echo started"
                  % (shlex.quote(inner), self.remote_log))
        rc, out, err = self._ssh(remote, timeout=15, retry_on_timeout=False)
        if rc != 0 or 'started' not in out:
            # 失败或超时：把可能已经半起来的进程杀干净，保证下一次点击
            # 是从零开始的干净启动（用户要求"超时就杀掉，别影响第二次点击"）。
            self.kill_all()
            detail = (err or out or '无输出').strip().splitlines()
            return False, '启动失败（已清理现场）：' + (
                detail[-1] if detail else '未知原因')
        self.log('[%s] 已在工控机启动（%s）' % (self.target, launch_arg))
        return True, '已启动'

    def stop(self):
        """优雅停止工控机上的整套系统（含被单独启动的孤儿节点）。

        退出 GUI 时必须真正杀干净，否则下次打开会直接连上残留进程，
        界面一进去就有传感器数据（用户多次反馈的问题）。

        性能：整套停止动作**合并成一次 SSH 往返**在远端执行。
        之前逐个 pkill 要 7~10 次 SSH，每次都要重新握手，最坏累计
        130 秒+，在 WiFi 上极易触发超时且关窗明显卡顿。合并后
        单次命令即可完成，通常 10 秒内返回。
        """
        pats = [PROC_PATTERN] + ORPHAN_PATTERNS
        alive_any = ' || '.join("pgrep -f '%s' >/dev/null" % p for p in pats)

        # 全部逻辑远端执行：检测 → SIGINT → 等待 → TERM → 清理孤儿 → KILL
        script = """
if ! (%(alive)s); then echo NOOP; exit 0; fi
pkill -INT -f '%(launch)s' 2>/dev/null || true
for i in $(seq 1 20); do
  pgrep -f '%(launch)s' >/dev/null || break
  sleep 0.5
done
if pgrep -f '%(launch)s' >/dev/null; then
  pkill -TERM -f '%(launch)s' 2>/dev/null || true
  echo TERM
fi
for p in %(pats)s; do pkill -INT -f "$p" 2>/dev/null || true; done
sleep 2
if (%(orphan_alive)s); then
  for p in %(pats)s; do pkill -KILL -f "$p" 2>/dev/null || true; done
  echo KILLED
fi
if (%(alive)s); then echo STILL_ALIVE; else echo STOPPED; fi
""".strip() % {
            'alive': alive_any,
            'launch': PROC_PATTERN,
            'pats': ' '.join("'%s'" % p for p in pats),
            'orphan_alive': ' || '.join(
                "pgrep -f '%s' >/dev/null" % p for p in ORPHAN_PATTERNS),
        }

        rc, out, _err = self._ssh(script, timeout=60, retry_on_timeout=False)
        out = (out or '').strip()
        if rc != 0 and not out:
            return False, 'SSH 失败（未执行停止）'
        if 'NOOP' in out:
            return True, '本来就没在跑'
        if 'STILL_ALIVE' in out:
            self.log('[警告] 仍有进程残留，请到工控机上手动检查')
            return False, '部分进程未能停止'
        if 'KILLED' in out:
            self.log('[警告] 有节点未响应 SIGINT，已强制 KILL')
        self.log('[%s] 已停止（含孤儿节点清理）' % self.target)
        return True, '已停止'

    def _alive_patterns(self):
        """返回**仍在运行**的进程模式名（一次 SSH 往返查完整个名单）。

        排障与"断开连接"后的复查都靠它：单看 launch 父进程不够 ——
        launch 已经死了、但雷达/底盘驱动这类孤儿还活着时，is_running()
        会返回 False 从而跳过清理（这正是"退出 GUI 还能拿到雷达数据"的成因）。
        """
        pats = [PROC_PATTERN] + ORPHAN_PATTERNS
        script = ' '.join(
            "pgrep -f '%s' >/dev/null && echo '%s';" % (p, p) for p in pats
        ) + ' true'
        rc, out, _err = self._ssh(script, timeout=15)
        if rc != 0:
            return ['<查询失败>']
        return [x for x in (out or '').split() if x]

    def disconnect(self):
        """彻底断开：优雅停 → 强杀兜底 → 复查，保证点了就一定干净。

        给 GUI 的「断开机器人连接」按钮用（用户反馈"退出界面后还能拿到
        底盘/雷达数据"）。比 stop() 更狠，两处关键差异：

        1. **不查 is_running()**：那个只看 launch 父进程，launch 死了孤儿
           还在时会返回 False，整个清理被跳过；
        2. **杀完复查**：真有残留就如实报错，不让界面假装已断开。
        """
        self.stop()
        self.kill_all()
        alive = self._alive_patterns()
        if alive:
            self.log('[警告] 断开后仍有残留：%s' % ', '.join(alive))
            return False, '仍有进程残留：' + '、'.join(alive[:4])
        self.log('[%s] 已彻底断开（无残留进程）' % self.target)
        return True, '已断开（工控机上已无机器人进程）'

    def tail_log(self, lines=30):
        """取工控机上 launch 日志的最后几行，排障用。"""
        rc, out, _err = self._ssh(
            'tail -n %d %s 2>/dev/null' % (lines, self.remote_log), timeout=8)
        return out if rc == 0 else '(无日志)'

    # ------------------------------------------------------------------ 巡线节点
    def route_follower_running(self):
        """巡线节点是否已在工控机上运行。"""
        rc, out, _err = self._ssh(
            "pgrep -f '%s' > /dev/null && echo yes || echo no"
            % ROUTE_FOLLOWER_PROC_PATTERN, timeout=8)
        return rc == 0 and out.strip() == 'yes'

    def deploy_route_follower(self, local_script):
        """确保巡线节点在工控机上运行：查进程 → 比对 md5 → 上传 → 启动。

        返回 (ok, msg)。启动用 setsid nohup 整体重定向（与 start() 同款
        防挂起写法），脚本内容变化（本地改过）时自动重新上传。
        """
        import hashlib
        if self.route_follower_running():
            return True, '巡线节点已在运行（复用，不重启）'
        # 本地脚本 md5
        try:
            with open(local_script, 'rb') as f:
                local_md5 = hashlib.md5(f.read()).hexdigest()
        except Exception as e:                                  # noqa: BLE001
            return False, f'读不到本地巡线节点脚本 {local_script}: {e}'
        # 远端 md5（不存在则为空）→ 不一致才传，省一次 scp
        rc, rmd5, _err = self._ssh(
            "md5sum %s 2>/dev/null | cut -d' ' -f1" % ROUTE_FOLLOWER_REMOTE,
            timeout=8)
        if rc != 0 or rmd5.strip() != local_md5:
            self.log('[巡线] 上传节点脚本到工控机…')
            rc, _out, err = self._scp(
                local_script, '%s:%s' % (self.target, ROUTE_FOLLOWER_REMOTE),
                timeout=30)
            if rc != 0:
                return False, '上传巡线节点失败: %s' % (err or 'scp 失败')
        # 启动（环境与 start() 完全一致：ROS + DDS 环境变量）
        inner = (
            "source /opt/ros/humble/setup.bash && "
            "export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp "
            "CYCLONEDDS_URI=file:///home/%s/cyclonedds.xml && "
            "python3 %s" % (self.user, ROUTE_FOLLOWER_REMOTE)
        )
        remote = ("setsid nohup bash -c %s > %s 2>&1 < /dev/null & echo started"
                  % (shlex.quote(inner), ROUTE_FOLLOWER_LOG))
        rc, out, err = self._ssh(remote, timeout=15, retry_on_timeout=False)
        if rc != 0 or 'started' not in out:
            return False, '启动巡线节点失败: %s' % (err or out or '超时')
        # 简单存活确认（给 python 启动留 1s）
        time.sleep(1.0)
        if not self.route_follower_running():
            log_tail = self._ssh('tail -n 5 %s 2>/dev/null' % ROUTE_FOLLOWER_LOG,
                                 timeout=8)[1]
            return False, ('巡线节点启动后即退出，日志末尾：\n' + (log_tail or '(无)'))
        return True, '巡线节点已启动'

    def stop_route_follower(self):
        """停止巡线节点（「停止导航」时用；stop()/disconnect() 也会按
        ORPHAN_PATTERNS 自动清理它，这里是即时精确停止）。"""
        pat = ROUTE_FOLLOWER_PROC_PATTERN
        self._ssh("pkill -INT -f '%s' 2>/dev/null; sleep 1; "
                  "pkill -KILL -f '%s' 2>/dev/null; true" % (pat, pat),
                  timeout=10, retry_on_timeout=False)

    # ------------------------------------------------------------------ 参数
    def ros_param_set(self, node, name, value):
        """在工控机上执行 `ros2 param set`，动态调整运行中节点的参数。

        node 形如 '/route_follower'、'/controller_server'、'/amcl'；
        name 形如 'v_max'、'FollowPath.max_vel_x'（yaml 嵌套用点号）。
        返回 (bool, 说明)。参数多为 Nav2 动态参数，set 后即时生效，无需重启。

        ⚠️ retry_on_timeout=False：ros2 param set 有副作用，超时重试会
        重复设置；与 start()/stop() 同一约定。
        """
        node = node.lstrip('/')
        if isinstance(value, float):
            value = f'{value:.6g}'
        inner = (
            "source /opt/ros/humble/setup.bash && "
            "export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp "
            "CYCLONEDDS_URI=file:///home/%s/cyclonedds.xml && "
            "ros2 param set /%s %s %s" % (self.user, node, name, value)
        )
        rc, out, err = self._ssh(inner, timeout=15, retry_on_timeout=False)
        if rc != 0:
            return False, (err or out or 'SSH 失败').strip()
        # 成功时 ros2 param set 输出 "Set parameter successful"
        return True, (out or '参数已设置')

    # ------------------------------------------------------------------ 地图
    def ensure_maps_dir(self):
        """确保工控机上的地图目录存在（slam_toolbox 不会自己建目录）。"""
        self._ssh('mkdir -p %s' % self.maps_dir, timeout=8)

    def has_map(self, name):
        rc, _out, _err = self._ssh(
            'test -f %s.yaml -a -f %s.pgm' % (self.remote_path(name),
                                              self.remote_path(name)),
            timeout=8)
        return rc == 0

    def fetch_map(self, name, local_dir):
        """把工控机上的地图拉一份到本地，供 GUI 显示与打点。

        工控机上命名是 <name>.yaml / <name>.pgm；
        本地统一为 <local_dir>/map.yaml + map.pgm（GUI 的惯例），
        所以拉回来后要把 yaml 里的 image: 字段改写成 map.pgm。

        返回本地 map.yaml 路径，失败返回 None。
        """
        try:
            os.makedirs(local_dir, exist_ok=True)
        except Exception as e:                                  # noqa: BLE001
            self.log('创建本地地图目录失败: %s' % e)
            return None

        remote = self.remote_path(name)
        local_yaml = os.path.join(local_dir, 'map.yaml')
        local_pgm = os.path.join(local_dir, 'map.pgm')

        src = '%s:%s.pgm' % (self.target, remote)
        dst = local_pgm
        rc, _out, err = self._scp(src, dst, timeout=30)
        if rc != 0:
            self.log('拉取地图 %s 失败: %s' % (name, err or 'scp 失败'))
            return None

        # yaml 单独拉到临时名，再改写 image 字段
        rc, _out, err = self._scp('%s:%s.yaml' % (self.target, remote),
                                  local_yaml, timeout=15)
        if rc != 0:
            self.log('拉取地图描述 %s 失败: %s' % (name, err or 'scp 失败'))
            return None
        try:
            with open(local_yaml, 'r', encoding='utf-8') as f:
                text = f.read()
            lines = []
            for line in text.splitlines():
                if line.strip().startswith('image:'):
                    lines.append('image: map.pgm')
                else:
                    lines.append(line)
            with open(local_yaml, 'w', encoding='utf-8') as f:
                f.write('\n'.join(lines) + '\n')
        except Exception as e:                                  # noqa: BLE001
            self.log('改写地图描述失败: %s' % e)
            return None
        return local_yaml

    def _scp(self, src, dst, timeout=30):
        # ⚠️ ControlMaster=no + ControlPath=none：让 scp **另开一条独立连接**，
        # 不要复用 ssh 的持久化连接。
        # 原因：~/.ssh/config 开了 ControlMaster 复用（为了让 GUI 的频繁 ssh 不重复
        # 握手），复用连接是单条 TCP。scp 拉地图（几百 KB~几 MB）会把这条连接占满，
        # 同一时刻发出的 ssh 小命令（echo ok / pgrep）就排在后面等 → 触发超时。
        # 传文件走独立连接后，命令通道永远畅通。
        cmd = ['scp', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10',
               '-o', 'StrictHostKeyChecking=no',
               '-o', 'ControlMaster=no', '-o', 'ControlPath=none',
               src, dst]
        try:
            p = subprocess.run(cmd, capture_output=True, text=True,
                               timeout=timeout)
            return p.returncode, p.stdout, p.stderr
        except subprocess.TimeoutExpired:
            return -1, '', 'scp 超时'
        except Exception as e:                                  # noqa: BLE001
            return -2, '', str(e)
