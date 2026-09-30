#!/usr/bin/env python3
"""巡线跟随节点（route_follower）——机器人本机侧的"走线算法"。

真车安全架构：
  GUI（笔记本）──发──▶ /dt01/route/goal      (Path)    巡线路线（运行中收到会被忽略）
           ──发──▶ /dt01/route/ctrl      (String)  'start' / 'stop' / 'vmax:0.30'
           ──发──▶ /dt01/route/heartbeat (String)  2Hz 心跳
  本节点（机器人本机）──▶ /cmd_vel（20Hz 本地闭环，不依赖 GUI 存活）
                    ◀── /amcl_pose（定位）、/scan（避障）
                    ──▶ /dt01/route/status (String, JSON)  进度回报给 GUI

安全机制：
  - GUI 断链/崩溃 → 心跳超时（hb_timeout）自动零速停车
  - 定位丢失 → 立即停车等待恢复
  - 单点超时（goal_timeout）→ 跳过该点继续后续点
  - 前方 < stop_dist → 蠕行 + 满转向绕行（不停车等待）
  - 重复 'start'/'stop' 均为幂等操作

巡线规则（与 GUI 打点连线一致）：
  每段先原地转向对正下一点（先调姿态再出发）→ 前视点追踪走直线
  （严格贴打点连线）→ 分层避障：预警区(avoid_far)小角度提前变道、
  近距区(avoid_near)大幅转向边转边走、紧急区(stop_dist)蠕行绕行；
  绕行侧带记忆防摇摆，障碍解除后慢速回线。

用法：
  仿真（本机，GUI 自动拉起）：
    python3 route_follower_node.py --ros-args -p use_sim_time:=true
  真机（工控机）：
    python3 route_follower_node.py
  参数在线调（无需改代码）：
    ros2 param set /route_follower v_max 0.30
"""
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def _wrap(angle):
    """归一化角度到 [-pi, pi]。"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RouteFollower(Node):
    def __init__(self):
        super().__init__('route_follower')
        for name, default in [
            ('v_max', 0.35),          # 最大线速度 m/s
            ('w_max', 0.80),          # 最大角速度 rad/s
            ('arrive_tol', 0.15),      # 到点判定半径 m
            # ---- 分层避障（2026-09-29 重写：提前绕 / 边转边走 / 慢回线）----
            ('avoid_far', 1.50),       # 预警区触发距离 m（>此距离不避障）
            ('avoid_near', 1.00),      # 近距区触发距离 m（此距离内大幅转向）
            ('stop_dist', 0.22),       # 紧急区边界 m（此距离内蠕行绕行）
            ('avoid_w', 0.80),         # 避障角速度上限 rad/s
            ('v_turn_min', 0.15),      # 近距最低前进速度 m/s（边转边走下限）
            ('v_creep', 0.06),         # 紧急区蠕行速度 m/s（替代原停车等待）
            ('avoid_off_max', 0.80),   # 前视点横向偏移上限 m
            ('recover_rate', 0.98),    # 偏移回线衰减速率（越大回线越慢）
            ('kp_rot', 1.6),           # 原地对正 P 增益
            ('min_w', 0.12),           # 克服静摩擦的最小角速度 rad/s
            ('kp_lat', 2.2),           # 直线追踪 P 增益
            ('look_ahead', 0.5),      # 前视距离 m
            ('goal_timeout', 90.0),    # 单点巡线超时 s
            ('hb_timeout', 2.0),       # GUI 心跳超时 s（断链停车）
        ]:
            self.declare_parameter(name, default)

        # ---- 输入 ----
        self.create_subscription(Path, '/dt01/route/goal', self._on_goal, 10)
        self.create_subscription(String, '/dt01/route/ctrl', self._on_ctrl, 10)
        self.create_subscription(String, '/dt01/route/heartbeat',
                                 self._on_hb, 10)
        self.create_subscription(String, '/dt01/route/keepout',
                                 self._on_keepout, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._on_pose, 10)

        # ---- 输出 ----
        # 真机关键：自主控制输出必须发 /cmd_vel_nav（与 Nav2 同一入口），
        # 经 safety_mux 急停/防撞/限速仲裁后变成 /cmd_vel 到底盘。
        # 直接发 /cmd_vel 会绕过安全层并与其抢底盘（严禁）。
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')
        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.status_pub = self.create_publisher(
            String, '/dt01/route/status', 10)

        # ---- 状态 ----
        self._route = []          # [(x, y), ...] 待走点位
        self._idx = 0             # 当前执行点
        self._phase = 'idle'      # idle | rotate | drive
        self._seg_start = (0.0, 0.0)   # 当前段起点（投影回线用）
        self._avoid_off = 0.0     # 避障横向偏移（+左 / -右，m）
        self._avoid_side = 0.0    # 绕行侧记忆：+1 左 / -1 右 / 0 无（防左右摇摆）
        self._pose = None         # map 系 (x, y, yaw)
        self._scan = None
        self._keepout = []        # 电子围栏：[[(x,y),...], ...] 禁区多边形列表
        self._last_hb = None
        self._goal_t0 = None
        self._success = 0
        self._total = 0
        self._last_status_t = 0.0

        self.create_timer(0.05, self._step)       # 20Hz 控制回路
        self.create_timer(1.0, self._check_timeouts)
        self.get_logger().info('route_follower 就绪，等待 GUI 下发路线')

    def _p(self, name):
        return float(self.get_parameter(name).value)

    # ================= 输入 =================
    def _on_goal(self, msg: Path):
        if self._phase != 'idle':
            return   # 巡线运行中忽略路线更新（GUI 冗余重发包不扰乱执行）
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not pts:
            return
        # 电子围栏：剔除落在禁区多边形内的点（连线端点级过滤，含起点）
        kept = [p for p in pts if not self._in_keepout(*p)]
        dropped = len(pts) - len(kept)
        if dropped:
            self.get_logger().warn(f'电子围栏：剔除 {dropped} 个落在禁区内的点')
        if not kept:
            self.get_logger().error('电子围栏：全部目标点都在禁区内，路线被拒绝')
            self._publish_status(phase='idle',
                                 msg='全部目标点在禁区内，路线被拒绝', force=True)
            return
        self._route = kept
        self._total = len(kept)
        self._success = 0
        self._idx = 0
        self.get_logger().info(f'收到巡线路线：{len(kept)} 个点（等待 start 指令）')

    def _on_ctrl(self, msg: String):
        data = msg.data
        if data == 'stop':
            if self._phase != 'idle':
                self._halt('收到停止指令')
        elif data == 'start':
            if self._phase == 'idle' and self._route:
                self._last_hb = time.monotonic()
                self._begin_segment()
                self.get_logger().info('开始巡线')
        elif data.startswith('vmax:'):
            try:
                v = max(0.05, min(0.50, float(data.split(':', 1)[1])))
                self.set_parameters([
                    Parameter('v_max', Parameter.Type.DOUBLE, v)])
                self.get_logger().info(f'巡线速度已设为 {v:.2f} m/s')
            except Exception:
                pass

    def _on_hb(self, msg: String):
        self._last_hb = time.monotonic()

    def _on_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        self._pose = (p.x, p.y, yaw)

    def _on_scan(self, msg: LaserScan):
        self._scan = msg

    # ================= 电子围栏 =================
    def _on_keepout(self, msg: String):
        """接收 GUI 下发的禁区多边形列表（JSON）：
        [{'name':'施工区','points':[[x,y],...]}, ...]
        至少 3 个点才构成有效多边形，其余丢弃。
        """
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        zones = []
        if isinstance(data, list):
            for z in data:
                pts = z.get('points', []) if isinstance(z, dict) else z
                try:
                    poly = [(float(p[0]), float(p[1])) for p in pts]
                except Exception:
                    continue
                if len(poly) >= 3:
                    zones.append(poly)
        self._keepout = zones
        if zones:
            self.get_logger().info(f'电子围栏已更新：{len(zones)} 个禁区')

    @staticmethod
    def _point_in_poly(x, y, poly):
        """射线法判断点 (x, y) 是否在多边形内（含边界）。"""
        inside = False
        n = len(poly)
        j = n - 1
        for i in range(n):
            xi, yi = poly[i]
            xj, yj = poly[j]
            if ((yi > y) != (yj > y)) and \
                    (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
                inside = not inside
            j = i
        return inside

    def _in_keepout(self, x, y):
        """点是否落在任一禁区多边形内。"""
        for poly in self._keepout:
            if self._point_in_poly(x, y, poly):
                return True
        return False

    # ================= 段落控制 =================
    def _begin_segment(self):
        if self._idx >= len(self._route):
            self._finish()
            return
        if self._pose is not None:
            self._seg_start = (self._pose[0], self._pose[1])
        self._avoid_off = 0.0
        self._avoid_side = 0.0
        self._phase = 'rotate'    # 先原地对正，再直线出发
        self._goal_t0 = time.monotonic()

    def _halt(self, reason):
        self._publish_cmd(0.0, 0.0)
        self._phase = 'idle'
        self._publish_status(msg=reason, force=True)
        self.get_logger().info(f'巡线停止：{reason}')

    def _finish(self):
        self._publish_cmd(0.0, 0.0)
        self._phase = 'idle'
        self._publish_status(phase='done', msg='全部点位到达', force=True)
        self.get_logger().info(f'巡线完成：{self._success}/{self._total} 点到达')

    # ================= 20Hz 主回路 =================
    def _step(self):
        if self._phase == 'idle':
            return
        now = time.monotonic()

        # 心跳看门狗：GUI 断链/崩溃 → 零速停车（真车安全核心）
        hb_timeout = self._p('hb_timeout')
        if self._last_hb is None or now - self._last_hb > hb_timeout:
            self._halt(f'GUI 心跳丢失（断链保护，已停车）')
            return

        # 定位丢失：停车等待恢复
        if self._pose is None:
            self._publish_cmd(0.0, 0.0)
            self._publish_status(msg='定位不可用，停车等待恢复')
            return

        rx, ry, ryaw = self._pose
        tx, ty = self._route[self._idx]
        v_max = self._p('v_max')
        w_max = self._p('w_max')
        dist = math.hypot(tx - rx, ty - ry)

        # ---- 电子围栏实时校验：目标点被划入禁区（围栏运行中更新）→ 停车 ----
        if self._in_keepout(tx, ty):
            self._halt('目标点在禁区内，已停车（电子围栏）')
            return

        # ---- 到点判定 ----
        if dist <= self._p('arrive_tol'):
            self._publish_cmd(0.0, 0.0)
            elapsed = now - self._goal_t0 if self._goal_t0 else 0.0
            self._success += 1
            self._goal_t0 = None
            self.get_logger().info(
                f'到达第 {self._idx + 1}/{len(self._route)} 点'
                f'（用时 {elapsed:.1f}s）')
            self._idx += 1
            self._begin_segment()
            return

        # ---- 阶段1：原地转向对正下一点（车不动，只调方向）----
        if self._phase == 'rotate':
            err = _wrap(math.atan2(ty - ry, tx - rx) - ryaw)
            if abs(err) < 0.06:
                self._phase = 'drive'
                self.get_logger().info(f'姿态已对正，沿直线驶向第 {self._idx + 1} 点')
            else:
                w = self._p('kp_rot') * err
                w = max(-w_max, min(w_max, w))
                min_w = self._p('min_w')
                if abs(w) < min_w:
                    w = math.copysign(min_w, w)   # 克服底盘静摩擦
                self._publish_cmd(0.0, w)
                self._publish_status(
                    msg=f'第{self._idx + 1}点 原地对正中 '
                        f'(偏差 {math.degrees(err):.0f}°)')
                return

        # ---- 阶段2：直线巡线 + 分层避障 ----
        # 分层策略（2026-09-29 重写）：
        #   预警区 (near, far]  ：小角度提前变道，基本不减速，弧线切出
        #   近距区 (stop, near] ：大幅转向 + 保持前进（v 不为 0，边转边走）
        #   紧急区 (0, stop]    ：蠕行 + 满转向绕行（不停车等待）
        # 绕行侧带记忆（进避障锁一侧，前方恢复通畅才释放，防左右摇摆）；
        # 障碍解除后偏移慢速回线（绕完由巡线自然导回，不急拉）。
        goal_bearing = _wrap(math.atan2(ty - ry, tx - rx) - ryaw)

        # ---- 车头偏离目标过远：切回 rotate 原地对正 ----
        # drive 阶段是"边走边追"，目标跑到侧后方时只能前进绕大圈调头
        # （09-29 实测：避障带偏后绕了一整圈回到出发区）。目标方位超过
        # 135° 视为被避障带偏/冲过目标，停下原地对正再出发。
        if abs(goal_bearing) > math.radians(135.0):
            self._avoid_side = 0.0
            self._avoid_off = 0.0
            self._phase = 'rotate'
            self._publish_cmd(0.0, 0.0)
            self._publish_status(msg='车头偏离目标过大，原地重新对正',
                                 force=True)
            return

        front, left_d, right_d = self._route_obstacle()
        avoid_far = self._p('avoid_far')
        avoid_near = self._p('avoid_near')
        stop_dist = self._p('stop_dist')
        avoid_w = self._p('avoid_w')

        # ---- 绕行侧判定（最小转弯优先 + 记忆防摇摆）----
        if front < avoid_far:
            if self._avoid_side == 0.0:
                self._avoid_side = self._pick_avoid_side(
                    goal_bearing, left_d, right_d, avoid_near)
        else:
            # 前方恢复通畅即释放绕行记忆（残余横向偏移由 recover_rate 回落）。
            # ⚠ 旧条件 min(左,右) > avoid_near 在贴墙/走廊环境永不满足——
            # 侧面总有墙，绕行侧被锁死：前方明明通畅还满舵 → 原地画圈；
            # 且下方预警区公式 k 为负 → 角速度反向 + 速度超上限 +
            # 偏移累加到上限 → "走不了直线 / 绕一圈回出发区"（09-29 实测）。
            self._avoid_side = 0.0
        side = self._avoid_side

        # ---- 分区计算避障角速度 / 速度 ----
        zone = ''
        w_avoid = 0.0
        v_avoid = None
        if side != 0.0:
            if front <= stop_dist:
                # 紧急区：蠕行 + 满转向绕行
                zone = '紧急'
                w_avoid = side * avoid_w
                v_avoid = self._p('v_creep')
            elif front <= avoid_near:
                # 近距区：大幅转向 + 保持前进（不低于 v_turn_min）
                zone = '近距'
                w_avoid = side * avoid_w
                v_avoid = max(self._p('v_turn_min'),
                              v_max * front / avoid_near)
            else:
                # 预警区：小角度转向（越远越小）+ 轻微减速，丝滑提前变道
                zone = '预警'
                # k 钳制到 [0,1]：front > avoid_far 时（记忆尚未释放的瞬间）
                # 原公式会得到负 k → 角速度反向 + 速度超上限（09-29 实测 bug）
                k = (avoid_far - front) / max(avoid_far - avoid_near, 1e-6)
                k = min(max(k, 0.0), 1.0)
                w_avoid = side * avoid_w * (0.35 + 0.45 * k)
                v_avoid = v_max * (1.0 - 0.3 * k)
            # 前视点偏移继续建立（越近建得越快），让变道弧线更舒展
            off_rate = 0.008 + 0.012 * (1.0 - min(front / avoid_far, 1.0))
            off_max = self._p('avoid_off_max')
            self._avoid_off = max(-off_max, min(off_max,
                                                self._avoid_off + side * off_rate))
        else:
            # 障碍解除：偏移慢速回落（回线弱化）
            self._avoid_off *= self._p('recover_rate')
            if abs(self._avoid_off) < 0.02:
                self._avoid_off = 0.0

        # ---- 前视点巡线（含横向偏移）----
        sx, sy = self._seg_start
        dx, dy = tx - sx, ty - sy
        seg_len = math.hypot(dx, dy)
        if seg_len < 1e-6:
            aim_x, aim_y = tx, ty
        else:
            t = ((rx - sx) * dx + (ry - sy) * dy) / (seg_len * seg_len)
            la = min(self._p('look_ahead'), 0.6 * dist)
            t_look = min(1.0, t + la / seg_len)
            aim_x = sx + dx * t_look - dy / seg_len * self._avoid_off
            aim_y = sy + dy * t_look + dx / seg_len * self._avoid_off
        err = _wrap(math.atan2(aim_y - ry, aim_x - rx) - ryaw)
        w_track = max(-w_max, min(w_max, self._p('kp_lat') * err))

        if side != 0.0:
            # 避障角速度主导；巡线项弱化为 ±0.3 以内纠偏，防把车头拉回障碍
            w = w_avoid + max(-0.3, min(0.3, 0.3 * w_track))
            w = max(-w_max, min(w_max, w))
            v = v_avoid
        else:
            w = w_track
            v = v_max
            if abs(err) > 0.2:
                v *= 0.45                            # 偏航大：先降速修正

        if dist < 0.5:
            v = max(0.08, v * dist / 0.5)            # 接近目标：缓行防冲过点
        self._publish_cmd(v, w)

        msg = f'第{self._idx + 1}点 直线行驶 剩余 {dist:.2f} m'
        if side != 0.0:
            msg += (f' | [{zone}]前方障碍 {front:.2f} m，'
                    f'向{"左" if side > 0 else "右"}绕行')
        self._publish_status(dist=dist, front=front, msg=msg)

    # ================= 超时检查（1Hz）=================
    def _check_timeouts(self):
        if self._phase == 'idle' or self._goal_t0 is None:
            return
        if time.monotonic() - self._goal_t0 > self._p('goal_timeout'):
            self.get_logger().warn(
                f'第 {self._idx + 1} 点巡线超时，跳过该点继续')
            self._publish_cmd(0.0, 0.0)
            self._idx += 1
            self._begin_segment()

    # ================= 激光分区（前方 ±30° / 左右）=================
    @staticmethod
    def _pick_avoid_side(goal_bearing, left_d, right_d, avoid_near):
        """选绕行侧：最小转弯优先（2026-09-29 修复）。

        优先往**目标点所在的一侧**绕——转弯量最小、绕完顺路直达下一点；
        该侧有足够走廊（≥ avoid_near）或并不比另一侧窄才选。目标侧明显
        被堵（比如那边是墙）时，才退回"往宽的一侧绕"。

        旧逻辑只比左右宽度、不看目标方位：目标在左、右侧稍宽时会朝墙
        的方向绕，越绕越死（09-29 用户实测"明显左转一点就能绕开，
        却先向右绕进墙"）。
        """
        prefer = 1.0 if goal_bearing >= 0.0 else -1.0
        prefer_d = left_d if prefer > 0.0 else right_d
        other_d = right_d if prefer > 0.0 else left_d
        if prefer_d >= avoid_near or prefer_d >= other_d:
            return prefer
        return 1.0 if left_d > right_d else -1.0

    def _route_obstacle(self, half_fov=math.radians(30.0), max_range=4.0):
        inf = float('inf')
        scan = self._scan
        if scan is None:
            return inf, inf, inf
        r = np.asarray(scan.ranges, dtype=np.float64)
        if r.size == 0:
            return inf, inf, inf
        angles = scan.angle_min + np.arange(r.size) * scan.angle_increment
        valid = np.isfinite(r)
        valid &= (r >= max(scan.range_min, 0.0))
        valid &= (r < min(scan.range_max, max_range))
        fa = valid & (np.abs(angles) <= half_fov)
        la = valid & (angles > half_fov) & (angles <= math.pi / 2)
        ra = valid & (angles < -half_fov) & (angles >= -math.pi / 2)
        f = float(r[fa].min()) if fa.any() else inf
        l = float(r[la].min()) if la.any() else inf
        rr = float(r[ra].min()) if ra.any() else inf
        return f, l, rr

    # ================= 输出 =================
    def _publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def _publish_status(self, phase=None, dist=None, front=None, msg='',
                        force=False):
        """进度回报（节流 2Hz，关键事件 force 立即发）。"""
        now = time.monotonic()
        if not force and now - self._last_status_t < 0.5:
            return
        self._last_status_t = now
        d = {'phase': phase or self._phase, 'idx': self._idx,
             'dist': round(dist, 2) if dist is not None else None,
             'front': round(front, 2) if (front is not None
                                         and front != float('inf')) else None,
             'success': self._success, 'total': self._total, 'msg': msg}
        m = String()
        m.data = json.dumps(d, ensure_ascii=False)
        self.status_pub.publish(m)


def main():
    rclpy.init()
    node = RouteFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        try:
            node._publish_cmd(0.0, 0.0)   # Ctrl+C 退出前停车
        except Exception:
            pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
