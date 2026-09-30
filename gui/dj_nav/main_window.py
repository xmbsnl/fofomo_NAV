"""DJ-NAV 1.0 主窗口（真机专用版）。

从 DIJUN Navigator（01-simulation/dt01_sim/scripts/nav_gui）裁剪而来：
  · 移除全部仿真功能（Gazebo 启停 / 世界与车型选择 / 本机 SLAM·Nav2 进程）
  · 修复 P0：
      1. 所有 SSH 调用一律走后台线程（原来有 4 处在主线程同步执行，
         保存地图时界面反复卡死、还会与其它 SSH 任务抢锁）
      2. btn_sim 连上后不再显示「断开」（与红色「断开连接」按钮语义冲突）
  · 改进 P1：
      3. 顶栏最右新增全局急停 E-STOP（取消导航+停队列+发零速，X 键同效）
      4. 顶栏减负：地图三件套移入右侧「控制」页；车型/世界下拉随仿真一起移除
      5. 状态监控组从面板底部提到最上面（开车时一眼可见）

布局：
  顶部工具栏（启动 / 模式切换 / 急停）
  中央大地图（缩放/平移/点选）
  右侧控制面板（地图 / 状态 / 遥控 / 导航 / 传感器）
  底部日志 + 状态栏

研发：fofomo
"""
import json
import math
import os
import re
import shutil
import threading
import time

from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, QSettings, QEvent
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import (QMainWindow, QWidget, QSplitter, QVBoxLayout,
                             QHBoxLayout, QGridLayout, QGroupBox, QPushButton,
                             QLabel, QSlider, QListWidget, QListWidgetItem,
                             QInputDialog, QMessageBox, QPlainTextEdit,
                             QFrame, QAbstractItemView,
                             QComboBox, QScrollArea, QTabWidget,
                             QApplication, QLineEdit, QTextEdit)

from .map_view import MapView
from .sensor_panel import SensorPanel
from .tuning_panel import TuningPanel
from .pose_refiner import refine_pose
from .profiles import BASE_DIR, MAPS_DIR, SAVED_MAPS_DIR, REAL
from .remote_host import RemoteHost


class SensorWindow(QWidget):
    """传感器面板的独立窗口容器。

    关闭窗口时把面板交还给主窗口标签页，而不是让它随窗口一起销毁。
    """

    def __init__(self, panel, on_close):
        super().__init__()
        self._panel = panel
        self._on_close = on_close
        self.setWindowTitle('传感器监控')
        # 与主窗口统一的暗色背景，否则弹出后中央区域呈现默认纯白/纯黑，
        # 看起来像"面板内容丢了"。
        self.setStyleSheet('QWidget { background-color: #14171d; color: #e6edf3; }')
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(panel)


LOGO_FILE = os.path.join(BASE_DIR, 'logo.jpg')

DARK_QSS = """
QMainWindow, QWidget { background: #23262d; color: #d8dce3; font-size: 13px; }
QGroupBox { border: 1px solid #3a3f4a; border-radius: 6px; margin-top: 12px;
            padding-top: 8px; font-weight: bold; color: #9ab; }
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
QPushButton { background: #3a4152; border: 1px solid #4a5265; border-radius: 5px;
              padding: 7px 10px; color: #e8ecf2; }
QPushButton:hover { background: #475069; }
QPushButton:pressed { background: #2f6db3; }
QPushButton:checked { background: #2f6db3; border-color: #5aa0e8; }
QPushButton:disabled { background: #2c2f36; color: #666; }
QPushButton#danger { background: #6b3038; border-color: #8a4048; }
QPushButton#danger:hover { background: #7d3a44; }
QPushButton#startBtn { background: #2e6b3a; border-color: #3d8a4e; font-weight: bold;
                       padding: 9px 14px; }
QPushButton#startBtn:hover { background: #3a7f48; }
QPushButton#stopBtn { background: #6b3038; border-color: #8a4048; font-weight: bold;
                      padding: 9px 14px; }
QPushButton#stopBtn:hover { background: #7d3a44; }
QPushButton#modeBtn { font-weight: bold; padding: 8px 18px; }
QPushButton#estopBtn { background: #a02530; border: 2px solid #d04555; border-radius: 6px;
                       font-weight: bold; padding: 10px 18px; color: #ffffff; }
QPushButton#estopBtn:hover { background: #c02a38; }
QPushButton#estopBtn:pressed { background: #7d1c26; }
QListWidget { background: #1c1f25; border: 1px solid #3a3f4a; border-radius: 5px; }
QListWidget::item { padding: 5px; }
QListWidget::item:selected { background: #2f6db3; }
QPlainTextEdit { background: #17191e; border: 1px solid #3a3f4a; color: #9fd0a0;
                 font-family: monospace; font-size: 12px; }
QSlider::groove:horizontal { height: 6px; background: #3a3f4a; border-radius: 3px; }
QSlider::handle:horizontal { width: 16px; margin: -6px 0; border-radius: 8px;
                             background: #5aa0e8; }
QLabel#statusVal { color: #7fd0ff; font-family: monospace; }
QLabel#logTitle { color: #9ab; font-weight: bold; padding-left: 4px; }
QPushButton#logBtn { padding: 3px 9px; font-size: 12px; }
QLabel#title { font-size: 16px; font-weight: bold; color: #5aa0e8; }
QLabel#credits { color: #6a7280; font-size: 11px; }
QFrame#sep { background: #3a3f4a; max-height: 1px; }
QScrollArea { border: none; background: transparent; }
QScrollBar:vertical { background: #23262d; width: 10px; margin: 0; }
QScrollBar::handle:vertical { background: #3a4152; border-radius: 5px;
                             min-height: 24px; }
QScrollBar::handle:vertical:hover { background: #4a5265; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QScrollBar:horizontal { background: #23262d; height: 10px; margin: 0; }
QScrollBar::handle:horizontal { background: #3a4152; border-radius: 5px;
                               min-width: 24px; }
QScrollBar::handle:horizontal:hover { background: #4a5265; }
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal { width: 0; }
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical,
QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal { background: none; }
"""


class MainWindow(QMainWindow):
    # 后台线程（SSH / ROS 回调）写日志时走的转发信号。
    # Qt 控件只能在主线程碰，工作线程直接 appendPlainText 会随机段错误崩溃。
    log_signal = pyqtSignal(str)

    def __init__(self, bridge):
        super().__init__()
        self.bridge = bridge
        self.signals = bridge.signals

        self.setWindowTitle('DJ-NAV 1.0 · 真机控制台')
        self.resize(1500, 950)
        self.setMinimumSize(860, 560)

        self.current_mode = None     # None | 'mapping' | 'navigation'
        self.navigating = False
        self.waypoints = []
        self.maps = []               # [{'name': str, 'yaml': str, 'dir': str}, ...]
        self.current_map = None      # 当前地图名（None = 未找到地图）
        # 日志区常量：默认高度与最大保留行数（显示层与完整副本保持一致）。
        self._LOG_DEFAULT_H = 190
        self._LOG_MAX_BLOCKS = 500

        # 真机档案：本版本唯一的运行方式（无仿真）。
        self.profile = REAL
        self.remote = None           # RemoteHost：通过 SSH 操作工控机
        self._real_connected = False    # 工控机数据是否真的过来了
        # 日志：_log_full 保存未截断的完整原文，供「复制全部」使用。
        self._main_thread = threading.current_thread()
        self.log_signal.connect(self._log_impl)
        self._log_full = []
        self._all_points = {}        # {'地图名': [点位...]} 持久化存储
        # 电子围栏：{'地图名': [{'name','points':[[x,y],...]}, ...]} 持久化
        self._all_keepout = {}
        self._keepout_zones = []     # 当前地图的禁区列表（下发/打点拦截用）
        # 打点任务队列（提前打点 → 开始导航顺序执行 → 停止导航随时急停）
        self._nav_queue = []
        self._queue_idx = 0
        self._queue_active = False
        self._goal_retries = 0
        self._max_goal_retries = 1
        self._pending_remote_mode = None
        self._goal_timeout = 90.0
        self._nav_total_count = 0
        self._nav_success_count = 0
        self._init_pose_set = False
        self._auto_pose_attempts = 0
        self._saving_map_name = None    # 正在保存的地图名（防止重复触发保存）
        self._last_path_rx = 0.0        # 最近一次收到 /plan 的时间（幽灵路径清理用）
        # ---- 巡线行驶（打点连线）：控制算法在 route_follower 节点侧闭环 ----
        # 真车安全架构：节点由 GUI 部署到工控机上运行，GUI 只发 路线/心跳/急停；
        # GUI 断链或崩溃时节点看门狗自动零速停车（心跳超时保护）。
        self._route_active = False       # 巡线任务进行中（心跳随之发送）
        self._route_pts = []             # 待下发路线 [(x, y, yaw), ...]
        self._route_got_status = False   # 是否已收到节点回报（检测节点在跑）
        self._route_deploying = False    # 节点正在部署（防重复点击）
        self._ROUTE_SCRIPT = os.path.join(BASE_DIR, 'route_follower_node.py')

        self._build_ui()
        self._connect_ros()
        self._scan_maps()

        # ROS spin 定时器：常驻 executor（不做 add/remove_node，稳定不丢回调）。
        self._rclpy_mod = __import__('rclpy')
        _exec_mod = __import__('rclpy.executors', fromlist=['SingleThreadedExecutor'])
        self._executor = _exec_mod.SingleThreadedExecutor()
        self._executor.add_node(self.bridge)
        self._spin_err_logged = False
        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self._spin_ros)
        self.spin_timer.start(20)

        # 状态刷新定时器（位姿/TF）
        self.pose_timer = QTimer(self)
        self.pose_timer.timeout.connect(self._refresh_pose)
        self.pose_timer.start(100)

        # 单点导航超时看门狗
        self._goal_start_time = None
        self.watchdog_timer = QTimer(self)
        self.watchdog_timer.timeout.connect(self._check_goal_timeout)
        self.watchdog_timer.start(1000)

        # 手动控制发送定时器（20Hz）
        self.teleop_timer = QTimer(self)
        self.teleop_timer.timeout.connect(self._send_teleop)
        self.teleop_timer.start(50)
        self.keys_down = set()

        # 巡线心跳定时器（2Hz）：告诉工控机上的 route_follower 节点 GUI 还在线，
        # 节点超时收不到心跳会自动零速停车（断链保护）。开始巡线时才启动。
        self.route_hb_timer = QTimer(self)
        self.route_hb_timer.timeout.connect(self._send_route_hb)

        # 全局键盘遥控事件过滤器：挂到 QApplication 上，先于所有控件收到按键
        QApplication.instance().installEventFilter(self)

        # 初始化真机通道（SSH + 时钟 + 遥控话题）
        self._init_remote()
        self._update_mode_ui()
        self.log('DJ-NAV 1.0（真机版）已启动。请点「启动机器人」连接工控机。')

    # ================================================== 真机通道初始化
    def _init_remote(self):
        """建立 SSH 通道，同步时钟与遥控话题，并后台探测工控机状态。"""
        self.remote = RemoteHost(self.profile.remote_user, self.profile.remote_host,
                                 self.profile.remote_ws,
                                 self.profile.remote_maps_dir,
                                 log=self.log)
        # 1) 时钟：真机用系统时钟
        try:
            self.bridge.set_use_sim_time(False)
        except Exception as e:                                  # noqa: BLE001
            self.log(f'设置时钟失败: {e}')
        # 2) 遥控话题：/cmd_vel_teleop 经 safety_mux 仲裁（急停/防撞不失效）
        try:
            self.bridge.set_cmd_vel_topic(self.profile.cmd_vel_topic)
        except Exception as e:                                  # noqa: BLE001
            self.log(f'设置遥控话题失败: {e}')

        # 3) 后台探测（绝不阻塞主线程）
        def _probe_remote():
            # ⚠️ 主线程绝不发 ssh：每条都是一次远程命令，同步执行会把界面卡住
            # 好几秒，也会和用户的下一步操作（点「启动机器人」）抢 SSH 通道。
            self.remote.ensure_maps_dir()
            if not self.remote.is_reachable():
                self.log(f'[警告] 连不上工控机 {self.remote.target}：'
                         f'请检查同一 WiFi 网段 / 免密登录（ssh-copy-id）')
            elif self.remote.is_running():
                # 打开 GUI 就有数据，用户会以为"断开/清理功能失效了"。
                # 实际大多是上次异常退出（崩溃/强杀）留下的车上进程——
                # 这里必须把原因和两个出路讲清楚，别只报一句"在运行"。
                self.log('⚠⚠ 检测到工控机上已有机器人进程在运行')
                self.log('    （多为上次界面异常退出留下的，不是新启动的）')
                self.log('  · 想接着用：直接点「建图模式/导航模式」，GUI 会自动接管')
                self.log('  · 想清干净重来：点右上红色「断开连接」')

        threading.Thread(target=_probe_remote, daemon=True).start()

    # ================================================== 后台任务
    def _remote_task(self, fn, on_done):
        """把耗时的 SSH 操作丢到后台线程，完成后回 Qt 主线程回调。

        SSH 一条命令 5~20s，串在 Qt 主线程上界面就冻住 —— 用户会以为
        程序死了/超时了（实际上后台可能已经成功）。这里用
        「工作线程 + QTimer 主线程轮询」模式：结果安全地回到主线程。
        """
        box = {}

        def worker():
            try:
                box['result'] = fn()
            except Exception as e:                              # noqa: BLE001
                box['result'] = (False, str(e))
                box['error'] = True

        th = threading.Thread(target=worker, daemon=True)
        th.start()

        def poll():
            if th.is_alive():
                QTimer.singleShot(150, poll)
                return
            res = box.get('result')
            try:
                on_done(res)
            except Exception as e:                              # noqa: BLE001
                self.log(f'[远程] 回调异常: {e}')

        QTimer.singleShot(150, poll)

    def _async_stop_remote(self):
        """后台停掉工控机上的系统（不阻塞界面，不关心结果）。"""

        def worker():
            try:
                if self.remote.is_running():
                    self.remote.stop()
            except Exception as e:                              # noqa: BLE001
                self.log(f'[远程] 停止异常: {e}')

        threading.Thread(target=worker, daemon=True).start()

    def _disconnect_robot(self):
        """「断开连接」按钮：把工控机上的机器人进程全部杀干净。"""
        if not self.remote:
            QMessageBox.information(self, '断开连接', '未建立远程连接')
            return
        ret = QMessageBox.question(
            self, '断开机器人连接',
            '将杀掉工控机上全部机器人进程：\n'
            '  底盘驱动、雷达、IMU、EKF、建图/导航\n\n'
            '未保存的建图结果会丢失（需先点「另存地图」）。\n'
            '确定断开吗？',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if ret != QMessageBox.Yes:
            return
        self.btn_disconnect.setEnabled(False)
        self.st_sim.setText('断开中…')
        self.log('正在断开工控机连接（杀掉全部机器人进程）...')
        # SSH 要几十秒，走后台线程；结果回到主线程再动控件
        self._remote_task(self.remote.disconnect, self._on_disconnected)

    def _on_disconnected(self, result):
        ok, msg = (result if isinstance(result, tuple)
                   else (False, str(result)))
        self.btn_disconnect.setEnabled(True)
        self.log(f'断开连接：{msg}')
        if not ok:
            QMessageBox.warning(self, '断开失败', msg)
            self.st_sim.setText('未运行')
            return
        # 复位界面：车已经停了，不该再显示"运行中/已连接"
        self.current_mode = None
        self._real_connected = False
        self.btn_sim.setChecked(False)
        self.btn_sim.setText('启动机器人')
        self.st_sim.setText('未连接')
        self.st_network.setText('已断开工控机')
        self._update_mode_ui()
        self.log('已断开。重新作业请点「启动机器人」。')

    def _adopt_remote_mode(self, mode):
        """接管车上已在跑的模式：不重启进程，GUI 直接进入监控状态。

        典型场景：用户在终端手动起了建图，点「建图」按钮时不应重启
        （会丢掉已建未保存的图），应该直接接管。
        """
        self.current_mode = mode
        self.map_view.reset_fit()
        self.btn_sim.setChecked(True)
        self.btn_sim.setText('停止机器人')
        self.st_sim.setText('运行中')
        what = '建图' if mode == 'mapping' else '导航'
        self.log(f'{what}模式已接管（工控机上本来就在跑，未重启）')
        if mode == 'mapping':
            self.log('可以直接点「另存地图」保存当前建图结果')
        else:
            self._schedule_auto_init_pose()
        self.log(f'遥控发往 {self.profile.cmd_vel_topic}（GUI 遥控按钮已自动切换）')
        QTimer.singleShot(1500, self._check_real_link)
        self._update_mode_ui()

    def _on_remote_started(self, result):
        """后台启动（或重启）完成后的界面收尾。result=(ok, msg)。"""
        ok, msg = (result if isinstance(result, tuple) else (False, str(result)))
        if not ok:
            QMessageBox.warning(self, '启动失败', msg)
            self.btn_sim.setChecked(False)
            self.btn_sim.setText('启动机器人')
            self.st_sim.setText('未运行')
            self._update_mode_ui()
            return

        # 车上本来就在跑 → 接管（不重启，避免丢掉未保存的建图结果）
        if isinstance(msg, str) and msg.startswith('adopt:'):
            which = msg.split(':', 1)[1]
            if which in ('slam', 'nav'):
                self._adopt_remote_mode('mapping' if which == 'slam'
                                        else 'navigation')
            else:
                # 只起了机器人，没进任何模式
                self.btn_sim.setChecked(True)
                self.btn_sim.setText('停止机器人')
                self.st_sim.setText('运行中')
                self.log('机器人已在运行（工控机），可以点「建图」或「导航」')
                QTimer.singleShot(1500, self._check_real_link)
            self._update_mode_ui()
            return

        mode = self._pending_remote_mode or 'idle'
        if mode == 'idle':
            # 只起机器人：不设 current_mode，等用户点建图/导航
            self.btn_sim.setChecked(True)
            self.btn_sim.setText('停止机器人')
            self.st_sim.setText('运行中')
            self._reveal_sensor_panel()
            self.log('机器人已启动（工控机）。接下来：')
            self.log('  · 点「建图模式」→ 遥控绕场 → 「另存地图」')
            self.log('  · 或点「导航模式」→ 在地图上点初始位姿 → 点目标点')
            QTimer.singleShot(4000, self._check_real_link)
            self._update_mode_ui()
            return
        self.current_mode = mode
        self.map_view.reset_fit()
        self.btn_sim.setChecked(True)
        self.btn_sim.setText('停止机器人')
        self.st_sim.setText('运行中')
        if mode == 'mapping':
            self.log('建图模式已启动（工控机）：用遥控开车绕场，'
                     '速度 ≤0.3 m/s、走闭合回路，完成后点「另存地图」')
        else:
            map_file = self._remote_map_path()
            self.log(f'导航模式已启动（工控机）：'
                     f'{os.path.basename(map_file) if map_file else "(默认地图)"}')
            self._schedule_auto_init_pose()
        self.log(f'遥控发往 {self.profile.cmd_vel_topic}（GUI 遥控按钮已自动切换）')
        QTimer.singleShot(4000, self._check_real_link)
        self._update_mode_ui()

    # ================================================== UI 构建
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        # ---- 顶部工具栏（减负后：启动 / 模式 / 全图 / 急停） ----
        top = QHBoxLayout()
        logo = QLabel()
        if os.path.exists(LOGO_FILE):
            pm = QPixmap(LOGO_FILE)
            if not pm.isNull():
                logo.setPixmap(pm.scaled(34, 34, Qt.KeepAspectRatio,
                                         Qt.SmoothTransformation))
        logo.setFixedSize(34, 34)
        logo.setAlignment(Qt.AlignCenter)
        top.addWidget(logo)
        top.addSpacing(6)
        title = QLabel('DJ-NAV 1.0')
        title.setObjectName('title')
        top.addWidget(title)
        top.addSpacing(20)

        self.btn_sim = QPushButton('启动机器人')
        self.btn_sim.setCheckable(True)
        self.btn_sim.clicked.connect(self._toggle_robot)
        top.addWidget(self.btn_sim)

        frame = QFrame(); frame.setObjectName('sep'); top.addWidget(frame)

        self.btn_mapping = QPushButton('建图模式')
        self.btn_mapping.setObjectName('modeBtn')
        self.btn_mapping.setCheckable(True)
        self.btn_mapping.clicked.connect(lambda: self._switch_mode('mapping'))
        top.addWidget(self.btn_mapping)

        self.btn_navigation = QPushButton('导航模式')
        self.btn_navigation.setObjectName('modeBtn')
        self.btn_navigation.setCheckable(True)
        self.btn_navigation.clicked.connect(lambda: self._switch_mode('navigation'))
        top.addWidget(self.btn_navigation)

        self.btn_disconnect = QPushButton('断开连接')
        self.btn_disconnect.setObjectName('danger')
        self.btn_disconnect.setToolTip(
            '杀掉工控机上全部机器人进程（底盘驱动、雷达、IMU、建图/导航），彻底断开。\n'
            '未保存的建图会丢失，请先另存地图。')
        self.btn_disconnect.clicked.connect(self._disconnect_robot)
        top.addWidget(self.btn_disconnect)

        top.addStretch(1)

        self.btn_fit = QPushButton('全图')
        self.btn_fit.clicked.connect(lambda: self.map_view.fit_map())
        top.addWidget(self.btn_fit)

        # P1：全局急停 —— 任何时候点它都立即停车（取消导航 + 停队列 + 零速）。
        # 建图倒车快撞墙时，用户不该需要回忆"焦点在哪、该按哪个按钮"。
        self.btn_estop = QPushButton('⛔ 急停 E-STOP')
        self.btn_estop.setObjectName('estopBtn')
        self.btn_estop.setToolTip(
            '全局急停：立即取消导航、清空任务队列并发送零速。\n'
            '键盘 X 键等效（不受焦点影响）。')
        self.btn_estop.clicked.connect(self._emergency_stop)
        top.addWidget(self.btn_estop)

        root.addLayout(top)

        # ---- 中央：地图 + 右侧面板 ----
        splitter = QSplitter(Qt.Horizontal)

        self.map_view = MapView()
        self.map_view.pose_picked.connect(self._on_pose_picked)
        self.map_view.pick_rejected.connect(self._on_pick_rejected)
        self.map_view.pick_blocked.connect(self._on_pick_blocked)
        self.map_view.mouse_moved.connect(self._on_mouse_moved)
        self.map_view.keepout_drawn.connect(self._on_keepout_drawn)
        self.map_view.setMinimumWidth(180)
        splitter.addWidget(self.map_view)

        # 右侧面板：控制 / 传感器 两个标签页，共用一个滚动列
        self.right_tabs = QTabWidget()
        self.right_tabs.setMinimumWidth(240)
        self.right_tabs.setMaximumWidth(420)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(2, 2, 2, 2)
        right_layout.setSpacing(8)
        # P1：状态监控提到最上面 —— 开车时电量/速度/网络一眼可见，
        # 不用再滚到面板底部去找。地图管理组（原顶栏三件套）也收拢到这里。
        right_layout.addWidget(self._build_map_group())
        right_layout.addWidget(self._build_status_group())
        right_layout.addWidget(self._build_teleop_group())
        right_layout.addWidget(self._build_nav_group())
        right_layout.addStretch(1)

        right_scroll = QScrollArea()
        right_scroll.setWidgetResizable(True)
        right_scroll.setFrameShape(QFrame.NoFrame)
        right_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        right_scroll.setWidget(right)
        self.right_tabs.addTab(right_scroll, '控制')

        # 传感器监控页：必须包进 QScrollArea（面板 minimumSizeHint 高达 912px，
        # 直接塞进 QTabWidget 会把整个上方区顶到 ~950px，日志区被挤没）。
        self.sensor_panel = SensorPanel(robot_type='dt01')
        self.sensor_scroll = QScrollArea()
        self.sensor_scroll.setWidgetResizable(True)
        self.sensor_scroll.setFrameShape(QFrame.NoFrame)
        self.sensor_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.sensor_scroll.setWidget(self.sensor_panel)
        self.right_tabs.addTab(self.sensor_scroll, '传感器')
        self.sensor_win = None      # 弹出后的独立窗口（None = 内嵌在标签页）
        self.sensor_panel.popout_requested.connect(self._toggle_sensor_popout)

        # 调参页：现场调巡线/Nav2 参数（点「设」经 SSH 下发 ros2 param set）
        self.tuning_panel = TuningPanel()
        self.tuning_scroll = QScrollArea()
        self.tuning_scroll.setWidgetResizable(True)
        self.tuning_scroll.setFrameShape(QFrame.NoFrame)
        self.tuning_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.tuning_scroll.setWidget(self.tuning_panel)
        self.right_tabs.addTab(self.tuning_scroll, '调参')
        self.tuning_panel.param_set_requested.connect(self._on_param_set_requested)

        splitter.addWidget(self.right_tabs)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 0)

        # ---- 底部日志 ----
        main_splitter = QSplitter(Qt.Vertical)
        main_splitter.addWidget(splitter)

        self.log_panel = QWidget()
        log_layout = QVBoxLayout(self.log_panel)
        log_layout.setContentsMargins(0, 2, 0, 0)
        log_layout.setSpacing(3)
        log_layout.addLayout(self._build_log_toolbar())

        self.log_box = QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumBlockCount(self._LOG_MAX_BLOCKS)
        self.log_box.setMinimumHeight(44)
        self.log_box.setMaximumHeight(9999)
        log_layout.addWidget(self.log_box, 1)

        main_splitter.addWidget(self.log_panel)
        main_splitter.setStretchFactor(0, 1)
        main_splitter.setStretchFactor(1, 0)
        main_splitter.setCollapsible(0, False)
        main_splitter.setCollapsible(1, False)
        main_splitter.setHandleWidth(8)
        root.addWidget(main_splitter, 1)

        self._main_splitter = main_splitter
        self._restore_splitter_sizes()

        # ---- 状态栏 ----
        self.status_mouse = QLabel('地图: -, -')
        self.status_pose = QLabel('机器人: -')
        self.status_nav = QLabel('导航: 未启动')
        sb = self.statusBar()
        sb.addWidget(self.status_mouse, 1)
        sb.addWidget(self.status_pose, 1)
        sb.addWidget(self.status_nav, 2)
        self.credits = QLabel('研发：fofomo')
        self.credits.setObjectName('credits')
        sb.addPermanentWidget(self.credits)

    def _build_map_group(self):
        """P1：地图管理三件套（原顶栏），收拢到控制页顶部。"""
        box = QGroupBox('地图')
        v = QVBoxLayout(box)

        row = QHBoxLayout()
        self.map_combo = QComboBox()
        self.map_combo.setMinimumWidth(150)
        self.map_combo.setToolTip('已保存的地图列表（建图保存后自动出现）')
        self.map_combo.currentIndexChanged.connect(self._on_map_combo)
        row.addWidget(self.map_combo, 1)
        btn_refresh = QPushButton('刷新')
        btn_refresh.setToolTip('重新扫描本地与工控机的地图目录')
        btn_refresh.clicked.connect(self._sync_remote_maps)
        row.addWidget(btn_refresh)
        v.addLayout(row)

        self.btn_save_map = QPushButton('💾 另存地图')
        self.btn_save_map.setToolTip('把当前建图结果保存为一张新地图（需在建图模式）')
        self.btn_save_map.clicked.connect(self._save_map_dialog)
        v.addWidget(self.btn_save_map)
        return box

    def _build_teleop_group(self):
        box = QGroupBox('手动遥控')
        g = QGridLayout(box)

        self.teleop_buttons = {}
        layout_def = [
            ('forward', '↑ (W)', 0, 1),
            ('left', '← (A)', 1, 0),
            ('stop', '■ (X)', 1, 1),
            ('right', '→ (D)', 1, 2),
            ('back', '↓ (S)', 2, 1),
        ]
        for key, text, r, c in layout_def:
            btn = QPushButton(text)
            btn.setFocusPolicy(Qt.NoFocus)
            if key == 'stop':
                btn.setObjectName('danger')
                btn.clicked.connect(self._teleop_stop)
            else:
                btn.pressed.connect(lambda k=key: self._teleop_key_down(k))
                btn.released.connect(lambda k=key: self._teleop_key_up(k))
            self.teleop_buttons[key] = btn
            g.addWidget(btn, r, c)

        g.addWidget(QLabel('线速度'), 3, 0)
        self.slider_linear = QSlider(Qt.Horizontal)
        self.slider_linear.setRange(5, 100)      # 0.05~1.00 m/s
        self.slider_linear.setValue(30)
        self.slider_linear.setFocusPolicy(Qt.NoFocus)
        g.addWidget(self.slider_linear, 3, 1, 1, 2)
        self.lbl_linear = QLabel('0.30 m/s')
        self.lbl_linear.setObjectName('statusVal')
        self.slider_linear.valueChanged.connect(
            lambda v: self.lbl_linear.setText(f'{v/100:.2f} m/s'))
        self.lbl_linear.setMinimumWidth(70)
        g.addWidget(self.lbl_linear, 4, 0, 1, 3, Qt.AlignRight)

        g.addWidget(QLabel('角速度'), 5, 0)
        self.slider_angular = QSlider(Qt.Horizontal)
        self.slider_angular.setRange(10, 200)    # 0.10~2.00 rad/s
        self.slider_angular.setValue(60)
        self.slider_angular.setFocusPolicy(Qt.NoFocus)
        g.addWidget(self.slider_angular, 5, 1, 1, 2)
        self.lbl_angular = QLabel('0.60 rad/s')
        self.lbl_angular.setObjectName('statusVal')
        self.slider_angular.valueChanged.connect(
            lambda v: self.lbl_angular.setText(f'{v/100:.2f} rad/s'))
        g.addWidget(self.lbl_angular, 6, 0, 1, 3, Qt.AlignRight)
        return box

    def _build_nav_group(self):
        box = QGroupBox('导航控制')
        v = QVBoxLayout(box)

        # 行1：设定初始位姿 + 全局定位 + 打点
        row1 = QHBoxLayout()
        self.btn_set_pose = QPushButton('设定初始位姿')
        self.btn_set_pose.setCheckable(True)
        self.btn_set_pose.clicked.connect(self._toggle_set_pose)
        row1.addWidget(self.btn_set_pose)
        self.btn_global_loc = QPushButton('全局定位')
        self.btn_global_loc.setToolTip('机器人位置未知时，让 AMCL 自动全局搜索定位（无需手动点选）')
        self.btn_global_loc.clicked.connect(self._do_global_localization)
        row1.addWidget(self.btn_global_loc)
        self.btn_add_goal = QPushButton('＋ 打点')
        self.btn_add_goal.setCheckable(True)
        self.btn_add_goal.setToolTip('在地图上点击（拖动可设朝向）加入任务队列，'
                                     '相邻点自动连成最短直线，机器人只走画出的连线')
        self.btn_add_goal.clicked.connect(self._toggle_add_goal)
        row1.addWidget(self.btn_add_goal)
        v.addLayout(row1)

        # 行2：开始导航 / 停止导航（行进途中可随时停止）
        row2 = QHBoxLayout()
        self.btn_start_nav = QPushButton('▶ 开始导航')
        self.btn_start_nav.setObjectName('startBtn')
        self.btn_start_nav.setToolTip('从当前位置出发：先原地对正下一点方向，'
                                      '再沿打点连线直线行驶（仅避障时短暂偏线）')
        self.btn_start_nav.clicked.connect(self._start_queue)
        row2.addWidget(self.btn_start_nav)
        self.btn_stop_nav = QPushButton('■ 停止导航')
        self.btn_stop_nav.setObjectName('stopBtn')
        self.btn_stop_nav.setToolTip('立即取消当前导航并停车（行进中也可随时停止）')
        self.btn_stop_nav.clicked.connect(self._stop_navigation)
        row2.addWidget(self.btn_stop_nav)
        v.addLayout(row2)

        # 行3：电子围栏（在地图上画多边形禁区，巡线时自动避开）
        row3 = QHBoxLayout()
        self.btn_draw_keepout = QPushButton('＋禁区')
        self.btn_draw_keepout.setCheckable(True)
        self.btn_draw_keepout.setToolTip('进入禁区绘制模式：左键逐点添加，右键闭合（≥3 点）')
        self.btn_draw_keepout.clicked.connect(self._toggle_draw_keepout)
        row3.addWidget(self.btn_draw_keepout)
        self.btn_clear_keepout = QPushButton('清空禁区')
        self.btn_clear_keepout.setObjectName('danger')
        self.btn_clear_keepout.setToolTip('删除当前地图的全部禁区')
        self.btn_clear_keepout.clicked.connect(self._clear_keepout)
        row3.addWidget(self.btn_clear_keepout)
        v.addLayout(row3)

        # 任务队列
        self.lbl_queue = QLabel('任务队列（相邻点自动连线，开始后沿连线行驶）：')
        v.addWidget(self.lbl_queue)
        self.queue_list = QListWidget()
        self.queue_list.setSelectionMode(QAbstractItemView.SingleSelection)
        v.addWidget(self.queue_list)

        qrow = QHBoxLayout()
        self.btn_undo_point = QPushButton('↶ 撤销上一打点')
        self.btn_undo_point.setToolTip('移除任务队列中最近一次打点（正在执行的点除外）')
        self.btn_undo_point.clicked.connect(self._undo_last_point)
        qrow.addWidget(self.btn_undo_point)
        btn_clear = QPushButton('清空队列')
        btn_clear.clicked.connect(self._clear_queue)
        qrow.addWidget(btn_clear)
        btn_del_queue = QPushButton('删除选中')
        btn_del_queue.setObjectName('danger')
        btn_del_queue.clicked.connect(self._del_queue_item)
        qrow.addWidget(btn_del_queue)
        v.addLayout(qrow)

        # 已保存点位（双击加入队列）
        self.lbl_points = QLabel('已保存点位（双击加入队列）：')
        v.addWidget(self.lbl_points)
        self.point_list = QListWidget()
        self.point_list.setSelectionMode(QAbstractItemView.SingleSelection)
        self.point_list.itemDoubleClicked.connect(self._goto_point)
        v.addWidget(self.point_list)

        prow = QHBoxLayout()
        btn_add = QPushButton('保存当前位置为点位')
        btn_add.clicked.connect(self._add_point)
        prow.addWidget(btn_add)
        btn_del = QPushButton('删除选中')
        btn_del.setObjectName('danger')
        btn_del.clicked.connect(self._del_point)
        prow.addWidget(btn_del)
        v.addLayout(prow)
        return box

    def _build_status_group(self):
        box = QGroupBox('状态监控')
        g = QGridLayout(box)
        self.st_mode = QLabel('未启动')
        self.st_sim = QLabel('未运行')
        self.st_vel = QLabel('0.00 / 0.00')
        self.st_battery = QLabel('未接入')
        self.st_chassis = QLabel('未接入')
        self.st_network = QLabel('未连接工控机')
        for lbl in (self.st_mode, self.st_sim, self.st_vel,
                    self.st_battery, self.st_chassis, self.st_network):
            lbl.setObjectName('statusVal')
            lbl.setWordWrap(True)
        self.st_battery.setToolTip('实机：/battery_state（dt01_driver 发布）')
        self.st_network.setToolTip('实机：/dt01/chassis_status 心跳 + /dt01/network（预留）')
        g.addWidget(QLabel('当前模式'), 0, 0); g.addWidget(self.st_mode, 0, 1)
        g.addWidget(QLabel('运行状态'), 1, 0); g.addWidget(self.st_sim, 1, 1)
        g.addWidget(QLabel('速度(m/s,rad/s)'), 2, 0); g.addWidget(self.st_vel, 2, 1)
        g.addWidget(QLabel('电量'), 3, 0); g.addWidget(self.st_battery, 3, 1)
        g.addWidget(QLabel('底盘'), 4, 0); g.addWidget(self.st_chassis, 4, 1)
        g.addWidget(QLabel('网络'), 5, 0); g.addWidget(self.st_network, 5, 1)
        return box

    # ================================================== 调参下发
    def _on_param_set_requested(self, node, name, value):
        """调参面板点「设」：后台 SSH 在工控机上执行 ros2 param set。"""
        if not self.remote:
            self.log(f'调参：未连接工控机，无法下发 {node} {name}')
            return
        self.log(f'调参：下发 {node} {name} = {value:g} …')
        self._remote_task(
            lambda: self.remote.ros_param_set(node, name, value),
            lambda r: self._on_param_set_done(node, name, r))

    def _on_param_set_done(self, node, name, result):
        ok, msg = (result if isinstance(result, tuple) else (False, str(result)))
        if ok:
            self.log(f'调参：{node} {name} 已设置（{msg}）')
        else:
            self.log(f'调参失败：{node} {name} —— {msg}')

    # ================================================== 数据准入门控
    def _data_allowed(self):
        """是否允许把车上数据渲染到界面。

        GUI 的 ROS 节点从启动起就一直在订阅 /map、/scan、/plan、/odom 等
        话题 —— 这是 DDS 网络直达，**与是否点过"启动"无关**。所以只要
        车在跑（或同网段有别的机器在发），本机就会不停收到数据并画到界面上。
        这里做一道准入：必须已确认连上工控机（/odom、/scan 有数据）才渲染。
        """
        return bool(self._real_connected)

    # ================================================== ROS 信号连接
    def _connect_ros(self):
        s = self.signals
        s.map_received.connect(self._on_map_gated)
        s.scan_received.connect(self._on_scan)
        s.path_received.connect(self._on_path)
        s.cmd_vel_updated.connect(
            lambda lin, ang: self.st_vel.setText(f'{lin:+.2f} / {ang:+.2f}'))
        s.nav_state.connect(self._on_nav_state)
        s.goal_finished.connect(self._on_goal_finished)
        s.route_status.connect(self._on_route_status)
        s.battery_updated.connect(self._on_battery)
        s.chassis_updated.connect(self._on_chassis)
        s.network_updated.connect(self._on_network)
        s.log.connect(self.log)
        # 传感器监控
        s.pose_updated.connect(self._on_pose_signal)
        s.sonar_updated.connect(self._on_sonar)
        s.camera_cloud_updated.connect(self._on_camera_cloud)
        s.imu_updated.connect(self._on_imu)
        s.nav_decision.connect(self._on_nav_decision)
        s.avoid_decision.connect(self._on_avoid_decision)
        s.stop_reason.connect(self.sensor_panel.set_stop_reason)
        s.next_action.connect(self.sensor_panel.set_next_action)

    # ================================================== ROS 定时处理
    def _spin_ros(self):
        """50Hz 调度 ROS 回调（常驻 executor，无 add/remove 开销）。"""
        try:
            self._executor.spin_once(timeout_sec=0.02)
        except Exception as e:
            if not getattr(self, '_spin_err_logged', False):
                self._spin_err_logged = True
                self.log(f'⚠ ROS spin 异常(已自动恢复): {type(e).__name__}: {e}')

    def _check_goal_timeout(self):
        """单点导航超时看门狗：超时即取消并跳过该点，继续执行后续点。"""
        if self._route_active:
            return   # 巡线模式：单点超时由工控机上的 route_follower 自己管
        if not self._queue_active or self._goal_start_time is None:
            return
        elapsed = time.monotonic() - self._goal_start_time
        if elapsed < self._goal_timeout:
            return
        idx = self._queue_idx
        name = (self._nav_queue[idx]['name']
                if 0 <= idx < len(self._nav_queue) else '?')
        self.log(f'⏱ 第 {idx + 1} 点[{name}] 导航超时（{elapsed:.0f}s），'
                 f'取消该点并继续下一个')
        self._goal_start_time = None
        self.bridge.cancel_goal()
        self._goal_retries = 0
        self._queue_idx += 1
        QTimer.singleShot(1200, self._send_next_goal)

    def _refresh_pose(self):
        # 幽灵路径清理：/plan 只在导航进行中持续发布。超过 3s 没有新路径，
        # 说明界面上的绿线是上一次导航的残影 —— 典型场景是重启 GUI 后
        # 车上旧导航栈又补发了一次 /plan，用户看到的是"上次的幽灵路径"。
        if self._last_path_rx and time.monotonic() - self._last_path_rx > 3.0:
            self._last_path_rx = 0.0
            self.map_view.set_path([])
        pose = self.bridge.get_robot_pose()
        self._robot_pose = pose
        self.sensor_panel.update_pose(pose)
        if pose is not None and self._data_allowed():
            self.map_view.set_robot_pose(pose)
        if pose is not None:
            x, y, yaw = pose
            self.status_pose.setText(
                f'机器人: ({x:.2f}, {y:.2f})  {math.degrees(yaw):.0f}°')
            # 巡线路线可视化：起点跟随机器人实时位置，剩余连线一目了然
            if self._nav_queue:
                start = (self._queue_idx
                         if (self._queue_active and self._route_active) else 0)
                pts = [(x, y)]
                pts += [(p['x'], p['y']) for p in self._nav_queue[start:]]
                self.map_view.set_route(pts)
        else:
            self.status_pose.setText('机器人: 定位不可用')

    # ================================================== 话题数据处理
    def _on_map_gated(self, msg):
        """地图渲染门控：未确认连接工控机时不画车上的地图。"""
        if not self._data_allowed():
            return
        self.map_view.update_map(msg)

    def _on_scan(self, msg):
        if not self._data_allowed():
            return
        # 优先用雷达帧的 TF（含安装偏转），避免"左右反/前后反"。
        pose = self.bridge.scan_frame_transform(msg.header.frame_id)
        if pose is None:
            pose = getattr(self, '_robot_pose', None)
        points = self.bridge.scan_to_points(msg, pose)
        self.map_view.set_scan_points(points)
        self.sensor_panel.update_scan(msg)

    def _on_pose_signal(self, x, y, yaw):
        """AMCL/里程计推送的位姿：立即刷新机器人图标，不再依赖 TF 轮询。"""
        self._robot_pose = (x, y, yaw)
        if self._data_allowed():
            self.map_view.set_robot_pose((x, y, yaw))
        self.sensor_panel.update_pose((x, y, yaw))
        self.status_pose.setText(
            f'机器人: ({x:.2f}, {y:.2f})  {math.degrees(yaw):.0f}°')

    def _on_sonar(self, state_map):
        for topic, state in state_map.items():
            self.sensor_panel.update_sonar(topic, state)

    def _on_camera_cloud(self, state_map):
        for topic, state in state_map.items():
            self.sensor_panel.update_camera(topic, state)

    def _on_imu(self, angular_z, linear_x):
        self.sensor_panel.update_imu(angular_z, linear_x)

    # ================================================== 传感器面板弹出/停靠
    def _toggle_sensor_popout(self, popout):
        """把传感器面板在「右侧标签页」与「独立窗口」之间切换。

        正确顺序：先构造新窗口并 addWidget → 再 removeTab → show + 强制重绘。
        """
        if popout:
            if self.sensor_win is not None:
                self.sensor_win.raise_()
                self.sensor_win.activateWindow()
                return
            self.sensor_scroll.takeWidget()
            self.sensor_win = SensorWindow(self.sensor_panel,
                                            self._dock_sensor_panel)
            idx = self.right_tabs.indexOf(self.sensor_scroll)
            if idx >= 0:
                self.right_tabs.removeTab(idx)
            self.sensor_win.resize(430, 880)
            self.sensor_win.show()
            self.sensor_win.raise_()
            self.sensor_win.activateWindow()
            self.sensor_panel.updateGeometry()
            self.sensor_panel.show()
            for child in self.sensor_panel.findChildren(QWidget):
                child.update()
            self.sensor_panel.update()
            self.sensor_panel.repaint()
            self.sensor_panel.set_floating(True)
            self.log('传感器面板已弹出为独立窗口（可同时观察地图）')
        else:
            self._dock_sensor_panel()

    def _dock_sensor_panel(self):
        """把面板从独立窗口交还到右侧标签页。"""
        if self.sensor_win is not None:
            self.sensor_scroll.setWidget(self.sensor_panel)
            self.right_tabs.addTab(self.sensor_scroll, '传感器')
            self.right_tabs.setCurrentWidget(self.sensor_scroll)
            self.sensor_panel.updateGeometry()
            self.sensor_panel.show()
            for child in self.sensor_panel.findChildren(QWidget):
                child.update()
            self.sensor_panel.update()
            self.sensor_panel.repaint()
            self.sensor_win.deleteLater()
            self.sensor_win = None
        else:
            self.right_tabs.setCurrentWidget(self.sensor_scroll)
        self.sensor_panel.set_floating(False)

    def _reveal_sensor_panel(self):
        """机器人启动后把传感器面板带到前台，省去手动切标签页。"""
        if self.sensor_win is not None:
            self.sensor_win.show()
            self.sensor_win.raise_()
            self.sensor_win.activateWindow()
        else:
            self.right_tabs.setCurrentWidget(self.sensor_scroll)

    def _on_nav_decision(self, text):
        self.sensor_panel.set_nav_decision(text)
        self.sensor_panel.log_decision('导航', text)
        if text and '空闲' not in text:
            print(f'[导航决策] {text}', flush=True)

    def _on_avoid_decision(self, text):
        self.sensor_panel.set_avoid_decision(text)
        self.sensor_panel.log_decision('避障', text)
        if text and '空闲' not in text:
            print(f'[避障决策] {text}', flush=True)

    def _on_path(self, msg):
        if not self._data_allowed():
            return
        # 记录收到时间：/plan 只在导航进行中由规划器持续发布。
        # 3 秒没新路径就会被 _refresh_pose 当作残影清掉（见该函数）。
        self._last_path_rx = time.monotonic()
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.map_view.set_path(pts)

    def _on_nav_state(self, text):
        self.status_nav.setText(f'导航: {text}')
        self.log(f'[导航] {text}')
        self.navigating = ('导航中' in text or '目标已发送' in text)
        self._refresh_queue_ui()

    def _on_mouse_moved(self, x, y):
        self.status_mouse.setText(f'地图: ({x:.2f}, {y:.2f})')

    def _on_pick_rejected(self):
        """地图范围外的点选被拒绝（设位姿/打点都不允许落在图外）。"""
        self.log('⚠ 该点在地图范围外，无法设定。请在地图内部点击'
                 '（可先用滚轮缩放找到地图再点）')

    def _on_pick_blocked(self):
        """点选落在障碍/未知区域被拒绝（目标物理不可达）。"""
        self.log('⚠ 该点是障碍或未知区域，机器人走不到，未加入队列。'
                 '请在白色（空闲）区域打点')

    # ================================================== 机器人启停
    def _toggle_robot(self, checked):
        """「启动机器人 / 停止机器人」：SSH 在工控机上启停整套系统。

        本机一个进程都不起 —— 起两套会打架（双 EKF/双 SLAM 互发 TF）。
        SSH 脚本耗时数秒，全部走后台线程，界面保持响应。
        """
        if not self.remote:
            self.btn_sim.setChecked(False)
            return
        if checked:
            self.st_sim.setText('连接中…')
            # 「启动机器人」只起机器人本体，不进建图/导航 —— 那是用户
            # 随后点对应按钮才起的。
            self._pending_remote_mode = 'idle'

            def probe():
                # 只发一条 SSH：既探连通又查在跑什么（见 RemoteHost.status）。
                st = self.remote.status()
                if st is None:
                    return (False, 'SSH 连不上 %s' % self.remote.target)
                if st in ('slam', 'nav'):
                    return (True, 'adopt:' + st)
                if st == 'idle':
                    return (True, 'adopt:idle')
                return self.remote.start(mode='idle')

            self._remote_task(probe, self._on_remote_started)
        else:
            self._stop_all_modes()

    # ================================================== 模式切换
    def _switch_mode(self, mode, force_restart=False):
        if self.current_mode == mode:
            # 再点一次 = 退出该模式
            self._stop_all_modes()
            self._update_mode_ui()
            return
        if not self.remote:
            self._update_mode_ui()
            return
        map_file = ''
        if mode == 'navigation':
            map_file = self._remote_map_path()
            if not map_file:
                QMessageBox.warning(
                    self, '提示',
                    '工控机上没有可用地图。\n'
                    '请先建图并保存（「建图」→「另存地图」），再切到导航。')
                self._update_mode_ui()
                return
        self._pending_remote_mode = mode
        self.log(f'正在连接工控机（{self.remote.target}）…')
        self.st_sim.setText('连接中…')

        def on_probe(running):
            # running: 'slam' / 'nav' / None —— 车上现在实际跑的模式
            # ⚠️ force_restart（切换地图）时绝不能"接管"：车上导航栈还挂着
            # 旧地图，GUI 却显示新图 —— 车按旧图匹配、人按新图打点，
            # 两套坐标系对不上，激光贴图整体错乱（2026-09-28 切图"左右反"根因）。
            # 切图必须强制重启导航栈并带上新 map_file。
            if ((not force_restart)
                    and running == ('slam' if mode == 'mapping' else 'nav')):
                # 接管：车上已经在跑目标模式（可能是终端手动起的），
                # 不重启 —— 重启会丢掉已建但未保存的图。
                self._adopt_remote_mode(mode)
                return

            def do_restart():
                self.remote.stop()
                return self.remote.start(
                    mode='slam' if mode == 'mapping' else 'nav',
                    map_file=map_file)
            self._remote_task(do_restart, self._on_remote_started)

        self._remote_task(self.remote.running_mode, on_probe)

    def _stop_all_modes(self, ask_save=True):
        """停止当前模式：后台停工控机系统 + 复位界面状态。

        ask_save：退出建图模式前是否提示保存。关窗路径传 False
        （closeEvent 已先同步停车，再问保存没有意义）。
        """
        if ask_save and self.current_mode == 'mapping':
            ret = QMessageBox.question(
                self, '保存地图',
                '退出建图模式会停止工控机上的 SLAM。\n'
                '是否先保存当前建图结果？\n'
                '（选「保存」后请在保存完成后再点一次退出）',
                QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel,
                QMessageBox.Yes)
            if ret == QMessageBox.Cancel:
                # 取消退出：保持按钮选中状态不变
                self._update_mode_ui()
                return
            if ret == QMessageBox.Yes:
                self._save_map_dialog()
                self._update_mode_ui()
                return
        # 真机：SLAM/Nav2 和驱动是同一个 launch 拉起来的，
        # 停掉它就等于退出了当前模式（顺便清空被污染的代价地图）。
        self._async_stop_remote()
        self._real_connected = False
        self.current_mode = None
        self.btn_sim.setChecked(False)
        self.btn_sim.setText('启动机器人')
        self.st_sim.setText('未运行')
        # 退出导航时清空任务队列并复位点选模式
        if self._route_active:
            self.route_hb_timer.stop()
            self.bridge.send_route_ctrl('stop')   # 通知巡线节点停车
            self._route_active = False
        self._queue_active = False
        self.navigating = False
        self._nav_queue = []
        self._init_pose_set = False
        self._auto_pose_attempts = 0
        self._queue_idx = 0
        self._refresh_queue_ui()
        self.map_view.set_mode(MapView.MODE_PAN)
        self.map_view.set_goal_arrow(None)
        self.map_view.set_path([])            # 退出模式时清路径残影
        self._last_path_rx = 0.0
        self.map_view.set_route([])           # 清除巡线打点连线
        self.map_view.update_queue([], -1)
        self.btn_add_goal.setChecked(False)
        self.btn_set_pose.setChecked(False)
        self._update_mode_ui()

    def _update_mode_ui(self):
        self.btn_mapping.setChecked(self.current_mode == 'mapping')
        self.btn_navigation.setChecked(self.current_mode == 'navigation')
        mode_text = {'mapping': '建图模式', 'navigation': '导航模式'}.get(
            self.current_mode, '未启动')
        self.st_mode.setText(mode_text)

    # ================================================== 全局急停（P1）
    def _emergency_stop(self):
        """全局急停：取消导航 + 清空队列 + 发零速 + 停遥控。

        与「停止导航」的区别：急停在任何状态下都有效（包括手动遥控中），
        且同时清掉键盘按住状态 —— 一个按钮覆盖所有停车场景。
        """
        self._stop_navigation()
        self.keys_down.clear()
        self.log('⛔ 急停：已取消导航、清空队列并发送零速')

    # ================================================== 初始位姿自动定位
    def _schedule_auto_init_pose(self):
        """进入导航模式后，若用户未手动设定位姿，则自动设定初始位姿。

        AMCL 收到初始位姿后才会发布 map->odom，否则 map 坐标系不存在，
        全局代价地图无法转换 base_link->map，所有导航目标都会被拒绝。
        """
        self._init_pose_set = False
        self._auto_pose_attempts = 0
        # 延迟 12s：覆盖车上节点就绪 + AMCL configure/activate 的时间
        QTimer.singleShot(12000, self._auto_init_pose_once)

    def _arm_pose_check(self, x, y, yaw):
        """记录刚设定的初始位姿，4 秒后核对 AMCL 是否"翻面/跳走"。"""
        self._initpose_ref = (x, y, yaw)
        QTimer.singleShot(4000, self._verify_initial_pose)

    def _verify_initial_pose(self):
        """对称环境（长走廊/成排工位）AMCL 可能收敛到旋转 180° 的镜像解：
        激光贴图左右反、巡线位姿全错。以前"退出 GUI 重进就好"的真相是
        重进后又发了一次初始位姿把粒子拉回来了 —— 这里把该检查做成主动告警：
        设定位姿 4 秒后，AMCL 位姿若与设定值差得太远（跳走 >0.6m 或
        翻转 >60°），立即提示重新设定，不必等巡线跑错才发现。
        """
        if self.current_mode != 'navigation':
            return
        ref = getattr(self, '_initpose_ref', None)
        if ref is None:
            return
        pose = self.bridge.get_robot_pose()
        if pose is None:
            return
        dx = pose[0] - ref[0]
        dy = pose[1] - ref[1]
        dyaw = abs((pose[2] - ref[2] + math.pi) % (2 * math.pi) - math.pi)
        if math.hypot(dx, dy) > 0.6 or dyaw > math.radians(60):
            self.log('⚠ 定位可能"翻面"了：AMCL 收敛到了镜像位置'
                     '（对称环境的典型退化，激光贴图会左右反）。')
            self.log('  处理：重新点「设定初始位姿」（位置+车头朝向都要指对），'
                     '或点「全局定位」让 AMCL 自动重搜。'
                     '设完后保持车辆静止等待收敛。')

    def _auto_init_pose_once(self):
        if self.current_mode != 'navigation':
            return
        if self._init_pose_set:
            return
        # 已有 map->base 变换，说明定位已生效（可能用户已手动设定）
        if self.bridge.get_robot_pose() is not None:
            self._init_pose_set = True
            self.log('初始位姿已生效，机器人已定位')
            return
        # 优先用该地图上次保存的初始位姿：机器人每次出生点固定，地图↔世界
        # 变换固定，保存一次即可长期精确复用，不用每次手动点选。
        saved = self._load_initial_pose()
        if saved is not None:
            x, y, yaw = saved
            self.bridge.publish_initial_pose(x, y, yaw)
            self.map_view.set_init_arrow((x, y, yaw))
            self._init_pose_set = True
            self._arm_pose_check(x, y, yaw)
            self.log(f'已用保存的初始位姿定位 ({x:.2f}, {y:.2f})'
                     f'（如需重新定位请点"设定初始位姿"或"全局定位"）')
            self._auto_refine_pose()
            return
        odom = self.bridge.get_robot_odom_pose()
        if odom is not None:
            x, y, yaw = odom
            self.bridge.publish_initial_pose(x, y, yaw)
            self.map_view.set_init_arrow((x, y, yaw))
            self._init_pose_set = True
            self._arm_pose_check(x, y, yaw)
            self.log(f'已自动设定初始位姿为机器人当前位置 ({x:.2f}, {y:.2f})，'
                     f'请核对地图上的机器人图标，若位置不对请手动"设定初始位姿"')
        else:
            self._auto_pose_attempts += 1
            if self._auto_pose_attempts < 60:
                self.log(f'等待里程计 TF 可用 ({self._auto_pose_attempts}/60)...')
                QTimer.singleShot(1000, self._auto_init_pose_once)
            else:
                self.log('警告：无法获取里程计位姿，请点击"设定初始位姿"并在地图上标出机器人位置')

    def _load_initial_pose(self):
        """读取当前地图已保存的初始位姿，返回 (x, y, yaw) 或 None。"""
        m = next((x for x in self.maps if x['name'] == self.current_map), None)
        if m is None:
            return None
        path = os.path.join(m['dir'], 'initial_pose.json')
        if not os.path.exists(path):
            return None
        try:
            with open(path, encoding='utf-8') as f:
                d = json.load(f)
            return (float(d['x']), float(d['y']), float(d.get('yaw', 0.0)))
        except Exception:
            return None

    def _save_initial_pose(self, pose):
        """把初始位姿保存到当前地图目录，下次导航自动精确复用。"""
        m = next((x for x in self.maps if x['name'] == self.current_map), None)
        if m is None:
            return
        path = os.path.join(m['dir'], 'initial_pose.json')
        try:
            with open(path, 'w', encoding='utf-8') as f:
                json.dump({'x': pose[0], 'y': pose[1], 'yaw': pose[2]}, f)
            self.log(f'已保存初始位姿 → {os.path.relpath(path, BASE_DIR)}')
        except Exception as e:
            self.log(f'保存初始位姿失败: {e}')

    # ================================================== 初始位姿自动精修
    def _auto_refine_pose(self):
        """设定初始位姿后自动做一次「激光-地图」匹配精修。

        手动点选再准也有 ±0.3m/±15° 的误差，对称环境 AMCL 还可能收敛到
        镜像解。这里发完初始位姿等 1.5s（AMCL 发布 map->odom 后），取一帧
        激光按当前位姿转成世界点，在 ±1.2m/±50° 范围内搜索最优刚体修正
        （pose_refiner.refine_pose，后台线程），命中后把修正位姿重发
        /initialpose —— 用户只需指个大概，剩下的交给匹配。
        """
        self._refine_seq = getattr(self, '_refine_seq', 0) + 1
        seq = self._refine_seq
        QTimer.singleShot(1500, lambda: self._start_pose_refine(seq))

    def _start_pose_refine(self, seq, _tf_attempt=0):
        """采集匹配所需数据（主线程），把重计算丢进后台线程。"""
        if seq != self._refine_seq or self.current_mode != 'navigation':
            return          # 用户又重新设定了位姿 / 已退出导航模式：作废
        scan = self.bridge._latest_scan
        if scan is None:
            self.log('自动精修：暂无雷达数据，跳过')
            return
        lin, ang = self.bridge._odom_vel
        if abs(lin) > 0.05 or abs(ang) > 0.05:
            self.log('自动精修：机器人在运动，跳过（静止后重新设定一次即可）')
            return
        # 雷达帧位姿（TF map->laser，含安装偏转）：激光点就是按它转成世界系的。
        # 刚切地图/启动后 TF 缓存未填充 → 查不到是暂态，重试 3 次（2s 间隔）
        # 而不是直接放弃（09-30 实测：每次切图后第一次设位姿必报 TF 不可用）。
        laser = self.bridge.scan_frame_transform(scan.header.frame_id)
        if laser is None:
            if _tf_attempt < 3:
                QTimer.singleShot(
                    2000, lambda: self._start_pose_refine(seq, _tf_attempt + 1))
            else:
                self.log('自动精修：雷达 TF 持续不可用，跳过')
            return
        pts = self.bridge.scan_to_points(scan, laser)
        if len(pts) < 100:
            self.log(f'自动精修：有效激光点太少（{len(pts)}），跳过')
            return
        snap = self.map_view.occ_grid_snapshot()
        if snap is None:
            self.log('自动精修：地图占据栅格未就绪，跳过')
            return
        grid, ox, oy, res = snap
        self.log(f'自动精修：{len(pts)} 个激光点正在匹配地图…')
        self._remote_task(lambda: refine_pose(pts, grid, ox, oy, res),
                          lambda r: self._on_pose_refined(seq, r))

    def _on_pose_refined(self, seq, result):
        """精修完成（主线程）：把修正量作用到机器人位姿并重发 /initialpose。"""
        if seq != self._refine_seq:
            return
        if not isinstance(result, dict):
            self.log(f'自动精修异常: {result}')
            return
        if not result.get('ok'):
            if result.get('reason'):
                self.log('自动精修：' + result['reason'])
            return
        dx, dy, dyaw = result['correction']
        if (abs(dx) < 0.01 and abs(dy) < 0.01
                and abs(dyaw) < math.radians(0.5)):
            self.log(f'自动精修：激光已与地图贴合'
                     f'（残差 {result["score_before"] * 100:.0f}cm），无需修正')
            return
        # 修正量是 map 系刚体变换：对当前位姿整体作用后重发
        base = self.bridge.get_robot_pose()
        if base is None:
            self.log('自动精修完成，但定位不可用，未重发')
            return
        bx, by, byaw = base
        c, s = math.cos(dyaw), math.sin(dyaw)
        nx = c * bx - s * by + dx
        ny = s * bx + c * by + dy
        nyaw = math.atan2(math.sin(byaw + dyaw), math.cos(byaw + dyaw))
        self.bridge.publish_initial_pose(nx, ny, nyaw)
        self.map_view.set_init_arrow((nx, ny, nyaw))
        # 精修位姿作为新的基准：保存复用 + 以它为准核对 AMCL
        self._save_initial_pose((nx, ny, nyaw))
        self._arm_pose_check(nx, ny, nyaw)
        self.log(f'自动精修完成：修正 ({dx * 100:+.0f}, {dy * 100:+.0f}) cm / '
                 f'{math.degrees(dyaw):+.1f}°，'
                 f'残差 {result["score_before"] * 100:.0f} → '
                 f'{result["score_after"] * 100:.0f} cm')

    # ================================================== 多地图管理
    def _migrate_legacy_maps(self):
        """把旧结构 maps/<名字>/ 自动迁移到 maps/saved/<名字>/，统一归档。"""
        if not os.path.isdir(MAPS_DIR):
            return
        try:
            os.makedirs(SAVED_MAPS_DIR, exist_ok=True)
        except Exception as e:
            self.log(f'创建地图归档目录失败: {e}')
            return
        for entry in os.listdir(MAPS_DIR):
            if entry == 'saved':
                continue
            sub = os.path.join(MAPS_DIR, entry)
            if not os.path.isdir(sub):
                continue
            yaml_path = os.path.join(sub, 'map.yaml')
            if not os.path.exists(yaml_path):
                continue
            target = os.path.join(SAVED_MAPS_DIR, entry)
            if os.path.exists(target):
                continue  # saved 下已有同名，保留旧目录不动
            try:
                shutil.move(sub, target)
                self.log(f'已迁移旧地图 {entry} → saved/{entry}')
            except Exception as e:
                self.log(f'迁移地图 {entry} 失败: {e}')

    def _scan_maps(self, select=None):
        """扫描本地地图目录，收集所有 map.yaml 并刷新下拉框。"""
        self._migrate_legacy_maps()
        self.maps = []

        def _add(name, yaml_path, dir_path):
            self.maps.append({'name': name, 'yaml': yaml_path, 'dir': dir_path})

        top_yaml = os.path.join(MAPS_DIR, 'map.yaml')
        if os.path.exists(top_yaml):
            _add('默认地图', top_yaml, MAPS_DIR)

        if os.path.isdir(SAVED_MAPS_DIR):
            for entry in sorted(os.listdir(SAVED_MAPS_DIR)):
                sub = os.path.join(SAVED_MAPS_DIR, entry)
                yaml_path = os.path.join(sub, 'map.yaml')
                if os.path.isdir(sub) and os.path.exists(yaml_path):
                    _add(entry, yaml_path, sub)

        if os.path.isdir(MAPS_DIR):
            for entry in sorted(os.listdir(MAPS_DIR)):
                if entry == 'saved':
                    continue
                sub = os.path.join(MAPS_DIR, entry)
                yaml_path = os.path.join(sub, 'map.yaml')
                if os.path.isdir(sub) and os.path.exists(yaml_path):
                    _add(entry, yaml_path, sub)

        self.map_combo.blockSignals(True)
        self.map_combo.clear()
        for m in self.maps:
            self.map_combo.addItem(m['name'])
        target = select or self.current_map
        idx = next((i for i, m in enumerate(self.maps)
                    if m['name'] == target), 0)
        if self.maps:
            self.map_combo.setCurrentIndex(idx)
        self.map_combo.blockSignals(False)

        if not self.maps:
            self.current_map = None
            self.log('本地暂无地图。连接工控机后会自动同步车上的地图，'
                     '或点「刷新」手动拉取。')
            return
        self.log(f'已扫描到 {len(self.maps)} 张本地地图: ' +
                 ', '.join(m['name'] for m in self.maps))
        # 只有当调用方明确要求，或已处于建图/导航模式时才真正加载地图到视图
        # —— 启动时的自动扫描只填下拉框，否则 GUI 一打开就显示地图。
        if select or self.current_mode in ('mapping', 'navigation'):
            self._switch_map(self.maps[idx]['name'])
        else:
            self.current_map = self.maps[idx]['name']

    def _on_map_combo(self, idx):
        if idx < 0 or idx >= len(self.maps):
            return
        name = self.maps[idx]['name']
        if name == self.current_map:
            return
        if self.current_mode == 'navigation':
            ret = QMessageBox.question(
                self, '切换地图', f'切换地图[{name}]将重启导航栈（重新定位），是否继续？',
                QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
            if ret != QMessageBox.Yes:
                # 回退下拉框
                self.map_combo.blockSignals(True)
                self.map_combo.setCurrentIndex(
                    next((i for i, m in enumerate(self.maps)
                          if m['name'] == self.current_map), 0))
                self.map_combo.blockSignals(False)
                return
            self._switch_map(name)
            self._restart_navigation()
        else:
            self._switch_map(name)
        self.log(f'当前地图: {name}')

    def _switch_map(self, name):
        """切换当前地图：磁盘加载显示 + 加载该地图的点位。"""
        self.current_map = name
        m = next((x for x in self.maps if x['name'] == name), None)
        if m is None:
            return
        ret = self.map_view.load_map_file(m['yaml'])
        if ret is not None:
            w, h, res = ret
            self.log(f'已加载地图[{name}] {w}x{h} @ {res} m/px')
        else:
            self.log(f'地图[{name}] 加载失败：{m["yaml"]}（pgm/yaml 损坏或格式不支持）')
        self._load_points()
        self._load_keepout()

    def _current_map_yaml(self):
        """当前地图的 yaml 路径（无地图返回 None）。"""
        m = next((x for x in self.maps if x['name'] == self.current_map), None)
        return m['yaml'] if m else None

    def _remote_map_path(self):
        """当前地图在工控机上的 yaml 路径。

        导航 launch 是跑在工控机上的，给它传本机路径会找不到文件。
        """
        m = next((x for x in self.maps if x['name'] == self.current_map), None)
        if m is None:
            return ''
        return m.get('remote_yaml') or ''

    def _sync_remote_maps(self):
        """把工控机上的地图列表同步到本端（缺的自动 scp 回来）。

        ⚠️ 必须后台线程跑（_remote_task），绝不能在主线程同步执行：
        首次连通时这里要串行跑十几条 ssh/scp，十秒上下，同步执行
        界面会卡死，还会与用户此刻点「启动机器人」的 probe 线程竞争。
        """
        if not self.remote:
            self._scan_maps()
            return
        self._remote_task(self._sync_remote_maps_impl,
                          self._apply_remote_maps)

    def _sync_remote_maps_impl(self):
        """后台线程：只做 SSH / 文件传输，返回地图元数据列表，不碰任何 UI。"""
        if not self.remote.is_reachable():
            return []
        names = self.remote.list_maps()
        if not names:
            return []
        maps = []
        for name in names:
            local_dir = os.path.join(SAVED_MAPS_DIR, name)
            local_yaml = os.path.join(local_dir, 'map.yaml')
            if not (os.path.exists(local_yaml)
                    and os.path.exists(os.path.join(local_dir, 'map.pgm'))):
                local_yaml = self.remote.fetch_map(name, local_dir) or ''
            if local_yaml:
                maps.append({
                    'name': name,
                    'yaml': local_yaml,
                    'dir': local_dir,
                    'remote_yaml': self.remote.remote_path(name) + '.yaml',
                })
        maps.sort(key=lambda x: x['name'])
        return maps

    def _apply_remote_maps(self, maps):
        """主线程：用后台线程拉回的地图列表刷新下拉框与显示。"""
        if not maps:
            self._scan_maps()
            self.log('工控机上还没有地图，请先建图')
            return
        self.maps = maps

        self.map_combo.blockSignals(True)
        self.map_combo.clear()
        for m in self.maps:
            self.map_combo.addItem(m['name'])
        idx = next((i for i, m in enumerate(self.maps)
                    if m['name'] == self.current_map), 0)
        if self.maps:
            self.map_combo.setCurrentIndex(idx)
        self.map_combo.blockSignals(False)
        if self.maps:
            self._switch_map(self.maps[idx]['name'])
        self.log('工控机地图：' + ', '.join(m['name'] for m in self.maps))

    def _restart_navigation(self):
        """导航模式运行中切换地图：强制重启车上导航栈并加载新地图。"""
        # 先清空模式再进入（避免 _switch_mode 的"再点一次=退出"逻辑）
        self.current_mode = None
        self._update_mode_ui()
        # force_restart=True：车上还挂着旧地图，必须重启换图，
        # 否则"车按旧图匹配、GUI 显示新图"——切图后位姿全错（左右反）。
        self._switch_mode('navigation', force_restart=True)

    # ================================================== 真机连接
    def _check_real_link(self, _retry=0):
        """检查工控机数据是否真的到达本机。

        只看 ros2 topic list 是不够的 —— DDS 发现通了但数据不通时，
        话题名照样列得出来。这里按"最近 3 秒是否收到过数据"判断。

        _retry：启动预热期的自动重查次数（EKF 要等 imu_preprocess 和
        dt_ros2 就绪，5~15s），最多 4 次覆盖预热窗口，全部失败才亮红。
        """
        st = self.bridge.get_link_status()
        was_connected = self._real_connected
        self._real_connected = st['connected']
        if st['connected']:
            self.st_network.setText('已连接工控机')
            self.st_sim.setText('工控机在线')
            # P0 修复：连上后统一显示「停止机器人」。
            # 原来显示「断开」会与旁边的红色「断开连接」按钮混淆 ——
            # 两者行为不同（后者才做孤儿清理+复查）。
            self.btn_sim.setText('停止机器人')
            self.btn_sim.setChecked(True)
            self.log(f'✓ 已连上工控机（/odom、/scan 有数据）。'
                     f'现在可以进建图/导航模式。')
            # 确认连通后才同步车上地图 —— 避免"还没连接就在跳实机数据"
            if not was_connected and self.remote:
                self._sync_remote_maps()
                # P0 修复：孤儿进程探测（is_running 是一条 SSH）移到后台。
                # 原来在主线程同步调用，连上的一瞬间界面会卡住几秒，
                # 还可能和地图同步线程抢 _ssh_lock。
                self._remote_task(self.remote.is_running,
                                  self._on_orphan_probe)
            self._reveal_sensor_panel()
        elif self.remote is not None and _retry < 4:
            # 预热重查：不刷红、不刷排查清单，避免用户误判为"连不上"
            self.st_network.setText('预热中（等待工控机数据）')
            self.st_sim.setText('预热中…')
            if _retry == 0:
                self.log('车上节点启动预热中（EKF/驱动就绪要 5~15 秒），'
                         '每 3 秒自动重查，无需重复点击…')
            else:
                self.log(f'…还在预热，3 秒后进行第 {_retry + 1} 次重查')
            QTimer.singleShot(3000, lambda: self._check_real_link(_retry + 1))
        else:
            self.st_network.setText(f"未连接（{st['detail']}）")
            self.st_sim.setText('未连接')
            self.btn_sim.setChecked(False)
            self.btn_sim.setText('启动机器人')
            self._real_connected = False
            # 不弹模态框：切档案/轮询时弹窗会打断操作。只在日志里说明。
            self.log(f"✗ 未连上工控机：{st['detail']}")
            if not was_connected:
                self.log('  排查顺序：')
                self.log('   1) 工控机上机器人是否在跑（GUI 点「启动机器人」）')
                self.log('   2) 两边是否都跑过：bash install.sh --dds')
                self.log('   3) 是否同一网段、防火墙是否关闭；免密登录是否配好')
                self.log('   4) 时间是否同步：bash install.sh --ntp')
                self.log('   5) 工控机自检：bash install.sh --check')
        self._update_mode_ui()

    def _on_orphan_probe(self, result):
        """孤儿进程探测结果（后台 SSH 回来后在这里处理，主线程）。"""
        running = result if isinstance(result, bool) else True
        # 数据在灌、但 launch 父进程已经不在 = 上次没杀干净的孤儿节点
        # （典型：雷达驱动）。不清理的话下次启动会两套进程互发 TF、
        # 串口互踩 → 位姿跳变。只在刚连上时提示一次，避免刷屏。
        if not running:
            self.log('⚠ 检测到工控机上有上次残留的节点'
                     '（launch 已退出，但雷达/底盘还在发数据）')
            self.log('   建议点「断开连接」清理后再启动，'
                     '否则会和新起的进程打架（位姿跳变）')

    # ================================================== 遥测显示
    def _on_battery(self, voltage, percentage, power_status):
        if voltage <= 0 and percentage <= 0:
            self.st_battery.setText('数据异常')
            return
        # percentage 归一化到 0~1（BatteryState 规范）；若异常直接截断
        pct = min(max(percentage, 0.0), 1.0) * 100.0
        status_map = {1: '充电中', 2: '放电中', 4: '已充满', 5: '未充电'}
        st = status_map.get(power_status, '')
        self.st_battery.setText(f'{pct:.0f}%  {voltage:.1f}V {st}'.strip())

    @staticmethod
    def _chassis_num(s):
        """从 '300mm/s' 这类值中提取数字部分。"""
        m = re.search(r'-?[\d.]+', s or '')
        return m.group(0) if m else '?'

    def _on_chassis(self, text):
        # 底盘心跳：收到即视为在线；显示关键字段（避免刷屏）
        try:
            parts = {k: v for k, v in
                     (pair.split('=', 1) for pair in text.split() if '=' in pair)}
            vx = self._chassis_num(parts.get('vx'))
            wz = self._chassis_num(parts.get('wz'))
            brief = (f"mode={parts.get('mode', '?')} "
                     f"vx={vx}mm/s wz={wz}rad/s")
            if parts.get('estop', '0') == '1':
                brief += ' ⚠急停'
            if parts.get('error', '0') != '0':
                brief += f" 错误码{parts.get('error')}"
        except Exception:
            brief = text
        self.st_chassis.setText(brief)
        self.st_network.setText('在线（底盘心跳）')

    def _on_network(self, text):
        self.st_network.setText(f'网络: {text}')

    # ================================================== 保存地图
    def _save_map_dialog(self):
        """保存地图（全部后台化，P0 修复）。

        原版有两条 SSH（running_mode / ensure_maps_dir）在主线程同步执行，
        点「另存地图」界面会卡住好几秒，还会与其它后台 SSH 抢锁。
        现在流程：先取名 → 后台准备（连通性 + 建目录 + 确认 SLAM 在跑）
        → 回调里调用保存服务（本地 DDS，不涉 SSH）→ 后台轮询文件出现。
        """
        if self._saving_map_name:
            QMessageBox.information(
                self, '提示', f'正在保存地图「{self._saving_map_name}」，请稍候…')
            return
        name, ok = QInputDialog.getText(self, '保存地图', '地图名称（如：office58）：')
        if not ok or not name.strip():
            return
        name = name.strip()
        if any(m['name'] == name for m in self.maps):
            ret = QMessageBox.question(
                self, '覆盖确认', f'地图"{name}"已存在，是否覆盖？',
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if ret != QMessageBox.Yes:
                return
        self._saving_map_name = name
        self.btn_save_map.setEnabled(False)
        self.log(f'正在准备保存地图[{name}]（检查工控机状态）...')

        def prepare():
            # 后台线程：全部 SSH 操作集中在这里，不碰任何 UI。
            if not self.remote.is_reachable():
                return (False, '连不上工控机 %s' % self.remote.target)
            self.remote.ensure_maps_dir()
            # GUI 没进"建图模式"也允许保存（用户可能在终端手动建图），
            # 但必须确认车上真的有 SLAM 在跑，否则保存必然失败。
            if self.current_mode != 'mapping':
                if self.remote.running_mode() != 'slam':
                    return (False, '没检测到正在进行的建图（工控机上没有 SLAM 进程）')
            return (True, name)

        self._remote_task(prepare, self._on_save_map_prepared)

    def _on_save_map_prepared(self, result):
        """保存准备完成（后台回调，主线程）：真正发起保存。"""
        ok, msg = (result if isinstance(result, tuple) else (False, str(result)))
        name = self._saving_map_name
        if not ok:
            self._saving_map_name = None
            self.btn_save_map.setEnabled(True)
            QMessageBox.warning(self, '无法保存', str(msg))
            return
        # 真机：slam_toolbox 跑在工控机上，必须传**工控机**的路径，
        # 传本机路径会静默失败（文件写在不存在的地方，地图丢失）。
        prefix = self.remote.remote_path(name)
        self.log(f'正在保存到工控机：{prefix}.pgm / .yaml ...')
        if not self.bridge.save_map(prefix):
            self.log('[警告] 保存服务调用失败，请确认工控机上 SLAM 正在运行')
            self._saving_map_name = None
            self.btn_save_map.setEnabled(True)
            return
        # 轮询文件出现（也全部走后台，见 _poll_remote_map_saved）
        self._poll_remote_map_saved(name, attempts=6)

    def _poll_remote_map_saved(self, name, attempts):
        """轮询工控机上的地图文件是否出现（P0 修复：后台化）。

        原版每 1.2s 在**主线程**同步发一条 SSH（has_map），保存期间
        界面反复卡顿，且会与其它后台 SSH 抢锁排队。
        现在每次轮询发起一个后台任务，回调里决定拉取或排下一次 ——
        回调在主线程，UI 更新安全；SSH 全在后台。
        """
        if self._saving_map_name != name:
            return

        self._remote_task(lambda: self.remote.has_map(name),
                          lambda has: self._on_map_file_polled(name, attempts, has))

    def _on_map_file_polled(self, name, attempts, has):
        """轮询结果处理（主线程）。"""
        if self._saving_map_name != name:
            return
        if has:
            # 文件已出现：scp 拉回本地（后台）再刷新列表
            def pull():
                local_dir = os.path.join(SAVED_MAPS_DIR, name)
                return self.remote.fetch_map(name, local_dir)

            self._remote_task(pull, lambda r: self._on_map_fetched(name, r))
            return
        if attempts > 0:
            QTimer.singleShot(1200, lambda: self._poll_remote_map_saved(
                name, attempts - 1))
        else:
            self._saving_map_name = None
            self.btn_save_map.setEnabled(True)
            self.log('工控机上仍未出现地图文件，请确认 SLAM 在运行、'
                     '且地图目录可写（~/dt01_maps）')

    def _on_map_fetched(self, name, result):
        """地图拉回本地后的收尾（主线程）。"""
        self._saving_map_name = None
        self.btn_save_map.setEnabled(True)
        if result:
            self.log(f'地图已保存并同步到本端：'
                     f'{os.path.join(SAVED_MAPS_DIR, name, "map.yaml")}')
            self._sync_remote_maps()
        else:
            self.log(f'地图已保存在工控机，但同步到本端失败（可稍后点「刷新」重试）')

    # ================================================== 地图点选
    def _toggle_set_pose(self, checked):
        if checked:
            self.btn_add_goal.setChecked(False)
            self.btn_draw_keepout.setChecked(False)
            self.map_view.set_mode(MapView.MODE_SET_POSE)
            self.log('请在地图上点击并拖动以设定初始位姿')
        else:
            self.map_view.set_mode(MapView.MODE_PAN)

    def _do_global_localization(self):
        """一键全局定位：让 AMCL 自动搜索机器人位姿，无需手动点选。"""
        if self.current_mode != 'navigation':
            QMessageBox.warning(self, '提示', '请先切换到导航模式再全局定位')
            return
        self.bridge.publish_global_localization()
        self._init_pose_set = False
        self.log('已请求全局定位：AMCL 正在自动搜索位姿，请等待收敛（勿移动机器人）')

    def _toggle_add_goal(self, checked):
        if checked:
            self.btn_set_pose.setChecked(False)
            self.btn_draw_keepout.setChecked(False)
            self.map_view.set_mode(MapView.MODE_SET_GOAL)
            self.log('打点模式：在地图上点击（可拖动设朝向），松开即加入任务队列')
        else:
            self.map_view.set_mode(MapView.MODE_PAN)

    def _on_pose_picked(self, x, y, yaw):
        if self.btn_set_pose.isChecked():
            self.bridge.publish_initial_pose(x, y, yaw)
            self.map_view.set_init_arrow((x, y, yaw))
            self._init_pose_set = True
            # 保存到地图目录：下次进导航自动精确复用，不用再手动点选
            self._save_initial_pose((x, y, yaw))
            # 自动「激光贴图」精修：用户指个大概，1.5s 后用激光匹配修正
            self._auto_refine_pose()
            # 4s 后核对 AMCL 是否翻面/跳走（对称环境镜像解主动告警）
            self._arm_pose_check(x, y, yaw)
            self.btn_set_pose.setChecked(False)
            self.map_view.set_mode(MapView.MODE_PAN)
        elif self.btn_add_goal.isChecked():
            self._add_to_queue(f'打点{len(self._nav_queue) + 1}', x, y, yaw)
            # 保持打点模式：可连续在地图上打多个点，再点按钮退出
            return
        self.map_view.set_mode(MapView.MODE_PAN)

    # ================================================== 打点任务队列
    def _add_to_queue(self, name, x, y, yaw):
        """把一个目标点加入任务队列（不立即导航）。"""
        if self.current_mode != 'navigation':
            QMessageBox.warning(self, '提示', '请先切换到导航模式再打点')
            return
        # 电子围栏：点在禁区多边形内 → 拒绝打点（从源头拦截危险/不可达目标）
        if self._point_in_keepout(x, y):
            QMessageBox.warning(self, '提示',
                                f'该点 ({x:.2f}, {y:.2f}) 在禁区内，不能作为目标点')
            self.log(f'打点被拒绝：({x:.2f}, {y:.2f}) 落在禁区内')
            return
        self._nav_queue.append({'name': name, 'x': x, 'y': y, 'yaw': yaw})
        self._refresh_queue_ui()
        self.log(f'已加入队列: {name} ({x:.2f}, {y:.2f})  [共 {len(self._nav_queue)} 点]')

    def _refresh_queue_ui(self):
        self.queue_list.clear()
        for i, p in enumerate(self._nav_queue):
            marker = '▶ ' if (self._queue_active and i == self._queue_idx) else f'{i+1}. '
            item = QListWidgetItem(
                f"{marker}{p['name']}   ({p['x']:.2f}, {p['y']:.2f})")
            if self._queue_active and i == self._queue_idx:
                item.setForeground(Qt.cyan)
            self.queue_list.addItem(item)
        # 同步地图可视化：大红旗帜 + 编号 + 点间连线（巡线路线）
        self.map_view.update_queue(self._nav_queue, self._queue_idx)
        pose = getattr(self, '_robot_pose', None)
        if self._nav_queue and pose is not None:
            start = (self._queue_idx
                     if (self._queue_active and self._route_active) else 0)
            pts = [(pose[0], pose[1])]
            pts += [(p['x'], p['y']) for p in self._nav_queue[start:]]
            self.map_view.set_route(pts)
        else:
            self.map_view.set_route([])

    def _clear_queue(self):
        if self._queue_active:
            self._stop_navigation()
        self._nav_queue = []
        self._queue_idx = 0
        self._refresh_queue_ui()
        self.map_view.set_goal_arrow(None)
        self.log('任务队列已清空')

    def _del_queue_item(self):
        row = self.queue_list.currentRow()
        if row < 0 or row >= len(self._nav_queue):
            return
        if self._queue_active and row == self._queue_idx:
            QMessageBox.warning(self, '提示', '该点正在执行中，请先停止导航')
            return
        del self._nav_queue[row]
        if row < self._queue_idx:
            self._queue_idx -= 1
        self._refresh_queue_ui()

    def _undo_last_point(self):
        """撤销最近一次打点：移除任务队列最后一个未执行的点。"""
        if not self._nav_queue:
            QMessageBox.information(self, '提示', '任务队列为空，没有可撤销的打点')
            return
        last = len(self._nav_queue) - 1
        if self._queue_active and last == self._queue_idx:
            QMessageBox.warning(self, '提示', '最后一个打点正在执行中，请先停止导航')
            return
        p = self._nav_queue.pop()
        self._refresh_queue_ui()
        self.log(f'已撤销打点: {p["name"]}  [剩余 {len(self._nav_queue)} 点]')

    def _start_queue(self):
        """开始巡线：机器人按打点连线行驶（先原地对正下一点，再走直线）。"""
        if self.current_mode != 'navigation':
            QMessageBox.warning(self, '提示', '请先切换到导航模式')
            return
        if not self._nav_queue:
            QMessageBox.warning(self, '提示', '任务队列为空，请先在地图上打点')
            return
        if self._queue_active:
            self.log('任务已在执行中')
            return
        # 兜底：若尚未定位（map 坐标系不存在），立即用里程计位姿自动设定初始位姿，
        # 等待 AMCL 发布 map->odom 后再起步巡线。
        if not self._init_pose_set and self.bridge.get_robot_pose() is None:
            odom = self.bridge.get_robot_odom_pose()
            if odom is not None:
                self.bridge.publish_initial_pose(*odom)
                self.map_view.set_init_arrow(odom)
                self._init_pose_set = True
                self.log('已自动设定初始位姿，等待定位生效后开始巡线...')
        self._begin_route()

    # ================================================== 巡线（route_follower 节点跑在工控机上）
    def _begin_route(self, _attempt=0):
        """部署工控机侧巡线节点并下发打点路线。

        协议：goal(路线) + ctrl('start')，冗余发 3 次对抗 DDS 发现延迟；
        节点侧幂等（运行中收到 goal 忽略，重复 start 无副作用）。
        速度取自遥控滑块，随 ctrl 'vmax:x.xx' 一并下发。
        """
        if not self._nav_queue:
            return
        if self.bridge.get_robot_pose() is None:
            if _attempt < 15:
                self.log(f'等待定位可用后开始巡线 ({_attempt + 1}/15)...')
                QTimer.singleShot(1000, lambda: self._begin_route(_attempt + 1))
                return
            self.log('定位一直不可用，巡线任务取消（请先「设定初始位姿」或「全局定位」）')
            self._refresh_queue_ui()
            return
        if not os.path.exists(self._ROUTE_SCRIPT):
            self.log(f'✗ 找不到巡线节点脚本: {self._ROUTE_SCRIPT}')
            return
        if self._route_deploying:
            self.log('巡线节点正在部署中，请稍候…')
            return
        self._route_deploying = True
        self.log('正在准备工控机上的巡线节点（检查/上传/启动）…')
        self._remote_task(
            lambda: self.remote.deploy_route_follower(self._ROUTE_SCRIPT),
            self._on_route_deployed)

    def _on_route_deployed(self, result):
        """部署完成（后台 SSH 回来，主线程）：下发巡线路线。"""
        self._route_deploying = False
        ok, msg = (result if isinstance(result, tuple) else (False, str(result)))
        if not ok:
            self.log(f'✗ 巡线节点部署失败：{msg}')
            return
        self.log(f'✓ {msg}')
        self._queue_active = True
        self._queue_idx = 0
        self._goal_retries = 0
        self._nav_total_count = len(self._nav_queue)
        self._nav_success_count = 0
        self._route_active = True
        self._route_got_status = False
        self.navigating = True          # 阻断手动遥控，避免指令打架
        self._route_pts = [(p['x'], p['y'], p.get('yaw', 0.0))
                           for p in self._nav_queue]
        self.route_hb_timer.start(500)   # 2Hz 心跳：断链保护
        for delay in (0, 800, 1600):
            QTimer.singleShot(delay, self._push_route_once)
        # 4.5s 内节点无任何回报 → 大概率节点没在跑或 DDS 没通
        QTimer.singleShot(4500, self._check_follower_alive)
        self.log(f'巡线开始：已下发 {len(self._route_pts)} 个点，'
                 f'速度 {self.slider_linear.value() / 100:.2f} m/s，'
                 f'机器人将先原地对正下一点方向、再严格沿画出的连线直线行驶')
        self._refresh_queue_ui()

    def _push_route_once(self):
        """下发一次路线包：keepout + vmax + goal + start。重复包节点侧幂等。"""
        if not self._route_active:
            return
        lin = min(self.slider_linear.value() / 100.0, 0.50)
        self.bridge.send_keepout(self._keepout_zones)
        self.bridge.send_route_ctrl(f'vmax:{lin:.2f}')
        self.bridge.send_route(self._route_pts)
        self.bridge.send_route_ctrl('start')

    def _send_route_hb(self):
        """2Hz 心跳：节点超过 hb_timeout（默认 2s）收不到即零速停车。"""
        if self._route_active:
            self.bridge.send_route_heartbeat()

    def _check_follower_alive(self):
        if self._route_active and not self._route_got_status:
            self.log('⚠ 巡线节点 4.5s 内无回报：请检查工控机上节点是否存活'
                     '（日志 /tmp/dt01_route_follower.log），确认后重新「开始导航」')

    def _on_route_status(self, d):
        """route_follower 节点的进度回报：驱动队列 UI / 状态栏 / 收尾。"""
        if not self._route_active:
            return
        phase = d.get('phase')
        if phase == 'done':
            # 巡线全部完成：用节点统计的成败数收尾
            self._nav_success_count = d.get('success', 0)
            self._nav_total_count = d.get('total', len(self._nav_queue))
            self._finish_queue()
            return
        if phase == 'idle':
            # 节点侧安全停车（断链保护/停止指令等），任务终止
            self.log(f'巡线已停止：{d.get("msg", "节点已停车")}')
            self._route_active = False
            self._queue_active = False
            self.navigating = False
            self.route_hb_timer.stop()
            self._goal_start_time = None
            self.status_nav.setText('导航: 巡线已停止')
            self.map_view.set_goal_arrow(None)
            self._refresh_queue_ui()
            return
        # rotate / drive：正常进度回报
        self._route_got_status = True
        idx = d.get('idx')
        if idx is not None and 0 <= idx < len(self._nav_queue):
            self._queue_idx = idx
        msg = d.get('msg') or ''
        if msg:
            self.status_nav.setText(f'巡线: {msg}')
        self._refresh_queue_ui()

    def _finish_queue(self):
        self._queue_active = False
        self._route_active = False
        self.navigating = False
        self.route_hb_timer.stop()
        self.bridge.send_route_ctrl('stop')   # 双保险（节点已完成时为 no-op）
        self._queue_idx = 0
        self._goal_retries = 0
        self._goal_start_time = None
        total = self._nav_total_count
        success = self._nav_success_count
        self._nav_total_count = 0
        self._nav_success_count = 0
        self._nav_queue = []
        self._refresh_queue_ui()
        self.map_view.set_goal_arrow(None)
        self.map_view.set_path([])      # 清掉已结束任务的路径残影
        self._last_path_rx = 0.0
        self.status_nav.setText('导航: 巡线结束')
        # 按成功数如实汇报，避免"全超时"却报"执行完毕"误导用户
        if total > 0 and success == 0:
            self.log(f'任务结束：{total} 个点均未到达（导航失败，机器人未移动）')
        elif success < total:
            self.log(f'任务结束：{success}/{total} 个点到达，其余超时/失败被跳过')
        else:
            self.log('任务队列执行完毕')

    def _on_goal_finished(self, status):
        """上一个目标结束：
        成功 → 执行下一个点；
        取消 → 停止队列；
        失败/拒绝 → 重试当前点，重试耗尽后跳过该点继续执行后续点。"""
        if not self._queue_active or self._route_active:
            return
        if status == 'succeeded':
            elapsed = (time.monotonic() - self._goal_start_time
                       if self._goal_start_time else 0.0)
            self.log(f'✔ 到达点位 {self._queue_idx + 1}/{len(self._nav_queue)}'
                     f'（用时 {elapsed:.1f}s）')
            self._nav_success_count += 1
            self._goal_start_time = None
            self._queue_idx += 1
            self._goal_retries = 0
            self._send_next_goal()
            return
        if status == 'cancelled':
            self.log('导航被取消，任务停止')
            self._finish_queue()
            return
        # failed / rejected：先重试当前点
        if self._goal_retries < self._max_goal_retries:
            self._goal_retries += 1
            self._goal_start_time = None
            self.log(f'✖ 第 {self._queue_idx + 1} 点导航失败（{status}），'
                     f'{self._goal_retries}/{self._max_goal_retries} 次重试中，'
                     f'等待重规划后重试...')
            # 短暂等待，给导航栈恢复/重规划留出时间
            QTimer.singleShot(1500, self._send_next_goal)
            return
        # 重试耗尽：跳过当前点，继续执行后面的点
        self._goal_retries = 0
        self.log(f'✖ 第 {self._queue_idx + 1} 点连续失败（{status}），'
                 f'跳过该点，继续执行后续 {max(len(self._nav_queue) - self._queue_idx - 1, 0)} 点')
        self._queue_idx += 1
        self._send_next_goal()

    def _stop_navigation(self):
        """立即停止：停巡线节点 + 清空队列 + 发零速（行进途中可用）。"""
        self._queue_active = False
        self._route_active = False
        self.navigating = False
        self.route_hb_timer.stop()
        self._nav_queue = []
        self._queue_idx = 0
        self._goal_retries = 0
        self._goal_start_time = None
        self._refresh_queue_ui()
        # 急停链路分两级：① ctrl 'stop' 走 DDS 毫秒级到达，节点立即零速停车；
        # ② SSH 杀工控机上的节点进程只是清理，放后台线程绝不阻塞急停。
        self.bridge.send_route_ctrl('stop')
        threading.Thread(target=self.remote.stop_route_follower,
                         daemon=True).start()
        self.bridge.cancel_goal()
        self.bridge.publish_cmd(0.0, 0.0)   # 保险：直接停车
        self.map_view.set_goal_arrow(None)
        self.map_view.set_path([])            # 清除路径残影
        self._last_path_rx = 0.0
        self.map_view.update_queue([], -1)    # 清除地图上的队列旗帜
        self.log('已停止导航（队列已清空，机器人已停车）')

    # ================================================== 电子围栏管理（按地图分组）
    @property
    def _keepout_file(self):
        return self.profile.keepout_file

    def _load_keepout(self):
        keepout_file = self._keepout_file
        try:
            data = {}
            if os.path.exists(keepout_file):
                with open(keepout_file, encoding='utf-8') as f:
                    data = json.load(f)
            self._all_keepout = data.get('maps', {}) if isinstance(data, dict) else {}
            key = self.current_map or '默认地图'
            self._keepout_zones = self._all_keepout.get(key, [])
        except Exception as e:
            self.log(f'电子围栏文件读取失败: {e}')
            self._keepout_zones = []
            self._all_keepout = {}
        self.map_view.set_keepout_zones(self._keepout_zones)

    def _save_keepout(self):
        keepout_file = self._keepout_file
        try:
            os.makedirs(os.path.dirname(keepout_file), exist_ok=True)
            key = self.current_map or '默认地图'
            self._all_keepout[key] = self._keepout_zones
            with open(keepout_file, 'w', encoding='utf-8') as f:
                json.dump({'maps': self._all_keepout}, f,
                          ensure_ascii=False, indent=2)
        except Exception as e:
            self.log(f'电子围栏保存失败: {e}')

    def _point_in_keepout(self, x, y):
        """点是否落在任一禁区多边形内（射线法，含边界）。"""
        for z in self._keepout_zones:
            poly = z.get('points', [])
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
            if inside:
                return True
        return False

    def _toggle_draw_keepout(self, checked):
        if checked:
            self.btn_set_pose.setChecked(False)
            self.btn_add_goal.setChecked(False)
            self.map_view.set_mode(MapView.MODE_DRAW_KEEPOUT)
            self.log('禁区绘制：左键逐点添加，右键闭合（≥3 点），再点「＋禁区」退出')
        else:
            self.map_view.set_mode(MapView.MODE_PAN)

    def _on_keepout_drawn(self, points):
        """禁区闭合完成（主线程）：命名 → 保存 → 显示。"""
        name, ok = QInputDialog.getText(self, '新建禁区', '禁区名称：')
        if not ok or not name.strip():
            self.map_view.set_mode(MapView.MODE_PAN)
            self.btn_draw_keepout.setChecked(False)
            return
        name = name.strip()
        self._keepout_zones.append({'name': name,
                                    'points': [[round(x, 3), round(y, 3)]
                                               for x, y in points]})
        self._save_keepout()
        self.map_view.set_keepout_zones(self._keepout_zones)
        self.map_view.set_mode(MapView.MODE_PAN)
        self.btn_draw_keepout.setChecked(False)
        self.log(f'已添加禁区: {name}（{len(points)} 个顶点）')

    def _clear_keepout(self):
        if not self._keepout_zones:
            # 兜底：内存里没有禁区，但屏幕可能残留未闭合的绘制草稿
            # （虚线轮廓）—— 强制退出绘制模式并刷新一次显示。
            self.map_view.set_mode(MapView.MODE_PAN)
            self.btn_draw_keepout.setChecked(False)
            self.map_view.set_keepout_zones([])
            QMessageBox.information(self, '提示',
                                    '当前地图没有已保存的禁区\n'
                                    '（画面上的红色虚线轮廓是未闭合的绘制草稿，已清除）')
            return
        ret = QMessageBox.question(
            self, '清空禁区', f'将删除当前地图全部 {len(self._keepout_zones)} 个禁区，确定？',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if ret != QMessageBox.Yes:
            return
        self._keepout_zones = []
        self._save_keepout()
        self.map_view.set_keepout_zones([])
        self.log('已清空当前地图的电子围栏禁区')

    # ================================================== 点位管理（按地图分组）
    @property
    def _points_file(self):
        return self.profile.points_file

    def _load_points(self):
        points_file = self._points_file
        try:
            data = {}
            if os.path.exists(points_file):
                with open(points_file, encoding='utf-8') as f:
                    data = json.load(f)
            # 兼容旧格式 {"points": [...]} → 迁移到"默认地图"名下
            if isinstance(data, dict) and 'points' in data:
                self._all_points = {'默认地图': data['points']}
            else:
                self._all_points = data.get('maps', {}) if isinstance(data, dict) else {}
            key = self.current_map or '默认地图'
            self.waypoints = self._all_points.get(key, [])
        except Exception as e:
            self.log(f'点位文件读取失败: {e}')
            self.waypoints = []
            self._all_points = {}
        self._refresh_points_ui()

    def _save_points(self):
        points_file = self._points_file
        try:
            os.makedirs(os.path.dirname(points_file), exist_ok=True)
            key = self.current_map or '默认地图'
            self._all_points[key] = self.waypoints
            with open(points_file, 'w', encoding='utf-8') as f:
                json.dump({'maps': self._all_points}, f,
                          ensure_ascii=False, indent=2)
        except Exception as e:
            self.log(f'点位保存失败: {e}')

    def _refresh_points_ui(self):
        self.point_list.clear()
        for p in self.waypoints:
            item = QListWidgetItem(
                f"{p['name']}   ({p['x']:.2f}, {p['y']:.2f})")
            self.point_list.addItem(item)
        self.map_view.update_waypoints(self.waypoints)
        self.lbl_points.setText(
            f'点位列表[{self.current_map or "默认地图"}]（双击导航）：')

    def _add_point(self):
        pose = getattr(self, '_robot_pose', None)
        if pose is None:
            QMessageBox.warning(self, '提示', '机器人定位不可用，无法记录点位')
            return
        name, ok = QInputDialog.getText(self, '新建点位', '点位名称：')
        if not ok or not name.strip():
            return
        name = name.strip()
        if any(p['name'] == name for p in self.waypoints):
            QMessageBox.warning(self, '提示', '点位名称已存在')
            return
        x, y, yaw = pose
        self.waypoints.append({'name': name, 'x': x, 'y': y, 'yaw': yaw})
        self._save_points()
        self._refresh_points_ui()
        self.log(f'已保存点位: {name} ({x:.2f}, {y:.2f})')

    def _del_point(self):
        row = self.point_list.currentRow()
        if row < 0 or row >= len(self.waypoints):
            return
        name = self.waypoints[row]['name']
        del self.waypoints[row]
        self._save_points()
        self._refresh_points_ui()
        self.log(f'已删除点位: {name}')

    def _goto_point(self, item):
        row = self.point_list.row(item)
        if row < 0 or row >= len(self.waypoints):
            return
        if self.current_mode != 'navigation':
            QMessageBox.warning(self, '提示', '请先切换到导航模式')
            return
        p = self.waypoints[row]
        self._add_to_queue(p['name'], p['x'], p['y'], p.get('yaw', 0.0))

    # ================================================== 手动遥控
    def _teleop_speeds(self):
        return (self.slider_linear.value() / 100.0,
                self.slider_angular.value() / 100.0)

    def _teleop_key_down(self, key):
        self.keys_down.add(key)

    def _teleop_key_up(self, key):
        self.keys_down.discard(key)

    def _teleop_stop(self):
        self.keys_down.clear()
        self.bridge.publish_cmd(0.0, 0.0)

    def _send_teleop(self):
        if not self.keys_down:
            return
        # 导航进行中不发送手动指令，避免与导航输出冲突
        if self.navigating:
            # 提示一次即可，避免 20Hz 刷屏：说明按键被导航状态拦下了
            if not getattr(self, '_teleop_block_notified', False):
                self._teleop_block_notified = True
                self.log('手动遥控被忽略：当前正在导航，请先点「停止导航」')
            return
        self._teleop_block_notified = False
        lin, ang = self._teleop_speeds()
        vx, vz = 0.0, 0.0
        if 'forward' in self.keys_down:
            vx += lin
        if 'back' in self.keys_down:
            vx -= lin
        if 'left' in self.keys_down:
            vz += ang
        if 'right' in self.keys_down:
            vz -= ang
        self.bridge.publish_cmd(vx, vz)

    # ---- 键盘遥控：应用级事件过滤器（不受焦点影响） ----
    _TELEOP_KEYMAP = None   # 延迟到 eventFilter 里初始化（依赖 Qt）

    def _teleop_keymap(self):
        if MainWindow._TELEOP_KEYMAP is None:
            MainWindow._TELEOP_KEYMAP = {
                Qt.Key_W: 'forward', Qt.Key_S: 'back',
                Qt.Key_A: 'left', Qt.Key_D: 'right',
            }
        return MainWindow._TELEOP_KEYMAP

    def eventFilter(self, obj, event):
        """全局键盘遥控：W/A/S/D 控车，X 全局急停。不受焦点影响。"""
        et = event.type()
        if et not in (QEvent.KeyPress, QEvent.KeyRelease):
            return super().eventFilter(obj, event)
        if event.isAutoRepeat():
            return False
        # 有文本输入控件获得焦点时让路（避免打断用户打字）
        if isinstance(obj, (QLineEdit, QPlainTextEdit, QTextEdit)):
            return super().eventFilter(obj, event)
        key = event.key()
        kmap = self._teleop_keymap()
        if et == QEvent.KeyPress:
            if key in kmap:
                self._teleop_key_down(kmap[key])
                return True
            if key == Qt.Key_X:
                # P1：X 键升级为全局急停（原来只发零速，不清导航队列）
                self._emergency_stop()
                return True
        else:  # KeyRelease
            if key in kmap:
                self._teleop_key_up(kmap[key])
                if not self.keys_down:
                    self.bridge.publish_cmd(0.0, 0.0)
                return True
        return super().eventFilter(obj, event)

    def keyPressEvent(self, event):
        # 兼容保留：过滤器已覆盖绝大多数情况；此处兜底。
        if event.isAutoRepeat():
            return
        mapping = {Qt.Key_W: 'forward', Qt.Key_S: 'back',
                   Qt.Key_A: 'left', Qt.Key_D: 'right'}
        key = event.key()
        if key in mapping:
            self._teleop_key_down(mapping[key])
        elif key == Qt.Key_X:
            self._emergency_stop()
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        mapping = {Qt.Key_W: 'forward', Qt.Key_S: 'back',
                   Qt.Key_A: 'left', Qt.Key_D: 'right'}
        key = event.key()
        if key in mapping:
            self._teleop_key_up(mapping[key])
            # 所有方向键都松开时发一次零速停车
            if not self.keys_down:
                self.bridge.publish_cmd(0.0, 0.0)
        else:
            super().keyReleaseEvent(event)

    # ================================================== 日志工具条
    def _build_log_toolbar(self):
        """日志区工具条：复制全部 / 复制选中 / 清空 / 放大还原。

        复制全部复制的是**未截断**的原始日志 —— log() 会把超过 300 字符的
        单行截断显示，排查问题时被截断的部分往往是关键。
        """
        bar = QHBoxLayout()
        bar.setContentsMargins(0, 0, 0, 0)
        bar.setSpacing(6)

        title = QLabel('日志')
        title.setObjectName('logTitle')
        bar.addWidget(title)

        self.btn_copy_log = QPushButton('复制全部')
        self.btn_copy_log.setObjectName('logBtn')
        self.btn_copy_log.setToolTip(
            '把当前所有日志（含被显示截断的完整内容）复制到剪贴板')
        self.btn_copy_log.clicked.connect(self._copy_all_logs)
        bar.addWidget(self.btn_copy_log)

        btn_copy_sel = QPushButton('复制选中')
        btn_copy_sel.setObjectName('logBtn')
        btn_copy_sel.setToolTip('只复制日志框里鼠标选中的部分')
        btn_copy_sel.clicked.connect(self._copy_selected_logs)
        bar.addWidget(btn_copy_sel)

        btn_clear = QPushButton('清空')
        btn_clear.setObjectName('logBtn')
        btn_clear.setToolTip('清空日志显示（不影响正在运行的进程）')
        btn_clear.clicked.connect(self._clear_logs)
        bar.addWidget(btn_clear)

        bar.addStretch(1)

        self.btn_log_expand = QPushButton('放大')
        self.btn_log_expand.setCheckable(True)
        self.btn_log_expand.setObjectName('logBtn')
        self.btn_log_expand.setToolTip(
            '把日志区放大到占下半屏，再点一次还原。\n'
            '也可以直接拖动中间的分隔条，位置会被记住。')
        self.btn_log_expand.toggled.connect(self._toggle_log_expanded)
        bar.addWidget(self.btn_log_expand)

        return bar

    # ================================================== 日志控制
    def _copy_all_logs(self):
        """复制全部日志（未截断的完整内容）。"""
        text = '\n'.join(self._log_full)
        if not text:
            QMessageBox.information(self, '提示', '当前没有日志可复制。')
            return
        QApplication.clipboard().setText(text)
        self.log(f'已复制 {len(self._log_full)} 行日志到剪贴板')

    def _copy_selected_logs(self):
        """复制鼠标选中的部分。"""
        cur = self.log_box.textCursor()
        if not cur.hasSelection():
            QMessageBox.information(self, '提示',
                                    '请先在日志框里用鼠标选中要复制的内容。')
            return
        QApplication.clipboard().setText(cur.selectedText())
        self.log('已复制选中内容到剪贴板')

    def _clear_logs(self):
        self.log_box.clear()
        self._log_full.clear()

    def _toggle_log_expanded(self, expanded):
        """放大日志区 / 还原。用总高度的一半作为放大目标。"""
        if not hasattr(self, '_main_splitter'):
            return
        sp = self._main_splitter
        total = max(sp.height(), sp.sizeHint().height(), 400)
        if expanded:
            log_h = max(240, int(total * 0.5))
            self.btn_log_expand.setText('还原')
        else:
            log_h = self._LOG_DEFAULT_H
            self.btn_log_expand.setText('放大')
        sp.setSizes([max(120, total - log_h), log_h])

    def _restore_splitter_sizes(self):
        """恢复上次的日志区高度；没有记录则用默认值。"""
        sp = self._main_splitter
        saved = QSettings().value('ui/log_height', None, type=int)
        total = max(sp.height(), sp.sizeHint().height(), 400)
        log_h = saved if (saved and 44 <= saved < total - 120) \
            else self._LOG_DEFAULT_H
        sp.setSizes([max(120, total - log_h), log_h])

    def _save_splitter_sizes(self):
        """记住日志区高度，下次启动保持用户调好的布局。"""
        if not hasattr(self, '_main_splitter'):
            return
        sizes = self._main_splitter.sizes()
        if len(sizes) == 2:
            QSettings().setValue('ui/log_height', sizes[1])

    # ================================================== 日志与退出
    def log(self, text):
        # ⚠️ 线程安全：RemoteHost 的 SSH 调用全在后台线程里跑（_remote_task），
        # 它每一步都会 log()；如果这里直接碰 log_box（QPlainTextEdit），
        # 就是从非主线程操作 QWidget —— 表现为随机段错误崩溃。
        # 后台线程一律经 log_signal 转发，真正的写入只在主线程做。
        main_thread = getattr(self, '_main_thread', None)
        if main_thread is not None and threading.current_thread() is not main_thread:
            self.log_signal.emit(str(text))
            return
        self._log_impl(text)

    def _log_impl(self, text):
        # 显示时截断超长行（避免一行刷屏），但内部保留完整原文，
        # 「复制全部」才能拿到被截断的部分。
        self._log_full.append(text)
        if len(self._log_full) > self._LOG_MAX_BLOCKS:
            del self._log_full[:-self._LOG_MAX_BLOCKS]
        shown = text if len(text) <= 300 else text[:300] + '...'
        self.log_box.appendPlainText(shown)

    def closeEvent(self, event):
        self._save_splitter_sizes()
        # ------------------------------------------------------------
        # 【关键】关窗必须**同步**把车上的进程停掉。
        # 以前走 daemon 后台线程停 —— 主进程一退出 daemon 线程立即被杀，
        # 而 SSH 停止命令要 5~20s，结果"窗口关了车上的 launch 还在跑"。
        # 这里同步等待（最多 ~20s），确保真的杀干净再退出。
        #
        # ⚠️ 不要用 is_running() 做前置判断：launch 一旦退出，雷达、底盘
        # 驱动这些子节点会变成孤儿继续活着 —— stop() 自己是"有就停、
        # 没有就 NOOP 立即返回"，所以可以无条件调用。
        # ------------------------------------------------------------
        if self.remote:
            try:
                self.log('正在停止工控机上的系统（请稍候，勿关闭窗口）...')
                QApplication.processEvents()
                ok, msg = self.remote.stop()
                self.log(f'工控机停止结果：{msg}')
                QApplication.processEvents()
            except Exception as e:                              # noqa: BLE001
                self.log(f'[远程] 关闭时停止异常: {e}')
        # 复位模式状态（不再弹保存提示 —— 车上进程已在上一步停掉）
        self._stop_all_modes(ask_save=False)
        # 关闭主窗口时一并关掉弹出的传感器窗口（面板会先归还到标签页）
        if self.sensor_win is not None:
            self.sensor_win.close()
        event.accept()
