"""传感器监控面板：雷达扫描范围、各传感器量程/状态/障碍距离、导航与避障决策。

组成：
  SensorRadarWidget  极坐标雷达图：量程环、FOV 扇区、实时障碍点、超声波束
  SensorPanel        右侧面板：雷达/超声/RGB-D 列表 + 决策输出区
"""
import math
import time

import numpy as np
from PyQt5.QtCore import Qt, QRectF, QPointF, pyqtSignal, QTimer
from PyQt5.QtGui import (QColor, QFont, QPainter, QPen, QBrush, QPolygonF,
                         QRadialGradient)
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel,
                             QGroupBox, QFrame, QSizePolicy, QTableWidget,
                             QTableWidgetItem,                              QHeaderView, QAbstractItemView,
                             QPlainTextEdit, QPushButton)

# 传感器量程配置（与 urdf/dt01d.urdf.xacro、config/sim/nav2_params_dt01d.yaml 对齐）
SENSOR_SPECS = {
    'scan': {
        'name': '2D LiDAR(合并)',
        'min_range': 0.32,
        'max_range': 30.0,
        'display_range': 6.0,   # 雷达图上显示的最大半径（m）
        'color': QColor(53, 211, 224),
        'note': '360°  建图/定位/前向+侧向避障',
    },
    'camera_l': {
        'name': 'RGB-D 左',
        'min_range': 0.05,
        'max_range': 10.0,
        'fov': math.radians(87.0),
        'color': QColor(255, 154, 61),
        'note': '外八 45° 俯 -8°  玻璃/悬空障碍',
    },
    'camera_r': {
        'name': 'RGB-D 右',
        'min_range': 0.05,
        'max_range': 10.0,
        'fov': math.radians(87.0),
        'color': QColor(255, 154, 61),
        'note': '外八 -45° 俯 -8°  玻璃/悬空障碍',
    },
    'sonar_fl': {'name': '超声 前左', 'min_range': 0.03, 'max_range': 4.0,
                 'fov': math.radians(30.0), 'bearing': math.radians(15.0),
                 'color': QColor(110, 231, 160)},
    'sonar_fr': {'name': '超声 前右', 'min_range': 0.03, 'max_range': 4.0,
                 'fov': math.radians(30.0), 'bearing': math.radians(-15.0),
                 'color': QColor(110, 231, 160)},
    'sonar_sl': {'name': '超声 左侧', 'min_range': 0.03, 'max_range': 4.0,
                 'fov': math.radians(30.0), 'bearing': math.radians(90.0),
                 'color': QColor(110, 231, 160)},
    'sonar_sr': {'name': '超声 右侧', 'min_range': 0.03, 'max_range': 4.0,
                 'fov': math.radians(30.0), 'bearing': math.radians(-90.0),
                 'color': QColor(110, 231, 160)},
    'sonar_rl': {'name': '超声 后左', 'min_range': 0.03, 'max_range': 4.0,
                 'fov': math.radians(30.0), 'bearing': math.radians(165.0),
                 'color': QColor(110, 231, 160)},
    'sonar_rr': {'name': '超声 后右', 'min_range': 0.03, 'max_range': 4.0,
                 'fov': math.radians(30.0), 'bearing': math.radians(-165.0),
                 'color': QColor(110, 231, 160)},
    'imu': {
        'name': 'IMU',
        'min_range': 0.0,
        'max_range': 0.0,
        'color': QColor(180, 160, 255),
        'note': '航向/角速度，不测距',
    },
}
SONAR_ORDER = ['sonar_fl', 'sonar_fr', 'sonar_sl',
               'sonar_sr', 'sonar_rl', 'sonar_rr']
CAMERA_ORDER = ['camera_l', 'camera_r']

# 各车型实际配备的传感器（与 urdf/*.xacro 一致）。
# 面板据此只显示本车型有的行，未配备的隐藏而不是显示成"无数据"。
#   dt01  : 顶部单雷达 + IMU
#   dt01d : 双前置雷达(合并为 /scan) + 双 RGB-D + 6 路超声 + IMU
ROBOT_SENSORS = {
    'dt01': ['scan', 'imu'],
    'dt01d': (['scan'] + CAMERA_ORDER + SONAR_ORDER + ['imu']),
}

# 距离分级配色（近 → 远）：红 / 橙 / 黄 / 绿
LEVEL_COLORS = [
    (0.35, QColor(255, 70, 70), '危险'),
    (0.60, QColor(255, 160, 50), '警戒'),
    (1.00, QColor(255, 215, 70), '减速'),
]
SAFE_COLOR = QColor(110, 231, 160)
OFFLINE_COLOR = QColor(150, 160, 175)
# 超过该时长没收到数据即判定为离线（话题不存在 / 节点未启动）
DATA_TIMEOUT = 3.0


def level_color(dist):
    """按距离返回 (配色, 等级名)。inf 视为安全。"""
    if dist is None or not np.isfinite(dist):
        return SAFE_COLOR, '空闲'
    for thr, color, name in LEVEL_COLORS:
        if dist < thr:
            return color, name
    return SAFE_COLOR, '安全'


def fmt_dist(dist):
    """距离格式化：inf 显示为 '—'（射空，不是障碍）。"""
    if dist is None or not np.isfinite(dist):
        return '—'
    return f'{dist:.2f} m'


class SensorRadarWidget(QWidget):
    """极坐标雷达图：量程环 + 超声扇区 + 实时障碍点 + 最近障碍标注。

    机器人位于中心，正上方为车头方向，角度逆时针为正（与 ROS 一致）。
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(260, 260)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setToolTip('极坐标视图：中心为机器人，正上方为车头。'
                        '实心点=激光回波，空心圆环=超声回波，橙扇=RGB-D 视场；'
                        '颜色按距离分级（红=危险近，绿=安全远）。')

        self.display_range = SENSOR_SPECS['scan']['display_range']
        self.scan_ranges = None        # np.ndarray，与角度数组一一对应
        self.scan_angle_min = -math.pi
        self.scan_angle_inc = 2.0 * math.pi / 1440.0
        self.sonar_state = {}
        self.camera_state = {}
        self.robot_pose = None
        self.nearest = (float('inf'), None)   # (距离, 传感器名)
        self.active = set(ROBOT_SENSORS['dt01'])

    # ================= 数据更新 =================
    def set_scan(self, msg):
        """接收合并后的 /scan：只保留有效回波，射空值丢弃。"""
        if msg is None:
            self.scan_ranges = None
            self.update()
            return
        r = np.asarray(msg.ranges, dtype=np.float64)
        self.scan_angle_min = msg.angle_min
        self.scan_angle_inc = msg.angle_increment
        # 与 ros_bridge.scan_to_points 同一套过滤规则：
        # 排除 NaN/inf、range_min 以内（自身结构）以及等于 range_max 的射空值
        valid = np.isfinite(r)
        valid &= (r >= max(msg.range_min, 0.0))
        valid &= (r < min(msg.range_max, self.display_range))
        self.scan_ranges = np.where(valid, r, np.nan)
        self.update()

    def set_sonar(self, topic, state):
        self.sonar_state[topic] = state
        self.update()

    def set_camera(self, topic, state):
        self.camera_state[topic] = state
        self.update()

    def set_pose(self, pose):
        self.robot_pose = pose
        self.update()

    def set_active(self, active):
        """按车型只绘制本车配备的传感器（未配备的不画 FOV 扇区）。"""
        self.active = set(active)
        self.update()

    # ================= 绘制 =================
    def _geom(self):
        """返回 (中心点, 每米对应的像素数)。"""
        w, h = self.width(), self.height()
        cx, cy = w / 2.0, h / 2.0
        radius = min(w, h) / 2.0 - 14.0
        return cx, cy, max(radius / self.display_range, 1e-6)

    def _to_px(self, cx, cy, scale, angle, dist):
        """机器人极坐标（angle：0=车头正前，正值=向左）→ 屏幕像素。"""
        # 屏幕 y 向下，车头朝上；机器人向左（+angle）对应屏幕向左（-x）
        return cx - dist * scale * math.sin(angle), cy - dist * scale * math.cos(angle)

    @staticmethod
    def _to_px_xy(cx, cy, scale, x, y):
        """机器人直角坐标（x 前、y 左）→ 屏幕像素。"""
        return cx - y * scale, cy - x * scale

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        cx, cy, scale = self._geom()
        radius = self.display_range * scale

        # ---- 背景 ----
        p.fillRect(self.rect(), QColor(20, 23, 29))
        glow = QRadialGradient(cx, cy, 0, cx, cy, radius)
        glow.setColorAt(0.0, QColor(30, 40, 48))
        glow.setColorAt(1.0, QColor(20, 23, 29))
        p.setBrush(QBrush(glow))
        p.setPen(Qt.NoPen)
        p.drawEllipse(QPointF(cx, cy), radius, radius)

        # ---- 距离环 + 刻度 ----
        p.setFont(QFont('monospace', 8))
        for d in (1.0, 2.0, 3.0, 4.0, 5.0, 6.0):
            if d > self.display_range:
                break
            rr = d * scale
            p.setPen(QPen(QColor(60, 72, 88), 1))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(QPointF(cx, cy), rr, rr)
            p.setPen(QColor(110, 128, 150))
            p.drawText(QPointF(cx + 3, cy - rr + 11), f'{d:.0f}m')

        # ---- 十字与车头方向 ----
        p.setPen(QPen(QColor(60, 72, 88), 1))
        p.drawLine(QPointF(cx - radius, cy), QPointF(cx + radius, cy))
        p.drawLine(QPointF(cx, cy - radius), QPointF(cx, cy + radius))
        p.setPen(QPen(QColor(230, 237, 243), 2))
        p.drawLine(QPointF(cx, cy), QPointF(cx, cy - radius))

        # ---- RGB-D 视场扇区 ----
        for cam in CAMERA_ORDER:
            if cam not in self.active:
                continue
            spec = SENSOR_SPECS[cam]
            fov = spec['fov']
            start = int(math.degrees(math.pi / 2 - fov / 2))
            span = int(math.degrees(fov))
            rr = min(spec['max_range'], self.display_range) * scale
            p.setBrush(QBrush(QColor(255, 154, 61, 26)))
            p.setPen(QPen(QColor(255, 154, 61, 120), 1))
            p.drawPie(QRectF(cx - rr, cy - rr, 2 * rr, 2 * rr),
                      start * 16, span * 16)

        # ---- 超声波束扇区（命中则按距离着色）----
        for name in SONAR_ORDER:
            if name not in self.active:
                continue
            spec = SENSOR_SPECS[name]
            st = self.sonar_state.get('/' + name, {})
            bearing = spec['bearing']
            fov = spec['fov'] / 2.0
            active = bool(st.get('active'))
            dist = st.get('obstacle_dist', float('inf'))
            rr = min(spec['max_range'], self.display_range) * scale
            if active and np.isfinite(dist):
                color, _ = level_color(dist)
                p.setBrush(QBrush(QColor(color.red(), color.green(),
                                         color.blue(), 70)))
                p.setPen(QPen(color, 2))
            else:
                p.setBrush(QBrush(QColor(110, 231, 160, 22)))
                p.setPen(QPen(QColor(110, 231, 160, 70), 1))
            # Qt 的 drawPie：0° 在 3 点方向，逆时针为正，单位为 1/16 度
            # （0=右, 90=上, 180=左, 270=下）。机器人方位 bearing（0=正前，
            # 正值=向左）→ Qt 角度 90°+bearing；扇区 [bearing-fov, bearing+fov]
            # 对应 start = 90+bearing-fov，逆时针扫 2*fov。
            start_deg = 90.0 + math.degrees(bearing - fov)
            span_deg = math.degrees(2.0 * fov)
            p.drawPie(QRectF(cx - rr, cy - rr, 2 * rr, 2 * rr),
                      int(round(start_deg)) * 16, int(round(span_deg)) * 16)

        # ---- 激光回波点 ----
        nearest = float('inf')
        nearest_name = None
        if self.scan_ranges is not None and self.scan_ranges.size:
            idx = np.nonzero(~np.isnan(self.scan_ranges))[0]
            if idx.size:
                angles = self.scan_angle_min + idx * self.scan_angle_inc
                dists = self.scan_ranges[idx]
                # 抽稀到大屏约 900 点，避免绘制卡顿
                step = max(1, idx.size // 900)
                for a, d in zip(angles[::step], dists[::step]):
                    color, _ = level_color(d)
                    x, y = self._to_px(cx, cy, scale, a, d)
                    p.setPen(Qt.NoPen)
                    p.setBrush(QBrush(color))
                    p.drawEllipse(QPointF(x, y), 2.0, 2.0)
                dmin = float(np.min(dists))
                if dmin < nearest:
                    nearest, nearest_name = dmin, '激光'

        # ---- 超声障碍点（空心圆环，与激光实心点区分来源）----
        for name in SONAR_ORDER:
            if name not in self.active:
                continue
            spec = SENSOR_SPECS[name]
            st = self.sonar_state.get('/' + name, {})
            if not st.get('active'):
                continue
            d = st.get('obstacle_dist', float('inf'))
            if not np.isfinite(d):
                continue
            color, _ = level_color(d)
            x, y = self._to_px(cx, cy, scale, spec['bearing'], d)
            p.setPen(QPen(color, 2))
            p.setBrush(Qt.NoBrush)          # 空心圆环 = 超声
            p.drawEllipse(QPointF(x, y), 4.0, 4.0)

        # ---- 超声最近距离标注 ----
        for name in SONAR_ORDER:
            if name not in self.active:
                continue
            st = self.sonar_state.get('/' + name, {})
            if not st.get('active'):
                continue
            d = st.get('obstacle_dist', float('inf'))
            if np.isfinite(d) and d < nearest:
                nearest = d
                nearest_name = SENSOR_SPECS[name]['name']

        # ---- RGB-D 最近距离标注 ----
        for cam in CAMERA_ORDER:
            if cam not in self.active:
                continue
            st = self.camera_state.get(cam, {})
            if not st.get('active'):
                continue
            d = st.get('min_dist', float('inf'))
            if np.isfinite(d) and d < nearest:
                nearest = d
                nearest_name = SENSOR_SPECS[cam]['name']

        self.nearest = (nearest, nearest_name)

        # ---- 车体轮廓：真实的 footprint 矩形（x 前 0.38 / 后 -0.28，y 左右 ±0.24）----
        body = QPolygonF([
            QPointF(*self._to_px_xy(cx, cy, scale, 0.38, 0.24)),
            QPointF(*self._to_px_xy(cx, cy, scale, 0.38, -0.24)),
            QPointF(*self._to_px_xy(cx, cy, scale, -0.28, -0.24)),
            QPointF(*self._to_px_xy(cx, cy, scale, -0.28, 0.24)),
        ])
        p.setPen(QPen(QColor(90, 170, 255), 2))
        p.setBrush(QBrush(QColor(90, 170, 255, 90)))
        p.drawPolygon(body)

        # ---- 最近障碍读数 ----
        p.setFont(QFont('monospace', 10, QFont.Bold))
        if nearest_name and np.isfinite(nearest):
            color, level = level_color(nearest)
            p.setPen(color)
            p.drawText(QRectF(6, 4, self.width() - 12, 18), Qt.AlignLeft,
                       f'最近障碍 {nearest:.2f} m  [{level}]  {nearest_name}')
        else:
            p.setPen(QColor(110, 231, 160))
            p.drawText(QRectF(6, 4, self.width() - 12, 18), Qt.AlignLeft,
                       '最近障碍 —  [空闲]')

        # ---- 图例（形状区分来源：激光=实心点，超声=空心圆环，RGB-D=视场色块）----
        p.setFont(QFont('monospace', 8))
        legend = [
            ('激光', QColor(255, 70, 70), 'dot'),
            ('超声', QColor(255, 70, 70), 'ring'),
            ('RGB-D', QColor(255, 154, 61), 'box'),
        ]
        # 图例只列本车型配备的传感器
        if CAMERA_ORDER[0] not in self.active:
            legend = [l for l in legend if l[0] != 'RGB-D']
        if SONAR_ORDER[0] not in self.active:
            legend = [l for l in legend if l[0] != '超声']
        x = 6.0
        for text, color, kind in legend:
            if kind == 'dot':
                p.setPen(Qt.NoPen)
                p.setBrush(QBrush(color))
                p.drawEllipse(QPointF(x + 4, self.height() - 10), 3.5, 3.5)
            elif kind == 'ring':
                p.setPen(QPen(color, 2))
                p.setBrush(Qt.NoBrush)
                p.drawEllipse(QPointF(x + 4, self.height() - 10), 3.5, 3.5)
            else:  # box：RGB-D 视场
                p.setPen(Qt.NoPen)
                p.setBrush(QBrush(color))
                p.drawRect(QRectF(x + 1, self.height() - 13, 7, 7))
            p.setPen(QColor(150, 165, 185))
            p.drawText(QPointF(x + 12, self.height() - 6), text)
            x += 58


class SensorPanel(QWidget):
    """传感器监控面板：雷达图 + 传感器状态表 + 决策输出。

    既可以作为标签页内嵌在右侧面板，也可以弹出成独立窗口
    （导航时需要同时看地图和传感器，弹出更实用）。
    """

    # 请求弹出为独立窗口 / 放回标签页
    popout_requested = pyqtSignal(bool)

    def __init__(self, parent=None, robot_type='dt01'):
        super().__init__(parent)
        self.radar = SensorRadarWidget()
        self.robot_type = robot_type
        self.floating = False
        # 每个传感器最后一次收到数据的时刻（monotonic 秒），用于离线判定
        self._last_update = {}
        # 首次接收标记：用于在终端打印「首次收到 topic X」日志，便于排查信号链路
        self._first_seen = {}
        # 每个传感器最近 N 次的接收时间戳，用于实时显示频率
        self._rx_times = {key: [] for key in ROBOT_SENSORS['dt01d']}
        self._build_ui()
        self.set_active_sensors(robot_type)

        # 离线看门狗：话题不存在 / 节点没起时，把对应行标成"离线"，
        # 而不是一直显示含糊的"无数据"
        self._offline_timer = QTimer(self)
        self._offline_timer.timeout.connect(self._check_offline)
        self._offline_timer.start(1000)

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(8)

        # 弹出/停靠切换：导航时把传感器弹成独立窗口，可同时看地图和传感器
        bar = QHBoxLayout()
        bar.setContentsMargins(0, 0, 0, 0)
        self.btn_popout = QPushButton('⧉ 弹出为独立窗口')
        self.btn_popout.setObjectName('typeBtn')
        self.btn_popout.setToolTip('弹出为独立窗口，可同时观察地图与传感器实时状态')
        self.btn_popout.clicked.connect(self._on_popout_clicked)
        bar.addStretch(1)
        bar.addWidget(self.btn_popout)
        root.addLayout(bar)

        # ---- 雷达扫描范围 ----
        box_scope = QGroupBox('传感器扫描范围')
        v = QVBoxLayout(box_scope)
        v.addWidget(self.radar, 1)
        self.lbl_scope = QLabel('量程：雷达 0.32~30.00 m（显示 6 m） | '
                                'RGB-D 0.05~10.00 m | 超声 0.03~4.00 m')
        self.lbl_scope.setObjectName('statusVal')
        self.lbl_scope.setWordWrap(True)
        self.lbl_scope.setStyleSheet('font-size: 11px; color: #8b9bb4;')
        v.addWidget(self.lbl_scope)
        root.addWidget(box_scope, 3)

        # ---- 接收诊断：实时频率 + 最后一次接收时间，1Hz 刷新 ----
        self.lbl_diag = QLabel('诊断：等待数据...')
        self.lbl_diag.setObjectName('statusVal')
        self.lbl_diag.setStyleSheet(
            'font-family: monospace; font-size: 11px; color: #8b9bb4;')
        self.lbl_diag.setWordWrap(True)
        root.addWidget(self.lbl_diag)
        self._diag_timer = QTimer(self)
        self._diag_timer.timeout.connect(self._refresh_diag)
        self._diag_timer.start(1000)

        # ---- 传感器实时状态表 ----
        box_table = QGroupBox('传感器实时状态')
        tv = QVBoxLayout(box_table)
        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(['传感器', '量程', '障碍距离', '状态'])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionMode(QAbstractItemView.NoSelection)
        self.table.setAlternatingRowColors(True)
        hh = self.table.horizontalHeader()
        hh.setSectionResizeMode(0, QHeaderView.Stretch)
        hh.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(3, QHeaderView.ResizeToContents)
        self.table.setMinimumHeight(160)
        self._rows = {}
        self._add_row('scan', SENSOR_SPECS['scan']['name'], '0.32~30.0 m')
        for cam in CAMERA_ORDER:
            self._add_row(cam, SENSOR_SPECS[cam]['name'], '0.05~10.0 m')
            # 相机点云订阅已禁用（rclpy 无法反序列化 gazebo PointCloud2），
            # 明确标"未接入"，避免和"离线无话题"（数据链路故障）混淆
            self._set_row(cam, '—', '未接入(点云格式不兼容)', OFFLINE_COLOR)
        for name in SONAR_ORDER:
            spec = SENSOR_SPECS[name]
            self._add_row(name, spec['name'],
                          f'{spec["min_range"]:.2f}~{spec["max_range"]:.1f} m')
        self._add_row('imu', 'IMU', '不测距')
        tv.addWidget(self.table)
        root.addWidget(box_table, 2)

        # ---- 决策输出 ----
        box_dec = QGroupBox('导航 / 避障决策')
        dv = QVBoxLayout(box_dec)
        self.lbl_nav = QLabel('导航决策：空闲')
        self.lbl_avoid = QLabel('避障决策：无障碍')
        self.lbl_stop = QLabel('停车原因：—')
        self.lbl_next = QLabel('下一步：—')
        for lbl in (self.lbl_nav, self.lbl_avoid, self.lbl_stop, self.lbl_next):
            lbl.setObjectName('statusVal')
            lbl.setWordWrap(True)
            lbl.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.lbl_nav.setStyleSheet('color: #7fd0ff; font-family: monospace;')
        self.lbl_avoid.setStyleSheet('color: #6ee7a0; font-family: monospace;')
        self.lbl_stop.setStyleSheet('color: #ffb454; font-family: monospace;')
        self.lbl_next.setStyleSheet('color: #c7a2ff; font-family: monospace;')
        dv.addWidget(self.lbl_nav)
        dv.addWidget(self.lbl_avoid)
        dv.addWidget(self.lbl_stop)
        dv.addWidget(self.lbl_next)

        sep = QFrame(); sep.setObjectName('sep'); sep.setFrameShape(QFrame.HLine)
        dv.addWidget(sep)
        self.decision_log = QPlainTextEdit()
        self.decision_log.setReadOnly(True)
        self.decision_log.setMaximumBlockCount(300)
        self.decision_log.setMinimumHeight(110)
        self.decision_log.setPlaceholderText('决策变更日志（仅在状态变化时输出）')
        dv.addWidget(self.decision_log, 1)
        root.addWidget(box_dec, 2)

    def _add_row(self, key, name, range_text):
        row = self.table.rowCount()
        self.table.insertRow(row)
        for col, text in enumerate((name, range_text, '—', '无数据')):
            item = QTableWidgetItem(text)
            item.setTextAlignment(Qt.AlignCenter if col else Qt.AlignLeft | Qt.AlignVCenter)
            self.table.setItem(row, col, item)
        self._rows[key] = row

    def _set_row(self, key, dist_text, state_text, color, dist=None):
        row = self._rows.get(key)
        if row is None:
            return
        self.table.item(row, 2).setText(dist_text)
        st = self.table.item(row, 3)
        st.setText(state_text)
        st.setForeground(color)

    # ================= 车型自适应 =================
    def set_active_sensors(self, robot_type):
        """只显示本车型实际配备的传感器，未配备的行隐藏。

        默认车型 dt01 只有单雷达 + IMU，若把双 RGB-D 和 6 路超声全都列出来，
        这些行会永远停在"无数据"，面板看起来就是死的。
        """
        self.robot_type = robot_type
        active = set(ROBOT_SENSORS.get(robot_type, ROBOT_SENSORS['dt01']))
        for key, row in self._rows.items():
            self.table.setRowHidden(row, key not in active)
        # 未配备的传感器不再参与离线判定
        for key in list(self._last_update):
            if key not in active:
                del self._last_update[key]
        self.radar.set_active(active)
        n = len(active)
        self.lbl_scope.setText(
            f'车型 {robot_type.upper()}：共 {n} 路传感器 | '
            f'雷达 0.32~30.0 m（显示 6 m）| RGB-D 0.05~10.0 m | 超声 0.03~4.0 m'
            if robot_type == 'dt01d' else
            f'车型 {robot_type.upper()}：共 {n} 路传感器 | '
            f'雷达 0.25~30.0 m（显示 6 m）| 无 RGB-D / 无超声'
        )

    # ================= 弹出 / 停靠 =================
    def _on_popout_clicked(self):
        """请求主窗口把自己弹出成独立窗口，或放回标签页。"""
        self.popout_requested.emit(not self.floating)

    def set_floating(self, floating):
        """主窗口完成 reparent 后回调，同步按钮文案。"""
        self.floating = floating
        self.btn_popout.setText('⤡ 放回右侧面板' if floating
                                else '⧉ 弹出为独立窗口')

    def _check_offline(self):
        """超过 DATA_TIMEOUT 没收到数据 → 标记离线（话题未发布）。"""
        now = time.monotonic()
        active = set(ROBOT_SENSORS.get(self.robot_type, ROBOT_SENSORS['dt01']))
        # 相机点云订阅已禁用，不参与"离线"判定（保持"未接入"标注）
        active.difference_update(CAMERA_ORDER)
        for key in active:
            last = self._last_update.get(key)
            if last is not None and now - last <= DATA_TIMEOUT:
                continue
            # 从未收到过数据，或已超时
            st = self.table.item(self._rows[key], 3)
            txt = st.text()
            if txt.startswith('离线'):
                continue
            if last is None:
                self._set_row(key, '—', '离线（无话题）', OFFLINE_COLOR)
            else:
                self._set_row(key, '—', '离线（数据中断）', OFFLINE_COLOR)

    # ================= 接收诊断 =================
    def _record_rx(self, key):
        """记录一次接收时间戳，用于统计实时频率。"""
        now = time.monotonic()
        arr = self._rx_times.setdefault(key, [])
        arr.append(now)
        # 仅保留最近 2 秒内的时间戳
        cutoff = now - 2.0
        self._rx_times[key] = [t for t in arr if t >= cutoff]

    def _refresh_diag(self):
        """1Hz 刷新诊断行：每个传感器最近 2 秒内平均 Hz + 最后一次接收延迟。

        一眼就能看出「面板没显示」到底是「话题没数据」还是「数据已到但显示坏了」。
        """
        now = time.monotonic()
        active = set(ROBOT_SENSORS.get(self.robot_type, ROBOT_SENSORS['dt01']))
        names = {
            'scan': '/scan', 'imu': '/imu/data',
            'camera_l': '/camera_l_sensor/points',
            'camera_r': '/camera_r_sensor/points',
        }
        for n in SONAR_ORDER:
            names[n] = '/' + n
        parts = []
        for key in sorted(active, key=lambda k: (
                0 if k == 'scan' else 1 if k.startswith('camera') else
                2 if k.startswith('sonar') else 3, k)):
            arr = self._rx_times.get(key, [])
            hz_str = f'{len(arr) / 2.0:.1f}Hz' if arr else '0.0Hz'
            last = self._last_update.get(key)
            if last is None:
                age = '从未'
            else:
                age = f'{now - last:.1f}s'
            parts.append(f'{names.get(key, key)} {hz_str} ({age})')
        self.lbl_diag.setText('诊断：' + '  |  '.join(parts))

# ================= 外部数据入口 =================
    def update_scan(self, msg):
        self.radar.set_scan(msg)
        now = time.monotonic()
        self._last_update['scan'] = now
        self._record_rx('scan')
        # 首次接收到 /scan 时打印一次日志，便于排查信号链路
        if not self._first_seen.get('scan'):
            self._first_seen['scan'] = True
            print('[传感器面板] 首次收到 /scan 数据', flush=True)
        # 雷达行：统计有效回波数与最近距离
        if msg is None:
            self._set_row('scan', '—', '无数据', QColor(140, 152, 168))
            return
        r = np.asarray(msg.ranges, dtype=np.float64)
        valid = np.isfinite(r)
        valid &= (r >= max(msg.range_min, 0.0))
        valid &= (r < min(msg.range_max, self.radar.display_range))
        n = int(np.count_nonzero(valid))
        if n == 0:
            self._set_row('scan', '—', '空旷（无回波）', SAFE_COLOR)
        else:
            dmin = float(np.min(r[valid]))
            color, level = level_color(dmin)
            self._set_row('scan', f'{dmin:.2f} m',
                          f'{level}（{n} 点）', color, dmin)

    def update_sonar(self, topic, state):
        key = topic.lstrip('/')
        self.radar.set_sonar(topic, state)
        self._last_update[key] = time.monotonic()
        self._record_rx(key)
        if not self._first_seen.get(key):
            self._first_seen[key] = True
            print(f'[传感器面板] 首次收到 {topic} 数据 active={state.get("active")}',
                  flush=True)
        row = self._rows.get(key)
        if row is None:
            return
        if not state.get('active'):
            self._set_row(key, '—', '射空', SAFE_COLOR)
            return
        d = state.get('obstacle_dist', float('inf'))
        color, level = level_color(d)
        self._set_row(key, f'{d:.2f} m', level, color, d)

    def update_camera(self, topic, state):
        # topic 为逻辑键 camera_l / camera_r（桥接层已把实际话题
        # /camera_l_sensor/points 归一化），strip/split 兼容历史 /camera_l/points 写法
        key = topic.strip('/').split('/')[0]
        self.radar.set_camera(key, state)
        self._last_update[key] = time.monotonic()
        self._record_rx(key)
        if not self._first_seen.get(key):
            self._first_seen[key] = True
            print(f'[传感器面板] 首次收到 {topic} 数据 active={state.get("active")}',
                  flush=True)
        row = self._rows.get(key)
        if row is None:
            return
        if not state.get('active'):
            self._set_row(key, '—', '无点云', QColor(140, 152, 168))
            return
        d = state.get('min_dist', float('inf'))
        if not np.isfinite(d):
            self._set_row(key, '—', '视野内无障碍', SAFE_COLOR)
            return
        color, level = level_color(d)
        self._set_row(key, f'{d:.2f} m', level, color, d)

    def update_pose(self, pose):
        self.radar.set_pose(pose)

    def update_imu(self, angular_z, linear_x):
        """IMU 行：不测距，显示偏航角速度与纵向加速度。"""
        self._last_update['imu'] = time.monotonic()
        self._record_rx('imu')
        self._set_row('imu', '—',
                      f'ωz {angular_z:+.2f}°/s  ax {linear_x:+.2f}',
                      SENSOR_SPECS['imu']['color'])

    def set_nav_decision(self, text):
        self.lbl_nav.setText('导航决策：' + text)

    def set_avoid_decision(self, text):
        self.lbl_avoid.setText('避障决策：' + text)
        if '急停' in text:
            self.lbl_avoid.setStyleSheet(
                'color: #ff5d5d; font-family: monospace; font-weight: bold;')
        elif '减速' in text or '绕行' in text:
            self.lbl_avoid.setStyleSheet(
                'color: #ffb454; font-family: monospace;')
        else:
            self.lbl_avoid.setStyleSheet(
                'color: #6ee7a0; font-family: monospace;')

    def set_stop_reason(self, text):
        self.lbl_stop.setText('停车原因：' + text)
        if '发现障碍' in text:
            self.lbl_stop.setStyleSheet(
                'color: #ff5d5d; font-family: monospace; font-weight: bold;')
        elif text.startswith('停车'):
            self.lbl_stop.setStyleSheet(
                'color: #ffb454; font-family: monospace;')
        else:
            self.lbl_stop.setStyleSheet(
                'color: #6ee7a0; font-family: monospace;')

    def set_next_action(self, text):
        self.lbl_next.setText('下一步：' + text)
        self.lbl_next.setStyleSheet('color: #c7a2ff; font-family: monospace;')

    def log_decision(self, tag, text):
        self.decision_log.appendPlainText(f'[{tag}] {text}')
