# -*- coding: utf-8 -*-
"""导航 / 巡线参数在线调参面板（DJ-NAV 1.0）。

现场调参最常用的入口：把 route_follower（常规 / 避障）与 Nav2（运动控制 /
代价地图 / 定位）里最容易现场调、且运行时支持动态改的参数，按【用途】
分成 常规 / 避障 / 导航 三类集中到这里。点某一行右侧「设」即通过 SSH 在
工控机上执行一次 `ros2 param set`，无需重启导航栈（Nav2 多数参数为动态
参数，set 后即时生效）。

⚠ 面板改的是**运行时**参数：重启导航栈 / 巡线节点后会回到 yaml 或
route_follower_node.py 里 declare 的默认值，要长期生效需同步改源文件。

参数默认值与以下文件严格对齐（改动时需同步）：
  gui/route_follower_node.py                        —— 巡线 / 避障节点 declare 的默认值
  robot_ws/src/dt01_bringup/config/nav2_params_real.yaml —— Nav2 各节点默认值

每个参数"是什么意思、什么现象该调它"见 docs/调参手册.md。
"""
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QGroupBox, QPushButton, QLabel, QDoubleSpinBox)

# 暗色主题下 QDoubleSpinBox 没有全局样式，这里补一份局部 QSS 保证可读。
SPIN_QSS = """
QDoubleSpinBox { background: #1c1f25; border: 1px solid #3a3f4a;
                 border-radius: 4px; padding: 3px 4px; color: #e6edf3; }
QDoubleSpinBox:focus { border-color: #5aa0e8; }
"""

# =====================================================================
# 参数组定义：按【用途】分类（常规 / 避障 / 导航）
#   group:
#     title  标题（前缀即分类：常规 / 避障 / 导航）
#     node   ros2 param set 的目标节点（带前导斜杠的完整名）
#     desc   该组用途说明（显示在组标题下方）
#   params: 每项 (参数名, 显示名, 默认值, 最小值, 最大值, 步长, 小数位, 单位, 释义)
# =====================================================================
PARAM_GROUPS = [
    {
        'title': '常规 · 巡线控制',
        'node': '/route_follower',
        'desc': '巡线基础：速度上限、追踪/对正增益、前视距离、到点判定。与避障无关。',
        'params': [
            ('v_max', '最大线速度', 0.35, 0.05, 0.50, 0.05, 2, ' m/s',
             '直线行驶速度上限。超过 0.5 易引发定位跳变 / 车身抽搐。'),
            ('w_max', '最大角速度', 0.80, 0.10, 1.50, 0.05, 2, ' rad/s',
             '转向角速度上限，原地对正与直线纠偏都受它限制。'),
            ('kp_rot', '对正 P 增益', 1.60, 0.10, 5.00, 0.10, 2, '',
             '出发前原地转向对正下一点的比例增益。太大→转过头来回摆；太小→半天对不正。'),
            ('kp_lat', '追踪 P 增益', 2.20, 0.10, 5.00, 0.10, 2, '',
             '直线行驶贴合连线的比例增益。太大→蛇形摆动；太小→跑偏拉不回来。'),
            ('min_w', '最小角速度', 0.12, 0.00, 0.50, 0.01, 2, ' rad/s',
             '克服底盘静摩擦的角速度下限。原地转不动调大；转向抽搐调小。'),
            ('look_ahead', '前视距离', 0.50, 0.10, 2.00, 0.05, 2, ' m',
             '前视点追踪的预瞄距离。太小→贴线太紧易抖；太大→切弯提前、走内圈。'),
            ('arrive_tol', '到点半径', 0.15, 0.05, 0.50, 0.01, 2, ' m',
             '判定到达某个打点的距离。太小→在点上反复微调；太大→提前算到达。'),
            ('goal_timeout', '单点超时', 90.0, 5.00, 300.0, 5.00, 0, ' s',
             '单个打点的最长执行时间，超时自动跳过该点继续后续点。'),
        ],
    },
    {
        'title': '避障 · 分层绕障',
        'node': '/route_follower',
        'desc': '预警(1.5m)小角度提前变道 → 近距(1.0m)大幅转向边转边走 → 紧急(0.22m)蠕行绕行。',
        'params': [
            ('avoid_far', '提前绕障距离', 1.50, 0.50, 3.00, 0.10, 2, ' m',
             '【预警区】前方障碍小于此距离就开始小角度提前变道。调大→更早开始绕'
             '（窄走廊会一直偏离路线）；调小→退回"到眼前才躲"。'),
            ('avoid_near', '近距转向距离', 1.00, 0.30, 2.00, 0.10, 2, ' m',
             '【近距区】前方障碍小于此距离切换为大幅转向 + 保持前进，即"提前 1m 大幅转向"的阈值。'),
            ('stop_dist', '紧急蠕行边界', 0.22, 0.10, 0.60, 0.01, 2, ' m',
             '【紧急区】前方障碍小于此距离切换为蠕行绕行。调大→更早进入低速，更安全。'),
            ('avoid_w', '避障角速度', 0.80, 0.20, 1.50, 0.05, 2, ' rad/s',
             '避障转向的角速度上限。绕不开就调大；车身摆动太大就调小。'),
            ('v_turn_min', '近距最低速度', 0.15, 0.05, 0.30, 0.01, 2, ' m/s',
             '近距转向时的最低前进速度，保证"边转边走"。设 0 会退化成原地干转，绕不开静态障碍。'),
            ('v_creep', '紧急蠕行速度', 0.06, 0.00, 0.15, 0.01, 2, ' m/s',
             '紧急区的前进速度。设 0 = 停车等待（遇人挡路更安全）；>0 = 继续蹭着绕。'),
            ('avoid_off_max', '前视偏移上限', 0.80, 0.20, 1.50, 0.05, 2, ' m',
             '绕障时前视点横向偏移的最大值。绕大障碍 / 想切更大弧线就调大。'),
            ('recover_rate', '回线衰减速率', 0.98, 0.90, 1.00, 0.01, 2, '',
             '障碍解除后偏移回落的速度。越接近 1 回线越慢（绕完慢慢回去）；'
             '调小(0.92)会猛拉回线，可能二次贴障。'),
        ],
    },
    {
        'title': '导航 · 运动控制',
        'node': '/controller_server',
        'desc': 'Nav2 自由规划时的速度/加速度与到达判定（巡线模式不受此类影响）。',
        'params': [
            ('FollowPath.max_vel_x', '最大前进速度', 0.55, 0.10, 1.00, 0.05, 2, ' m/s',
             'Nav2 规划执行的速度上限。一次加 0.1，跑一段确认定位不飘再加。'),
            ('FollowPath.max_vel_theta', '最大角速度', 1.00, 0.20, 2.00, 0.05, 2, ' rad/s',
             'Nav2 转向角速度上限。'),
            ('FollowPath.acc_lim_x', '加速度上限', 0.80, 0.20, 2.00, 0.10, 2, ' m/s²',
             '前进加速度上限。起步/转向打滑（定位跳变的源头）就调小。'),
            ('general_goal_checker.xy_goal_tolerance', '到点容差', 0.30, 0.10, 1.00, 0.05, 2, ' m',
             'Nav2 判定到达目标点的距离。太小→膨胀区把车挡在圈外永远判不到到达。'),
        ],
    },
    {
        'title': '导航 · 全局代价地图',
        'node': '/global_costmap',
        'desc': '全局路径规划用的膨胀层：决定规划器敢不敢贴障碍。',
        'params': [
            ('inflation_layer.inflation_radius', '膨胀半径', 0.40, 0.10, 1.00, 0.05, 2, ' m',
             '把障碍"加厚"的距离。窄通道规划不出路就调小；贴墙太近就调大。'),
            ('inflation_layer.cost_scaling_factor', '代价衰减', 6.00, 1.00, 20.0, 0.50, 2, '',
             '膨胀区代价衰减快慢。越大越敢贴墙，越小越保守。'),
        ],
    },
    {
        'title': '导航 · 局部代价地图',
        'node': '/local_costmap',
        'desc': '实时避障用的滚动窗口膨胀层，影响 Nav2 绕障时的贴障程度。',
        'params': [
            ('inflation_layer.inflation_radius', '膨胀半径', 0.40, 0.10, 1.00, 0.05, 2, ' m',
             '实时避障的贴障余量。绕障时蹭墙就调大；窄处过不去就调小。'),
            ('inflation_layer.cost_scaling_factor', '代价衰减', 6.00, 1.00, 20.0, 0.50, 2, '',
             '局部膨胀区代价衰减。越大越敢贴墙。'),
        ],
    },
    {
        'title': '导航 · 定位',
        'node': '/amcl',
        'desc': 'AMCL 粒子滤波的更新节奏与激光采样，影响定位是否跟手。',
        'params': [
            ('update_min_d', '位移更新阈值', 0.05, 0.01, 0.50, 0.01, 2, ' m',
             '走够这个距离才做一次滤波更新。太大→粒子滞后于真实位姿→位姿跳变。'),
            ('update_min_a', '转角更新阈值', 0.05, 0.01, 0.50, 0.01, 2, ' rad',
             '转够这个角度才做一次滤波更新。太大→原地转圈时 AMCL 不更新→丢定位。'),
            ('max_beams', '激光束数', 90.0, 30.0, 300.0, 10.0, 0, '',
             '每帧参与打分的激光束数。定位不稳可调大；CPU 吃紧可调小。'),
        ],
    },
]


class TuningPanel(QWidget):
    """调参面板：按用途分组列出可调参数，每行一个数值框 + 「设」按钮。"""

    param_set_requested = pyqtSignal(str, str, float)   # (node, name, value)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._spins = {}   # (node, name) -> QDoubleSpinBox
        self._build_ui()

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(8)

        # 顶部：说明 + 恢复默认
        bar = QHBoxLayout()
        tip = QLabel('现场调参：改数值后点该行「设」，经 SSH 下发到工控机即时生效\n'
                     '（仅运行时有效，重启回到默认值；固化需改源文件）')
        tip.setWordWrap(True)
        tip.setStyleSheet('color: #8b9bb4; font-size: 11px;')
        bar.addWidget(tip, 1)
        btn_reset = QPushButton('恢复默认')
        btn_reset.setToolTip('把面板所有数值还原为默认值（仅改界面，不下发）')
        btn_reset.clicked.connect(self._reset_defaults)
        bar.addWidget(btn_reset)
        root.addLayout(bar)

        for group in PARAM_GROUPS:
            root.addWidget(self._build_group(group))
        root.addStretch(1)

    def _build_group(self, group):
        node = group['node']
        box = QGroupBox(group['title'])
        box.setStyleSheet(SPIN_QSS)
        v = QVBoxLayout(box)
        v.setSpacing(4)
        if group.get('desc'):
            lbl_desc = QLabel(group['desc'])
            lbl_desc.setWordWrap(True)
            lbl_desc.setStyleSheet('color: #8b9bb4; font-size: 11px;')
            v.addWidget(lbl_desc)

        g = QGridLayout()
        g.setHorizontalSpacing(6)
        g.setVerticalSpacing(3)
        for r, spec in enumerate(group['params']):
            name, label, default, vmin, vmax, step, decimals, unit = spec[:8]
            doc = spec[8] if len(spec) > 8 else ''

            lbl = QLabel(label)
            lbl.setToolTip(f'{node} / {name}\n{doc}' if doc else f'{node} / {name}')
            g.addWidget(lbl, r, 0)

            spin = QDoubleSpinBox()
            spin.setRange(vmin, vmax)
            spin.setSingleStep(step)
            spin.setDecimals(decimals)
            spin.setValue(default)
            if unit:
                spin.setSuffix(unit)
            spin.setMinimumWidth(90)
            if doc:
                spin.setToolTip(doc)
            g.addWidget(spin, r, 1)

            btn = QPushButton('设')
            btn.setToolTip(f'下发 {node} {name}')
            btn.clicked.connect(lambda _c, n=node, p=name, s=spin: self._apply(n, p, s))
            btn.setFixedWidth(36)
            g.addWidget(btn, r, 2)

            self._spins[(node, name)] = spin

        g.setColumnStretch(1, 1)
        v.addLayout(g)
        return box

    def _apply(self, node, name, spin):
        self.param_set_requested.emit(node, name, float(spin.value()))

    def _reset_defaults(self):
        for group in PARAM_GROUPS:
            for spec in group['params']:
                self._spins[(group['node'], spec[0])].setValue(spec[2])
