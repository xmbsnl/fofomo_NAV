"""地图画布：显示地图/机器人/激光/路径/点位，支持缩放、平移、点选设位姿与目标。"""
import math
import os
import re

import numpy as np
from PyQt5.QtCore import Qt, QPointF, QRectF, pyqtSignal
from PyQt5.QtGui import (QColor, QImage, QPainter, QPainterPath, QPen,
                         QPixmap, QTransform, QBrush, QPolygonF, QFont)
from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene, QGraphicsItem


class OverlayItem(QGraphicsItem):
    """动态叠加层：激光点 + 规划路径（世界坐标，随地图缩放）。

    机器人 / 初始位姿 / 当前目标 / 拖拽预览 这些**标记类**图元不在这里画 ——
    它们需要屏幕恒定大小（否则缩放后糊成一团），统一走下方
    FlagMarkItem / RobotMarkItem（与队列打点同款小旗帜样式，2026-09-28 改）。
    """

    def __init__(self):
        super().__init__()
        self.setZValue(10)
        self.scan_points = []       # [(x, y), ...] 世界坐标
        self.route_points = []      # 巡线路线（打点连线）：[(x, y), ...]
        self.path_points = []       # [(x, y), ...]
        self.keepout_zones = []     # 电子围栏禁区：[[(x,y),...], ...] 多边形列表
        self.keepout_draft = []     # 正在绘制的禁区草稿顶点 [(x,y), ...]

    def boundingRect(self):
        return QRectF(-1e4, -1e4, 2e4, 2e4)

    def _px(self, painter, size=3.0):
        """返回 size 屏幕像素对应的场景单位长度。"""
        scale = abs(painter.worldTransform().m11())
        return size / max(scale, 1e-6)

    @staticmethod
    def _safe_pen(color, width, style=Qt.SolidLine):
        """构造画笔，并把宽度钳制到 Qt 可接受的范围（缩放极小时笔宽
        会换算成 0/负数，Qt 会刷屏告警）。统一钳到 [0.5, 100]。"""
        try:
            w = float(width)
        except (TypeError, ValueError):
            w = 1.0
        if w != w or w in (float('inf'), float('-inf')):   # NaN / inf
            w = 1.0
        w = max(0.5, min(w, 100.0))
        return QPen(color, w, style)

    def paint(self, painter, option, widget=None):
        px = self._px(painter)

        # ---- 巡线路线（青色：打点之间的最短直线，机器人严格沿线行驶）----
        # 09-29 调细：1.4 → 1.0（用户反馈线条偏粗，淹没点位标记）
        if len(self.route_points) >= 2:
            pen = self._safe_pen(QColor(0, 190, 255), px * 1.0)
            pen.setCapStyle(Qt.RoundCap)
            pen.setJoinStyle(Qt.RoundJoin)
            painter.setPen(pen)
            route = QPainterPath(QPointF(*self.route_points[0]))
            for p in self.route_points[1:]:
                route.lineTo(QPointF(*p))
            painter.drawPath(route)

        # ---- 电子围栏禁区：淡红底 + 点阵纹理（明确标记为禁入区）+ 细边框 ----
        # 09-29：边框再细一档（0.9 → 0.55），内部加 Dense6 纹理强化"禁入"语义
        for poly in self.keepout_zones:
            if len(poly) < 3:
                continue
            qpoly = QPolygonF([QPointF(*p) for p in poly])
            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(QColor(255, 60, 60, 38)))
            painter.drawPolygon(qpoly)
            painter.setBrush(QBrush(QColor(255, 60, 60, 95), Qt.Dense6Pattern))
            painter.drawPolygon(qpoly)
            painter.setBrush(Qt.NoBrush)
            painter.setPen(self._safe_pen(QColor(255, 70, 70), px * 0.55))
            painter.drawPolygon(qpoly)

        # ---- 正在绘制的禁区草稿（红色虚线 + 顶点，与正式禁区同档细线）----
        if self.keepout_draft:
            painter.setBrush(Qt.NoBrush)
            if len(self.keepout_draft) >= 2:
                painter.setPen(self._safe_pen(QColor(255, 130, 130), px * 0.7,
                                              Qt.DashLine))
                draft = QPainterPath(QPointF(*self.keepout_draft[0]))
                for p in self.keepout_draft[1:]:
                    draft.lineTo(QPointF(*p))
                painter.drawPath(draft)
            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(QColor(255, 130, 130)))
            for p in self.keepout_draft:
                painter.drawEllipse(QPointF(*p), px * 2.0, px * 2.0)

        # ---- 规划路径（亮绿细线，09-29 调细：0.6 → 0.5）----
        if len(self.path_points) >= 2:
            pen = self._safe_pen(QColor(60, 220, 100), px * 0.5)
            pen.setCapStyle(Qt.RoundCap)
            pen.setJoinStyle(Qt.RoundJoin)
            painter.setPen(pen)
            path = QPainterPath(QPointF(*self.path_points[0]))
            for p in self.path_points[1:]:
                path.lineTo(QPointF(*p))
            painter.drawPath(path)

        # ---- 激光点 ----
        if self.scan_points:
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor(255, 80, 80, 160))
            d = px * 1.5
            for x, y in self.scan_points:
                painter.drawEllipse(QPointF(x, y), d, d)


class FlagMarkItem(QGraphicsItem):
    """初始位姿 / 当前目标 / 拖拽预览标记：与队列打点同款小旗帜。

    ItemIgnoresTransformations = 屏幕像素恒定大小，任何缩放级别下都
    清晰可辨（这正是队列打点旗帜的做法）。颜色区分用途：
      蓝=初始位姿  绿=当前目标  黄=拖拽预览
    """

    def __init__(self, name, color):
        super().__init__()
        self.setZValue(25)
        self.setFlag(QGraphicsItem.ItemIgnoresTransformations)
        self.name = name
        self.color = color
        self.yaw = 0.0

    def boundingRect(self):
        return QRectF(-16, -40, 100, 56)

    def paint(self, painter, option, widget=None):
        # 旗杆
        painter.setPen(QPen(self.color, 2.5))
        painter.drawLine(QPointF(0, 0), QPointF(0, -26))
        # 旗面
        painter.setBrush(QBrush(self.color))
        painter.setPen(QPen(QColor(255, 255, 255), 1))
        painter.drawPolygon(QPolygonF([
            QPointF(0, -26), QPointF(17, -21), QPointF(0, -16)]))
        # 底盘圆点（白描边，缩小时也醒目）
        painter.setBrush(QBrush(QColor(self.color.red(), self.color.green(),
                                       self.color.blue(), 220)))
        painter.setPen(QPen(QColor(255, 255, 255), 1.5))
        painter.drawEllipse(QPointF(0, 0), 6, 6)
        # 朝向线 + 小箭头（注意：item 坐标 y 向下，所以 y 取 -sin）
        painter.setPen(QPen(self.color, 2.5))
        end = QPointF(12 * math.cos(self.yaw), -12 * math.sin(self.yaw))
        painter.drawLine(QPointF(0, 0), end)
        a1 = self.yaw + math.radians(150)
        a2 = self.yaw - math.radians(150)
        painter.drawLine(end, end + QPointF(4 * math.cos(a1), -4 * math.sin(a1)))
        painter.drawLine(end, end + QPointF(4 * math.cos(a2), -4 * math.sin(a2)))
        # 名称
        if self.name:
            painter.setPen(self.color)
            font = QFont()
            font.setPointSize(8)
            font.setBold(True)
            painter.setFont(font)
            painter.drawText(QPointF(10, -4), self.name)


class PoseArrowItem(QGraphicsItem):
    """初始位姿 / 拖拽预览标记：单个大箭头（屏幕恒定大小）。

    2026-09-29 用户反馈：旧旗帜标记元素多（旗面+底盘圆点+小箭头）且小，
    拖拽定位时看不清、对齐初始位置操作成本高 —— 改为单个粗箭头+白描边，
    箭头中心即位姿点，任何缩放级别下清晰可辨。
    蓝=初始位姿  黄=拖拽预览
    """

    def __init__(self, color, name=''):
        super().__init__()
        self.setZValue(25)
        self.setFlag(QGraphicsItem.ItemIgnoresTransformations)
        self.color = color
        self.name = name
        self.yaw = 0.0

    def boundingRect(self):
        return QRectF(-27, -27, 62, 62)

    def paint(self, painter, option, widget=None):
        painter.save()
        # item 坐标 y 向下：世界 yaw 逆时针为正 → 画布需顺时针旋转（取负）
        painter.rotate(-math.degrees(self.yaw))
        # 单个大箭头（朝 +x，中心=位姿点）：总长 34px、头宽 22px，
        # 约为旧朝向线（12px）的 3 倍；纯箭头一个图元，无圆点/旗面混杂
        L, HW, SW = 34, 11, 5        # 总长 / 头半宽 / 杆半宽
        arrow = QPolygonF([
            QPointF(L, 0),
            QPointF(L - 15, -HW),
            QPointF(L - 15, -SW),
            QPointF(-L + 8, -SW),
            QPointF(-L + 8, SW),
            QPointF(L - 15, SW),
            QPointF(L - 15, HW),
        ])
        # 白描边底：深灰/浅白地图背景上都醒目
        painter.setPen(QPen(QColor(255, 255, 255), 3))
        painter.setBrush(QBrush(self.color))
        painter.drawPolygon(arrow)
        painter.restore()
        if self.name:
            painter.setPen(self.color)
            font = QFont()
            font.setPointSize(8)
            font.setBold(True)
            painter.setFont(font)
            painter.drawText(QPointF(-8, 26), self.name)


class RobotMarkItem(QGraphicsItem):
    """机器人：蓝色圆 + 朝向楔形（屏幕恒定大小，与旗帜标记同族小巧风格）。

    世界坐标半径的旧画法在缩放后要么糊成一团、要么大得遮挡激光点
    （2026-09-28 用户反馈），改为与队列标记一致的屏幕像素尺寸。
    """

    def __init__(self):
        super().__init__()
        self.setZValue(28)
        self.setFlag(QGraphicsItem.ItemIgnoresTransformations)
        self.yaw = 0.0

    def boundingRect(self):
        return QRectF(-24, -24, 48, 48)

    def paint(self, painter, option, widget=None):
        # 蓝色在白色地图上对比度低，先画一圈白描边底圆保证任何背景可见；
        # 09-28 首版 9px 太小，建图时被密集红色雷达点淹没（用户反馈"不见了"）。
        c = QColor(50, 180, 255)
        painter.setPen(QPen(QColor(255, 255, 255), 2.5))
        painter.setBrush(QBrush(QColor(50, 180, 255, 160)))
        painter.drawEllipse(QPointF(0, 0), 13, 13)
        # 朝向楔形（item 坐标 y 向下，y 取 -sin），白描边 + 实心蓝
        p1 = QPointF(13 * math.cos(self.yaw - 0.5), -13 * math.sin(self.yaw - 0.5))
        p2 = QPointF(21 * math.cos(self.yaw), -21 * math.sin(self.yaw))
        p3 = QPointF(13 * math.cos(self.yaw + 0.5), -13 * math.sin(self.yaw + 0.5))
        painter.setBrush(QBrush(QColor(50, 180, 255, 240)))
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.drawPolygon(QPolygonF([p1, p2, p3]))


class QueuedGoalItem(QGraphicsItem):
    """任务队列目标点：大红旗帜 + 编号，加粗高亮显示。"""

    def __init__(self, name, idx, x, y, yaw=0.0):
        super().__init__()
        self.setPos(x, y)
        self.setZValue(30)
        self.setFlag(QGraphicsItem.ItemIgnoresTransformations)
        self.name = name
        self.idx = idx            # 队列中的序号（0-based）
        self.yaw = yaw

    def boundingRect(self):
        return QRectF(-10, -50, 120, 60)

    def paint(self, painter, option, widget=None):
        # 旗杆（加粗）
        painter.setPen(QPen(QColor(255, 60, 60), 4))
        painter.drawLine(QPointF(0, 0), QPointF(0, -38))
        # 旗面（大红，更大）
        painter.setBrush(QBrush(QColor(255, 60, 60)))
        painter.setPen(QPen(QColor(255, 255, 255), 1.5))
        painter.drawPolygon(QPolygonF([
            QPointF(0, -38), QPointF(26, -31), QPointF(0, -24)]))
        # 底盘（大圆点 + 白描边）
        painter.setBrush(QBrush(QColor(255, 60, 60, 220)))
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.drawEllipse(QPointF(0, 0), 8, 8)
        # 序号（白色大号，居中在旗面下方）
        painter.setPen(QColor(255, 255, 255))
        font = QFont()
        font.setPointSize(11)
        font.setBold(True)
        painter.setFont(font)
        painter.drawText(QPointF(-3, -20), str(self.idx + 1))
        # 名称（旗杆右侧，大号加粗）
        painter.setPen(QColor(255, 200, 200))
        font.setPointSize(9)
        painter.setFont(font)
        painter.drawText(QPointF(12, -4), self.name)
        # 朝向（从圆点延伸的方向线）
        painter.setPen(QPen(QColor(255, 60, 60), 3))
        end = QPointF(14 * math.cos(self.yaw), -14 * math.sin(self.yaw))
        painter.drawLine(QPointF(0, 0), end)
        # 小箭头
        a1 = self.yaw + math.radians(150)
        a2 = self.yaw - math.radians(150)
        painter.drawLine(end, end + QPointF(5 * math.cos(a1), -5 * math.sin(a1)))
        painter.drawLine(end, end + QPointF(5 * math.cos(a2), -5 * math.sin(a2)))


class WaypointItem(QGraphicsItem):
    """已保存点位：旗帜样式（颜色可区分，默认橙色）。"""

    def __init__(self, name, x, y, yaw=0.0, color=QColor(255, 170, 40)):
        super().__init__()
        self.setPos(x, y)
        self.setZValue(20)
        self.setFlag(QGraphicsItem.ItemIgnoresTransformations)
        self.name = name
        self.yaw = yaw
        self.color = color

    def boundingRect(self):
        return QRectF(-4, -30, 80, 36)

    def paint(self, painter, option, widget=None):
        # 旗杆
        painter.setPen(QPen(self.color, 2))
        painter.drawLine(QPointF(0, 0), QPointF(0, -22))
        # 旗面
        painter.setBrush(QBrush(self.color))
        painter.setPen(Qt.NoPen)
        painter.drawPolygon(QPolygonF([
            QPointF(0, -22), QPointF(14, -18), QPointF(0, -14)]))
        # 底盘
        painter.setBrush(QBrush(QColor(self.color.red(), self.color.green(),
                                       self.color.blue(), 180)))
        painter.drawEllipse(QPointF(0, 0), 4, 4)
        # 名称
        painter.setPen(QColor(self.color.red(), min(self.color.green() + 40, 255),
                              self.color.blue() + 90))
        font = QFont()
        font.setPointSize(8)
        painter.setFont(font)
        painter.drawText(QPointF(6, 4), self.name)


class MapView(QGraphicsView):
    """主地图视图。"""

    # 点选完成信号：x, y, yaw（设位姿与设目标共用，由 mode 区分）
    pose_picked = pyqtSignal(float, float, float)
    mouse_moved = pyqtSignal(float, float)
    # 点选在地图范围外被拒绝（2026-09-28：地图外不允许打点/设位姿）
    pick_rejected = pyqtSignal()
    # 点选落在障碍/未知区域被拒绝（2026-09-28：目标不可达会触发
    # "卡住→旋转恢复→定位甩飞"的失控链，从源头拦截）
    pick_blocked = pyqtSignal()
    # 禁区绘制完成：闭合多边形的顶点列表 [(x,y), ...]（世界坐标）
    keepout_drawn = pyqtSignal(list)

    MODE_PAN = 'pan'
    MODE_SET_POSE = 'set_pose'
    MODE_SET_GOAL = 'set_goal'
    MODE_DRAW_KEEPOUT = 'draw_keepout'

    def __init__(self, parent=None):
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)

        self.setRenderHint(QPainter.Antialiasing)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.setBackgroundBrush(QColor(35, 38, 45))
        self.setDragMode(QGraphicsView.ScrollHandDrag)

        self.mode = self.MODE_PAN
        self._press_pos = None

        # 图元
        self.map_item = self._scene.addPixmap(QPixmap())
        self.map_item.setZValue(0)
        self.overlay = OverlayItem()
        self._scene.addItem(self.overlay)
        self._waypoint_items = {}
        self._queue_items = []
        self._keepout_draft = []    # 正在绘制的禁区顶点（世界坐标）
        # 标记类图元（屏幕恒定大小）：先隐藏，有数据才显示
        # 初始位姿/拖拽预览用单大箭头（09-29 识辨识度改进）；
        # 当前导航目标保留旗帜样式以示区分
        self._robot_mark = RobotMarkItem()
        self._init_mark = PoseArrowItem(QColor(80, 140, 255), '初始')
        self._goal_mark = FlagMarkItem('目标', QColor(60, 220, 100))
        self._preview_mark = PoseArrowItem(QColor(255, 210, 60))
        for it in (self._robot_mark, self._init_mark,
                   self._goal_mark, self._preview_mark):
            it.setVisible(False)
            self._scene.addItem(it)
        # 占据栅格缓存（点选空闲校验用）：0=空闲 100=障碍 -1=未知
        self._occ_grid = None
        self._occ_ox = 0.0
        self._occ_oy = 0.0
        self._occ_res = 0.05

        # 世界坐标 y 向上（机器人坐标系）
        self.scale(1, -1)
        self._has_fit = False

    # ================= 模式 =================
    def set_mode(self, mode):
        self.mode = mode
        self._preview_mark.setVisible(False)
        if mode == self.MODE_PAN:
            self.setDragMode(QGraphicsView.ScrollHandDrag)
            self.viewport().unsetCursor()
        elif mode == self.MODE_DRAW_KEEPOUT:
            self.setDragMode(QGraphicsView.NoDrag)
            self.viewport().setCursor(Qt.CrossCursor)
        else:
            self.setDragMode(QGraphicsView.NoDrag)
            self.viewport().setCursor(Qt.CrossCursor)
        # 任何模式切换都丢弃未闭合的禁区草稿。以前只在"进入绘制模式"时清，
        # 中途退出（再点「＋禁区」/切到设位姿/打点）会把已画的顶点留在地图上，
        # 看起来就是一块"清不掉的禁区标志"（2026-09-29 用户反馈）。
        if mode != self.MODE_DRAW_KEEPOUT and self._keepout_draft:
            self._keepout_draft = []
            self.overlay.keepout_draft = []
        self.overlay.update()

    # ================= 地图 =================
    def update_map(self, msg):
        """msg: nav_msgs/OccupancyGrid"""
        info = msg.info
        w, h = info.width, info.height
        if w == 0 or h == 0:
            return
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y

        arr = np.array(msg.data, dtype=np.int16).reshape(h, w)
        # 缓存占据栅格（-1 未知 / 0 空闲 / >0 占据）供点选校验
        self._occ_grid = arr
        self._occ_ox, self._occ_oy, self._occ_res = ox, oy, res
        img = np.full((h, w), 60, dtype=np.uint8)          # unknown 深灰
        img[arr == 0] = 235                                # free 浅
        occ = arr > 50
        img[occ] = 15                                      # 障碍 深黑
        mid = (arr > 0) & (arr <= 50)
        img[mid] = (235 - arr[mid].astype(np.float64) * 4.4).clip(15, 235).astype(np.uint8)

        img = np.ascontiguousarray(np.flipud(img))
        qimg = QImage(img.data, w, h, w, QImage.Format_Grayscale8).copy()
        self.map_item.setPixmap(QPixmap.fromImage(qimg))

        t = QTransform()
        t.translate(ox, oy)
        t.scale(res, -res)
        t.translate(0, -h)
        self.map_item.setTransform(t)

        if not self._has_fit:
            self.fit_map()
            self._has_fit = True

    def reset_fit(self):
        """下次地图更新时重新自动全图。"""
        self._has_fit = False

    # ================= 磁盘地图加载（多地图浏览）=================
    def load_map_file(self, yaml_path):
        """从磁盘加载 nav2 格式地图（map.yaml + map.pgm）并显示。

        返回 (width, height, resolution) 或 None。
        不依赖 ROS，用于浏览/切换已保存的多张地图。
        """
        try:
            info = _parse_map_yaml(yaml_path)
            pgm_path = os.path.join(os.path.dirname(yaml_path),
                                    info['image'])
            arr = _load_pgm(pgm_path)
        except Exception as e:
            return None

        h, w = arr.shape
        res = info['resolution']
        ox, oy = info['origin'][0], info['origin'][1]

        # negate=1 表示 PGM 里"黑=空闲"，与 nav2 默认相反，需再翻转一次
        gray = (255 - arr) if info.get('negate') else arr

        # gray 是归一化灰度（0=空闲, 255=障碍），必须先换算成 occupancy(0~100)
        # 才能与 yaml 里 0~1 量纲的阈值（0.65 / 0.196 等）比较。
        # 修复前直接拿 0~255 与 0.65 比较，全图像素都被判成障碍 → 整屏全黑。
        # 判定语义与 nav2_map_server/src/map_io.cpp loadMapFromFile() 保持一致：
        #   occupied = norm >= occupied_thresh
        #   free     = norm <= free_thresh
        #   其余中间灰度 = UNKNOWN（trinary 模式）
        occupancy = gray.astype(np.float32) * (100.0 / 255.0)
        # 缓存占据栅格（点选校验用）：0=空闲 100=障碍 -1=未知
        occ_grid = np.full((h, w), -1, dtype=np.int16)
        occ_grid[occupancy <= info['free_thresh'] * 100.0] = 0
        occ_grid[occupancy >= info['occupied_thresh'] * 100.0] = 100
        # PGM 第 0 行是图片顶部（y 最大），而 /map 话题的 OccupancyGrid
        # 第 0 行是 y 最小 —— 统一翻转成 ROS 约定（行 0 = y 最小），
        # 保证 _blocked_at 点选校验与激光精修匹配两条加载路径方向一致。
        # （修复：此前文件加载的地图上，点选校验/禁区判断上下镜像。）
        occ_grid = np.flipud(occ_grid)
        self._occ_grid = occ_grid
        self._occ_ox, self._occ_oy, self._occ_res = ox, oy, res
        img = np.full((h, w), 60, dtype=np.uint8)          # 中间灰度 → 未知 深灰
        img[occupancy >= info['occupied_thresh'] * 100.0] = 15   # 障碍 深黑
        img[occupancy <= info['free_thresh'] * 100.0] = 235      # 自由 浅
        img = np.ascontiguousarray(np.flipud(img))
        qimg = QImage(img.data, w, h, w, QImage.Format_Grayscale8).copy()
        self.map_item.setPixmap(QPixmap.fromImage(qimg))

        t = QTransform()
        t.translate(ox, oy)
        t.scale(res, -res)
        t.translate(0, -h)
        self.map_item.setTransform(t)

        self.fit_map()
        return (w, h, res)

    def fit_map(self):
        """缩放以完整显示地图。"""
        rect = self.map_item.sceneBoundingRect()
        if rect.isEmpty():
            return
        self.resetTransform()
        self.scale(1, -1)
        vs = self.viewport().size()
        s = min(vs.width() / max(rect.width(), 1e-6),
                vs.height() / max(rect.height(), 1e-6)) * 0.92
        self.scale(s, s)
        self.centerOn(rect.center())

    # ================= 叠加层数据 =================
    def set_robot_pose(self, pose):
        if pose is None:
            self._robot_mark.setVisible(False)
            return
        x, y, yaw = pose
        self._robot_mark.setPos(x, y)
        self._robot_mark.yaw = yaw
        self._robot_mark.setVisible(True)
        self._robot_mark.update()

    def set_scan_points(self, points):
        self.overlay.scan_points = points
        self.overlay.update()

    def set_path(self, points):
        self.overlay.path_points = points
        self.overlay.update()

    def set_route(self, points):
        """设置巡线路线（打点连线可视化）：points 为 [(x, y), ...] 世界坐标。

        起点传机器人当前位置即可：已走过的线段自然"消失"，
        地图上始终显示"机器人 → 剩余各点"的待走连线。
        """
        pts = [tuple(p) for p in points]
        if self.overlay.route_points != pts:
            self.overlay.route_points = pts
            self.overlay.update()

    def set_keepout_zones(self, zones):
        """设置电子围栏禁区显示：zones 为 [{'name','points':[[x,y],...]}, ...]。

        无条件赋值 + 重绘：清空（传入空列表）时也必须刷新，
        否则"有→空"的比较在某些时序下会漏掉重绘。
        """
        polys = []
        for z in zones:
            try:
                pts = [(float(p[0]), float(p[1])) for p in z.get('points', [])]
            except Exception:
                continue
            if len(pts) >= 3:
                polys.append(pts)
        self.overlay.keepout_zones = polys
        self.overlay.update()

    def set_goal_arrow(self, arrow):
        """当前导航目标标记：None 隐藏，(x, y, yaw) 显示绿色小旗。"""
        if arrow is None:
            self._goal_mark.setVisible(False)
            return
        x, y, yaw = arrow
        self._goal_mark.setPos(x, y)
        self._goal_mark.yaw = yaw
        self._goal_mark.setVisible(True)
        self._goal_mark.update()

    def set_init_arrow(self, arrow):
        """初始位姿标记：None 隐藏，(x, y, yaw) 显示蓝色小旗。"""
        if arrow is None:
            self._init_mark.setVisible(False)
            return
        x, y, yaw = arrow
        self._init_mark.setPos(x, y)
        self._init_mark.yaw = yaw
        self._init_mark.setVisible(True)
        self._init_mark.update()

    # ================= 点位标记 =================
    def update_waypoints(self, points):
        """已保存点位：list of dict {name, x, y, yaw}，橙色旗帜。"""
        for item in self._waypoint_items.values():
            self._scene.removeItem(item)
        self._waypoint_items.clear()
        for p in points:
            item = WaypointItem(p['name'], p['x'], p['y'], p.get('yaw', 0.0),
                                QColor(255, 170, 40))
            self._scene.addItem(item)
            self._waypoint_items[p['name']] = item

    # ================= 任务队列目标点（大红旗帜 + 编号）=================
    def update_queue(self, queue, active_idx=-1):
        """任务队列可视化：queue 为 [{'name','x','y','yaw'},...]，
        active_idx 为正在执行的点（-1 表示无）。大红旗帜 + 编号。"""
        for item in self._queue_items:
            self._scene.removeItem(item)
        self._queue_items = []
        for i, p in enumerate(queue):
            item = QueuedGoalItem(p['name'], i, p['x'], p['y'],
                                  p.get('yaw', 0.0))
            self._scene.addItem(item)
            self._queue_items.append(item)

    # ================= 鼠标交互 =================
    def wheelEvent(self, event):
        factor = 1.25 if event.angleDelta().y() > 0 else 0.8
        self.scale(factor, factor)

    def mousePressEvent(self, event):
        if self.mode == self.MODE_PAN:
            super().mousePressEvent(event)
            return
        if self.mode == self.MODE_DRAW_KEEPOUT:
            sp = self.mapToScene(event.pos())
            if event.button() == Qt.LeftButton:
                self._keepout_draft.append((sp.x(), sp.y()))
                self.overlay.keepout_draft = self._keepout_draft
                self.overlay.update()
            elif event.button() == Qt.RightButton:
                self._close_keepout_draft()
            event.accept()
            return
        if event.button() == Qt.LeftButton:
            self._press_pos = self.mapToScene(event.pos())
            self._preview_mark.setVisible(False)
        event.accept()

    def mouseMoveEvent(self, event):
        sp = self.mapToScene(event.pos())
        self.mouse_moved.emit(sp.x(), sp.y())
        if self.mode == self.MODE_PAN:
            super().mouseMoveEvent(event)
            return
        if self.mode == self.MODE_DRAW_KEEPOUT:
            event.accept()
            return
        if self._press_pos is not None:
            # 拖拽预览：黄色小旗（与打点同款样式），跟随拖拽向量指示朝向
            yaw = math.atan2(sp.y() - self._press_pos.y(),
                             sp.x() - self._press_pos.x())
            self._preview_mark.setPos(self._press_pos.x(), self._press_pos.y())
            self._preview_mark.yaw = yaw
            self._preview_mark.setVisible(True)
            self._preview_mark.update()
        event.accept()

    def mouseReleaseEvent(self, event):
        if self.mode == self.MODE_PAN:
            super().mouseReleaseEvent(event)
            return
        if self.mode == self.MODE_DRAW_KEEPOUT:
            event.accept()
            return
        if event.button() == Qt.LeftButton and self._press_pos is not None:
            sp = self.mapToScene(event.pos())
            dx = sp.x() - self._press_pos.x()
            dy = sp.y() - self._press_pos.y()
            yaw = math.atan2(dy, dx) if math.hypot(dx, dy) > 0.15 else 0.0
            px, py = self._press_pos.x(), self._press_pos.y()
            self._press_pos = None
            self._preview_mark.setVisible(False)
            # 地图范围外不允许打点/设位姿（2026-09-28 用户要求）：
            # 点在图外 → 发 pick_rejected，由主窗口提示，不产生任何标记。
            rect = self.map_item.sceneBoundingRect()
            if not rect.isEmpty() and not rect.contains(px, py):
                self.pick_rejected.emit()
            elif self._blocked_at(px, py):
                # 障碍/未知区域：目标物理上不可达，放行只会让导航
                # 卡在膨胀区边缘反复恢复（09-28 失控链的源头之一）
                self.pick_blocked.emit()
            else:
                self.pose_picked.emit(px, py, yaw)
        event.accept()

    def _blocked_at(self, x, y):
        """该世界坐标在占据栅格上是否为 障碍/未知（不可达）。"""
        if self._occ_grid is None:
            return False     # 还没加载地图时不拦（边界校验已挡图外）
        gx = int((x - self._occ_ox) / self._occ_res)
        gy = int((y - self._occ_oy) / self._occ_res)
        h, w = self._occ_grid.shape
        if not (0 <= gx < w and 0 <= gy < h):
            return True
        return int(self._occ_grid[gy, gx]) != 0

    def occ_grid_snapshot(self):
        """返回 (占据栅格, 原点x, 原点y, 分辨率)；地图未加载返回 None。

        供初始位姿激光精修（pose_refiner）使用，
        栅格约定与 /map 话题一致：行 0 = y 最小，0=空闲 >50=障碍 -1=未知。
        """
        if self._occ_grid is None:
            return None
        return (self._occ_grid, self._occ_ox, self._occ_oy, self._occ_res)

    def _close_keepout_draft(self):
        """闭合禁区草稿：>=3 点则发出 keepout_drawn，否则丢弃。"""
        if len(self._keepout_draft) >= 3:
            self.keepout_drawn.emit(list(self._keepout_draft))
        self._keepout_draft = []
        self.overlay.keepout_draft = []
        self.overlay.update()


# ================================================== 地图文件解析
def _parse_map_yaml(yaml_path):
    """解析 nav2 map.yaml，返回 {image, resolution, origin, occupied_thresh, free_thresh}。"""
    with open(yaml_path, encoding='utf-8') as f:
        text = f.read()
    info = {
        'image': 'map.pgm',
        'resolution': 0.05,
        'origin': [0.0, 0.0, 0.0],
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
        'negate': 0,
    }
    for key, val in re.findall(r'^(\w+):\s*(.+?)\s*$', text, re.M):
        v = val.strip()
        if key == 'image':
            info['image'] = v.strip('"\'')
        elif key == 'resolution':
            info['resolution'] = float(v)
        elif key == 'origin':
            nums = re.findall(r'-?[\d.]+', v)
            if len(nums) >= 3:
                info['origin'] = [float(n) for n in nums[:3]]
        elif key == 'occupied_thresh':
            info['occupied_thresh'] = float(v)
        elif key == 'free_thresh':
            info['free_thresh'] = float(v)
        elif key == 'negate':
            info['negate'] = int(float(v))
    return info


def _load_pgm(path):
    """读取 PGM 图像（支持 P2 ASCII / P5 二进制），返回 uint8 二维数组（0=白,255=黑）。"""
    with open(path, 'rb') as f:
        data = f.read()
    idx = 0
    n = len(data)

    def _next_token():
        nonlocal idx
        while idx < n and data[idx:idx + 1].isspace():
            idx += 1
        if idx < n and data[idx:idx + 1] == b'#':
            while idx < n and data[idx:idx + 1] != b'\n':
                idx += 1
            return _next_token()
        start = idx
        while idx < n and not data[idx:idx + 1].isspace():
            idx += 1
        return data[start:idx]

    magic = _next_token()
    width = int(_next_token())
    height = int(_next_token())
    maxval = int(_next_token())
    if magic not in (b'P2', b'P5'):
        raise ValueError(f'不支持的 PGM 格式: {magic}')

    if magic == b'P5':
        # 二进制，可能有单个空白字符分隔
        idx += 1
        raw = data[idx:idx + width * height]
        arr = np.frombuffer(raw, dtype=np.uint8).reshape(height, width)
    else:
        # ASCII 数值流
        vals = np.array(data[idx:].split(), dtype=np.int32)
        arr = vals[:width * height].reshape(height, width).astype(np.uint8)
    if maxval != 255:
        arr = (arr.astype(np.float32) * 255.0 / maxval).clip(0, 255).astype(np.uint8)
    # nav2 地图惯例：0=空闲(白) 255=障碍(黑)；负值(未知) 显示为灰色
    return 255 - arr
