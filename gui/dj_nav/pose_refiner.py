"""初始位姿自动精修：把「大概点一下」的初始位姿用激光匹配对齐到地图。

原理（相关扫描匹配 / correlative scan matching）：
  1. 对地图占据栅格做 chamfer 3-4 距离变换，得到「每格到最近障碍的距离场」；
  2. 对候选修正量 (dx, dy, dyaw)，把世界系激光点做刚体变换后查距离场，
     平均距离越小 = 激光与地图墙体贴合越好；
  3. 三级由粗到细搜索（步长 0.12m/5° → 0.02m/0.8° → 0.005m/0.2°），
     取最优解作为修正量。

纯 numpy 实现（不引入 scipy），计算在后台线程执行（_remote_task），
典型耗时 <1s。修正量是 map 系刚体变换，主窗口把它作用到机器人位姿后
重发 /initialpose —— 用户只需「指个大概」，剩下的交给匹配。

研发：fofomo
"""
import math

import numpy as np

# 距离场封顶（格）：图外/远离障碍处一律按此值罚分，
# 防止把激光「对」到地图空白/未知区域（那里没有约束力）
_FIELD_CAP = 60.0


def build_distance_field(occ_grid):
    """占据栅格 → chamfer 3-4 距离场（单位：格）。

    occ_grid: 0=空闲 100(>50)=障碍 -1=未知（行 0 = y 最小，ROS 约定）
    返回 float 数组：障碍格=0，其余=到最近障碍的近似欧氏距离（封顶）。

    向量化实现：行内传播用 minimum.accumulate 一行一次算完，
    整场只需 O(h) 次 numpy 运算，比逐格 Python 循环快两个数量级。
    """
    occ = np.asarray(occ_grid)
    h, w = occ.shape
    big = _FIELD_CAP * 3.0            # chamfer 权重按 3 缩放，最后除回 3
    g = np.where(occ > 50, 0.0, big)  # 障碍=0，其余（含未知）从封顶起算
    ar = np.arange(w, dtype=np.float64)

    def _row_lr(row):
        """行内左→右传播：f[j] = min(f[j], f[j-1] + 3)。"""
        return np.minimum.accumulate(row - 3.0 * ar) + 3.0 * ar

    def _row_rl(row):
        """行内右→左传播：f[j] = min(f[j], f[j+1] + 3)。

        正确的扫描技巧：h[j] = f[j] + 3j，从右取前缀最小
        m[j] = min_{i>=j} h[i]，则 m[j] - 3j = min_{i>=j}(f[i] + 3(i-j))。
        ⚠ 旧实现误用 "-3j / +3j"（与左→右传播同款），在宽地图上产生
        大幅负距离（w=1562 时 ~-70m）→ 精修恒判"已贴合"从未生效
        （2026-09-30 实测日志"残差 -831cm / -7016cm"定位）。
        """
        h = row + 3.0 * ar
        m = np.minimum.accumulate(h[::-1])[::-1]
        return m - 3.0 * ar

    # ---- 正向扫（上→下）：上邻域(3)/左上(4)/右上(4) + 行内左→右 ----
    g[0] = _row_lr(g[0])
    for i in range(1, h):
        prev = g[i - 1]
        row = g[i]
        dl = np.empty(w)
        dl[0] = big
        dl[1:] = prev[:-1] + 4.0
        dr = np.empty(w)
        dr[-1] = big
        dr[:-1] = prev[1:] + 4.0
        row = np.minimum(row, np.minimum(prev + 3.0, np.minimum(dl, dr)))
        g[i] = _row_lr(row)

    # ---- 反向扫（下→上）：下邻域(3)/左下(4)/右下(4) + 行内右→左 ----
    for i in range(h - 2, -1, -1):
        nxt = g[i + 1]
        row = g[i]
        dl = np.empty(w)
        dl[0] = big
        dl[1:] = nxt[:-1] + 4.0
        dr = np.empty(w)
        dr[-1] = big
        dr[:-1] = nxt[1:] + 4.0
        row = np.minimum(row, np.minimum(nxt + 3.0, np.minimum(dl, dr)))
        g[i] = _row_rl(row)

    g /= 3.0                          # 还原为「格」单位
    np.minimum(g, _FIELD_CAP, out=g)
    return g


def make_scorer(field, ox, oy, res):
    """构造评分函数 score(pts, dx, dy, dyaw) → 平均残差（米）。

    把世界系激光点做刚体变换（先绕 map 原点转 dyaw，再平移 dx/dy），
    查距离场取平均；图外点按封顶值罚分。
    """
    h, w = field.shape

    def score(pts, dx, dy, dyaw):
        c, s = math.cos(dyaw), math.sin(dyaw)
        x = pts[:, 0] * c - pts[:, 1] * s + dx
        y = pts[:, 0] * s + pts[:, 1] * c + dy
        gx = ((x - ox) / res).astype(np.int64)
        gy = ((y - oy) / res).astype(np.int64)
        inside = (gx >= 0) & (gx < w) & (gy >= 0) & (gy < h)
        n = len(pts)
        if not inside.any():
            return _FIELD_CAP * res
        total = float(field[gy[inside], gx[inside]].sum())
        total += (n - int(inside.sum())) * _FIELD_CAP
        return total / n * res

    return score


def refine_pose(pts, occ_grid, ox, oy, res, max_xy=1.2, max_yaw_deg=50.0):
    """在粗略对准附近搜索最优刚体修正 (dx, dy, dyaw)。

    pts:      [(x, y), ...] 世界系激光点（按当前「大概位姿」变换过的）
    occ_grid: 占据栅格（0=空闲 >50=障碍 -1=未知，行 0 = y 最小）
    ox/oy/res: 栅格原点与分辨率
    max_xy/max_yaw_deg: 修正搜索范围 —— 超出说明粗位姿太离谱，
               应提示用户点准一点或改用「全局定位」

    返回 dict:
      ok                  是否找到可信修正
      correction          (dx, dy, dyaw) map 系刚体修正，作用于位姿：
                          x' = cos(dyaw)·x - sin(dyaw)·y + dx
                          y' = sin(dyaw)·x + cos(dyaw)·y + dy
                          yaw' = yaw + dyaw
      score_before/after  修正前后的平均残差（米），越小贴合越好
      reason              ok=False 时的人话原因
    """
    pts = np.asarray(pts, dtype=np.float64)
    if len(pts) < 60:
        return {'ok': False, 'reason': f'有效激光点太少（{len(pts)}）'}
    # 降采样到 ≤360 点：匹配精度足够，搜索快一倍以上
    if len(pts) > 360:
        pts = pts[::int(math.ceil(len(pts) / 360.0))]

    field = build_distance_field(occ_grid)
    score = make_scorer(field, ox, oy, res)
    s0 = score(pts, 0.0, 0.0, 0.0)
    if s0 >= _FIELD_CAP * res * 0.999:
        return {'ok': False,
                'reason': '激光点全部落在地图外/未知区，无法匹配'}
    if s0 <= 0.06:
        # 激光已经贴在墙上：粗位姿本来就准，直接通过（零修正）
        return {'ok': True, 'correction': (0.0, 0.0, 0.0),
                'score_before': s0, 'score_after': s0}

    max_yaw = math.radians(max_yaw_deg)
    best = (0.0, 0.0, 0.0, s0)
    # 三级由粗到细：每级以上一级最优解为【固定中心】扫整张网格，
    # 整级扫完才更新中心。
    # ⚠ 旧实现在循环内即时更新 best、候选却按 best+偏移计算 —— 网格
    # 中心随 best 漂移，(-0.24,-0.24) 这类好解会被错过，收敛到搜索
    # 边界上的局部劣解（2026-09-30 合成测试定位：期望残差 0，实际
    # 收敛到边界 (dx=1.2, dyaw=20°) 残差 0.148）。
    stages = [
        (max_xy, max_yaw, 10),                # 步长 0.12m / 5°
        (max_xy / 10.0, max_yaw / 10.0, 6),   # 步长 0.02m / 0.83°
        (max_xy / 60.0, max_yaw / 60.0, 4),   # 步长 0.005m / 0.21°
    ]
    for span_xy, span_yaw, n in stages:
        cx, cy, cyaw, bs = best
        step_xy = span_xy / n
        step_yaw = span_yaw / n
        offs = [i * step_xy for i in range(-n, n + 1)]
        yaws = [i * step_yaw for i in range(-n, n + 1)]
        for dx in offs:
            for dy in offs:
                for dyaw in yaws:
                    v = score(pts, cx + dx, cy + dy, cyaw + dyaw)
                    if v < bs:
                        bs = v
                        best = (cx + dx, cy + dy, cyaw + dyaw, v)

    bx, by, byaw, bs = best
    # 撞到搜索边界 → 粗位姿偏差过大，解不可信（边缘外可能还有更优解）
    eps = 1e-6
    if (abs(bx) >= max_xy - eps or abs(by) >= max_xy - eps
            or abs(byaw) >= max_yaw - eps):
        return {'ok': False, 'score_before': s0,
                'reason': f'偏差超出精修范围（>{max_xy:.1f}m 或 '
                          f'>{max_yaw_deg:.0f}°），请点得更准一些，'
                          f'或改用「全局定位」'}
    # 残差需有明显改善（>10%）且绝对值可用（≤30cm）才应用，
    # 防止在退化环境（长走廊/空旷区）把噪声当信号放大
    if bs < s0 * 0.9 and bs <= 0.30:
        return {'ok': True, 'correction': (float(bx), float(by), float(byaw)),
                'score_before': s0, 'score_after': bs}
    return {'ok': False, 'score_before': s0, 'score_after': bs,
            'reason': f'匹配质量不佳（残差 {bs * 100:.0f}cm），未应用修正——'
                      f'环境可能太对称/太空旷，请手动核对'}
