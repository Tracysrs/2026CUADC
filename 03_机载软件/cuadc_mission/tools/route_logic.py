"""route_logic 的 Python 镜像（纯 stdlib）——与 tools/drop_logic.py 同款双端纪律。

C++ 权威实现 = include/cuadc_mission/route_logic.hpp；本文件逐函数移植，
用例与 test/test_route_logic.cpp 一一镜像（tools/test_route_logic.py）。
航线构建收敛纯函数的原因（勿回退）：2026-09-12"横着搜"修复——出生 yaw 与
场地前进方向错 90° 这类 bug，靠这里的回归锚点守门。
"""

import math

PI = math.pi


def heading_enu_rad_from_compass_deg(compass_deg: float) -> float:
    """罗盘航向（度，0=N 顺时针）→ ENU 航向角（弧度，自 +X 逆时针）。
    SSOT §10.2-1：yaw_ENU = 90° − compass，坐标系列坑之首，勿改。"""
    return (90.0 - compass_deg) * math.pi / 180.0


def field_to_local(yaw_enu: float, origin_x: float, origin_y: float,
                   origin_z: float, fx: float, fy: float, fz: float):
    """场地系（x 机头，y 左）→ 本地 ENU。yaw_enu = 起飞瞬间 ENU 航向（弧度）。
    回归锚点：罗盘 0°（机头朝北）时 field(1,0) → ENU(0,1)（北=机头方向），
    field(0,1)（机头左侧）→ ENU(-1,0)（西=朝北时的左）。"""
    c, s = math.cos(yaw_enu), math.sin(yaw_enu)
    return (origin_x + c * fx - s * fy,
            origin_y + s * fx + c * fy,
            origin_z + fz)


def build_serpentine(x0: float, x1: float, y0: float, y1: float,
                     lanes: int, along_x: bool):
    """弓字形蛇形航线（场地系，按飞行顺序，2×lanes 个航点，lanes≥1）。
    along_x=True ：带沿 x 扫，带位沿 y 均布（搜索区现状）；
    along_x=False：带沿 y 扫，带位沿 x 均布（侦察区长轴扫）。
    奇偶带交替方向；lanes==1 时带位居中。"""
    route = []
    if lanes < 1:
        return route
    for lane in range(lanes):
        t = 0.5 if lanes == 1 else lane / (lanes - 1)
        forward = lane % 2 == 0
        if along_x:
            y = y0 + (y1 - y0) * t
            route.append((x0 if forward else x1, y))
            route.append((x1 if forward else x0, y))
        else:
            x = x0 + (x1 - x0) * t
            route.append((x, y0 if forward else y1))
            route.append((x, y1 if forward else y0))
    return route


def nadir_across_half_width(alt_m: float, hfov_rad: float,
                            img_w_px: float, img_h_px: float) -> float:
    """下视相机横向（垂直于带方向）半覆盖宽度（米）。
    SDF horizontal_fov 张在图像宽度上，横向沿高度方向：
    tan(vfov/2) = tan(hfov/2)·(H_px/W_px)。iris_d435i：hfov=1.5rad、848×480
    → 2m 高横向半宽 ≈1.05m、4m 高 ≈2.11m。"""
    return abs(alt_m) * math.tan(hfov_rad / 2.0) * (img_h_px / img_w_px)


def max_cross_track_distance(span: float, lanes: int) -> float:
    """均匀布带下区内任一点到最近带的最大横向距离 = 带间距/2。
    覆盖判据：该值 ≤ nadir_across_half_width（带间无盲区），lanes==1 取半跨。"""
    s = abs(span)
    if lanes <= 1:
        return s / 2.0
    return s / (2.0 * (lanes - 1))
