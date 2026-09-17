# -*- coding: utf-8 -*-
"""场地地理数据脚本（CUADC 2026 · 到场调试用，只读不飞）

作用：
  1. 场地数据集中在下方 CONFIG 区，到场后改这一块即可直接调试；
  2. `probe` 就地测试模式：接上飞控读 GPS/罗盘/EKF/原点数据，与预设场地
     经纬度比对，给出"是否已在赛场"的判定和距离/方位角——在任意位置
     （宿舍/实验室/赛场）都能跑，用来验证无人机定位链路正常；
  3. `here` 模式：打印当前实测坐标，直接粘贴回本文件 CONFIG 区；
  4. `record N` 模式：记录 N 秒 GPS 数据到 CSV，评估定位质量（抖动/漂移）。

本脚本只读遥测，不发任何解锁/飞行指令，可在螺旋桨未装时使用。

用法（Jetson 或本机，需 pymavlink）：
  python3 site_geo_check.py            # = probe，就地测试
  python3 site_geo_check.py probe      # 同上
  python3 site_geo_check.py here       # 打印可粘贴回 CONFIG 的实测坐标块
  python3 site_geo_check.py record 60  # 记录 60 秒 GPS 到 CSV

场地数据出处（2026-09-13 检索）：
  - 凤鸣通用机场（自贡航空产业园内，贡井区成佳镇）WGS-84 约
    29.3765N, 104.6258E，机场标高约 345m（OurAirports CN-0051）；
  - 同园区兰田机场基准点 N29°22'0.36" E104°36'44.36"、标高 347.787m，
    跑道真方位 31°35'-211°35'（百度百科），可交叉验证标高量级；
  - 中文维基百科给凤鸣机场 29°22'15"N 104°37'31"E（与上面一致量级）；
  - 高德坐标 29.374038,104.627542 是 GCJ-02 加密坐标系，偏移约 500m，
    严禁直接填给飞控！
  到场后以手机 GPS / 飞控实测为准，用 `here` 模式覆盖下面的占位值。
  磁偏角约 -1.8°（西偏，2026 年近似值）；ArduPilot 开 GPS 后会自动修正，
  此值仅作人工核对罗盘航向用。

注意：任务代码（cuadc_mission）全部使用"起飞点相对 ENU"坐标，
search_x_min_m 等场地几何参数不依赖经纬度，到场后用卷尺/GPS 测距修订
mission_params.yaml 即可，本脚本不负责那部分。
"""

import csv
import math
import sys
import time

# ==================== CONFIG：到场只改这里 ====================

SITE_NAME = "自贡航空产业园（贡井区成佳镇·凤鸣通用机场）"
SITE_LAT = 29.376500          # WGS-84 纬度 (deg) —— 占位值，到场用 here 模式修订
SITE_LON = 104.625800         # WGS-84 经度 (deg)
SITE_ALT_MSL_M = 345.0        # 机场标高 (m, AMSL)，验证气压计/GPS 海拔用

MAG_DECLINATION_DEG = -1.8    # 磁偏角 (deg)，西偏为负；核对罗盘航向用

# 飞控连接：本机 USB 串口填 'COM5'（Windows）/ '/dev/ttyACM0'（Linux）；
# 数传/SITL 填 'udpin:0.0.0.0:14550' 或 'tcp:192.168.x.x:5760' 等
CONNECTION = "COM5"
BAUD = 115200                 # 串口波特率（udp 连接时忽略）

GPS_TOL_M = 2000.0            # 判定"已在赛场"的距离容差 (m)
PROBE_SECONDS = 10            # probe 模式采集时长（秒）
FIX_NAMES = ["无GPS", "无定位", "2D", "3D", "DGPS",
             "RTK浮点", "RTK固定", "静态", "PPP"]

# =============================================================


def equirect_offset_m(lat1, lon1, lat2, lon2):
    """两点( deg )间的近似距离(m)与真北方位角( deg, 从点1指向点2)。

    距离用简单球面公式（小距离误差可忽略）；方位角为真北基准。
    """
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    dp = p2 - p1
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    d = 2 * r * math.asin(math.sqrt(a))
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    brg = (math.degrees(math.atan2(y, x)) + 360.0) % 360.0
    return d, brg


def connect():
    from pymavlink import mavutil
    master = mavutil.mavlink_connection(CONNECTION, baud=BAUD, timeout=5)
    hb = master.wait_heartbeat(timeout=20)
    if hb is None:
        sys.exit("错误：20s 内未收到飞控心跳，检查连接串口/波特率/是否被其他 GCS 占用")
    print(f"已连接飞控 sysid={hb.get_srcSystem()} "
          f"类型={mavutil.mavlink.enums['MAV_TYPE'][hb.type].name}")
    # 请求关键报文：GPS、全局位置、家点、HUD（航向/气压高）
    for msg_id, iv in [(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 200000),
                       (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 200000),
                       (mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION, 1000000),
                       (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 500000),
                       (39, 1000000)]:  # SYSTEM_STATUS（方言未导出常量，用报文 ID）
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, iv, 0, 0, 0, 0, 0)
    return master


def collect(master, seconds):
    """采集 seconds 秒，返回各类型最新报文字典（GPS/GLOBAL 保留全部样本）。"""
    data = {}
    gps_samples, glob_samples = [], []
    deadline = time.time() + seconds
    while time.time() < deadline:
        msg = master.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            continue
        t = msg.get_type()
        if t in ("GPS_RAW_INT", "GLOBAL_POSITION_INT"):
            (gps_samples if t == "GPS_RAW_INT" else glob_samples).append(msg)
        data[t] = msg
    data["_gps_hist"] = gps_samples
    data["_glob_hist"] = glob_samples
    return data


def describe(data):
    """打印飞控定位/罗盘/原点状态，返回 (实测lat, 实测lon, 实测amsl_m)。"""
    gps = data.get("GPS_RAW_INT")
    glob = data.get("GLOBAL_POSITION_INT")
    home = data.get("HOME_POSITION")
    hud = data.get("VFR_HUD")

    print("\n--- 飞控就地测试（只读）---")
    lat = lon = amsl = None
    if gps:
        ft = gps.fix_type
        print(f"GPS1  定位: {FIX_NAMES[ft] if ft < len(FIX_NAMES) else ft} | "
              f"卫星: {gps.satellites_visible} | HDOP: {gps.eph / 100:.2f}")
        print(f"      经纬: {gps.lat / 1e7:.7f}, {gps.lon / 1e7:.7f} | "
              f"椭球高: {gps.alt / 1000:.1f} m")
        lat, lon = gps.lat / 1e7, gps.lon / 1e7
    else:
        print("GPS1  无数据（检查 GPS 模块接线/参数 GPS_TYPE）")

    if glob:
        amsl = glob.alt / 1000.0
        print(f"EKF   AMSL海拔: {amsl:.1f} m | 相对起飞点高度: {glob.relative_alt / 1000:.1f} m")
        if glob.hdg != 65535:
            hdg_mag_ekf = glob.hdg / 100.0  # GLOBAL_POSITION_INT.hdg 为真北航向
            print(f"      真北航向(EKF): {hdg_mag_ekf:.1f}° | "
                  f"地速: {math.hypot(glob.vx, glob.vy) / 100:.2f} m/s")
    else:
        print("EKF   无全局位置数据（未定位或数据流未开）")

    if home:
        hlat, hlon = home.latitude / 1e7, home.longitude / 1e7
        print(f"家点   {hlat:.7f}, {hlon:.7f} | AMSL {home.altitude / 1000:.1f} m")
    else:
        print("家点   尚未设置（GPS 未定位或未解锁过）")

    if hud:
        print(f"HUD   气压海拔: {hud.alt:.1f} m | "
              f"航向: {hud.heading:.1f}° | 空速: {hud.airspeed:.1f} m/s")

    # 与预设场地比对
    if lat is not None:
        d, brg = equirect_offset_m(lat, lon, SITE_LAT, SITE_LON)
        print(f"\n--- 与预设场地比对 ---")
        print(f"预设: {SITE_NAME}")
        print(f"      {SITE_LAT:.7f}, {SITE_LON:.7f} (标高 {SITE_ALT_MSL_M:.0f} m)")
        print(f"实测位置距场地预设点: {d:.1f} m, 方位角 {brg:.1f}°(真北)")
        if d <= GPS_TOL_M:
            print(f"判定: ✅ 已在场地范围（容差 {GPS_TOL_M:.0f} m）")
        else:
            print(f"判定: ❌ 不在场地范围（相距 {d / 1000:.2f} km > 容差 {GPS_TOL_M:.0f} m）")
            print("      —— 属正常（你可能在宿舍/实验室），本判定只做场地确认用")
        if amsl is not None:
            dalt = amsl - SITE_ALT_MSL_M
            print(f"海拔核对: 实测 AMSL {amsl:.1f} m - 预设 {SITE_ALT_MSL_M:.0f} m "
                  f"= {dalt:+.1f} m（|Δ|>50m 需查气压计/GPS 高度源）")
    return lat, lon, amsl


def gps_quality(hist):
    """统计定位质量：样本数/水平散布半径。"""
    if len(hist) < 5:
        return None
    lats = [m.lat / 1e7 for m in hist]
    lons = [m.lon / 1e7 for m in hist]
    mlat, mlon = sum(lats) / len(lats), sum(lons) / len(lons)
    rad = max(equirect_offset_m(mlat, mlon, la, lo)[0] for la, lo in zip(lats, lons))
    return len(hist), mlat, mlon, rad


def cmd_probe(master):
    data = collect(master, PROBE_SECONDS)
    lat, lon, amsl = describe(data)
    q = gps_quality(data["_gps_hist"])
    if q:
        n, mlat, mlon, rad = q
        print(f"\n定位质量: {n} 样本, 水平散布半径 {rad:.2f} m "
              f"({'良好' if rad < 2.0 else '偏差大, 检查天线遮挡/多径'})")
    if lat is not None:
        print("\n提示: 若实测位置即赛场，可运行 `python3 site_geo_check.py here` "
              "获取粘贴块覆盖 CONFIG。")


def cmd_here(master):
    print(f"采集 {PROBE_SECONDS}s ...")
    data = collect(master, PROBE_SECONDS)
    gps = data.get("GPS_RAW_INT")
    glob = data.get("GLOBAL_POSITION_INT")
    if not gps:
        sys.exit("错误：无 GPS 数据，无法生成配置块")
    lat, lon = gps.lat / 1e7, gps.lon / 1e7
    amsl = (glob.alt / 1000.0) if glob else SITE_ALT_MSL_M
    print("\n# ---- 把下面 4 行粘贴覆盖 CONFIG 区对应行 ----")
    print(f'SITE_NAME = "{SITE_NAME}"   # {time.strftime("%Y-%m-%d %H:%M")} 实测修订')
    print(f"SITE_LAT = {lat:.7f}")
    print(f"SITE_LON = {lon:.7f}")
    print(f"SITE_ALT_MSL_M = {amsl:.1f}")


def cmd_record(master, seconds):
    name = f"场地GPS实测_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    print(f"记录 {seconds}s -> {name}")
    rows = []
    deadline = time.time() + seconds
    while time.time() < deadline:
        msg = master.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            continue
        t = msg.get_type()
        if t == "GPS_RAW_INT":
            rows.append([time.strftime("%H:%M:%S"), t, msg.lat / 1e7, msg.lon / 1e7,
                         msg.alt / 1000.0, msg.satellites_visible, msg.eph / 100.0,
                         msg.fix_type])
        elif t == "GLOBAL_POSITION_INT":
            rows.append([time.strftime("%H:%M:%S"), t, msg.lat / 1e7, msg.lon / 1e7,
                         msg.alt / 1000.0, "", "", ""])
    with open(name, "w", newline="", encoding="utf-8") as f:
        csv.writer(f).writerows(
            [["时间", "报文", "纬度", "经度", "海拔m", "卫星数", "HDOP", "fix类型"]] + rows)
    print(f"完成，{len(rows)} 行。可用 Excel 查看漂移情况。")


def main():
    try:  # Windows GBK 控制台下 emoji/特殊字符兜底
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    mode = sys.argv[1] if len(sys.argv) > 1 else "probe"
    arg = int(sys.argv[2]) if len(sys.argv) > 2 else PROBE_SECONDS

    print(f"场地: {SITE_NAME} | 预设 {SITE_LAT:.7f}, {SITE_LON:.7f} | "
          f"磁偏角 {MAG_DECLINATION_DEG}°")
    master = connect()
    try:
        if mode == "probe":
            cmd_probe(master)
        elif mode == "here":
            cmd_here(master)
        elif mode == "record":
            cmd_record(master, arg)
        else:
            sys.exit(f"未知模式: {mode}（可用: probe / here / record N）")
    finally:
        master.close()


if __name__ == "__main__":
    main()
