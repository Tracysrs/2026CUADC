# -*- coding: utf-8 -*-
"""检查定高模块（测距仪）接入状态：串口配置、RNGFND1 参数、DISTANCE_SENSOR 实时数据流"""
import sys
import time
from pymavlink import mavutil

port = sys.argv[1] if len(sys.argv) > 1 else "COM17"
master = mavutil.mavlink_connection(port, baud=115200, timeout=5)
hb = master.wait_heartbeat(timeout=15)
print(f"已连接 {port}，载具: {mavutil.mavlink.enums['MAV_TYPE'][hb.type].name}")

# 请求数据流：DISTANCE_SENSOR(132) 10Hz + RANGEFINDER(173 在 beta 里可能没有，略)
for msg_id, iv in [(132, 100000)]:
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                 msg_id, iv, 0, 0, 0, 0, 0)

# 读关键参数（RNGFND1_* 懒加载：TYPE=0 时子参数不存在属正常）
want = ["SERIAL2_PROTOCOL", "SERIAL2_BAUD", "BRD_SER2_RTSCTS",
        "SERIAL5_PROTOCOL", "SERIAL5_BAUD",
        "RNGFND1_TYPE", "RNGFND1_ORIENT", "RNGFND1_MIN", "RNGFND1_MAX",
        "RNGFND1_SCALING", "RNGFND1_GNDCLR", "RNGFND1_ADDR", "RNGFND1_FUNCTION",
        "EK3_RNG_USE_HGT", "EK3_SRC1_POSZ", "PLND_ENABLED"]
params = {}
for p in want:
    master.mav.param_request_read_send(master.target_system, master.target_component, p.encode(), -1)

end = time.time() + 8
ds_msgs = []
while time.time() < end:
    msg = master.recv_match(blocking=False)
    if msg:
        t = msg.get_type()
        if t == "PARAM_VALUE":
            key = msg.param_id.rstrip(b"\x00").decode() if isinstance(msg.param_id, bytes) else msg.param_id
            if key not in params:
                params[key] = msg.param_value
        elif t == "DISTANCE_SENSOR":
            ds_msgs.append(msg)
    time.sleep(0.02)

SERIAL_PROTO = {1: "MAVLink1", 2: "MAVLink2", 5: "RS232", 7: "串口无人值守遥测", 11: "Rangefinder",
                13: "SLCAN", 15: "Frsky D", 16: "LTM", 17: "显示板", 22: "IRC Telemetry", 23: "MAVLink2 高流量"}
ORIENT = {0: "前", 1: "前右", 2: "右", 3: "后右", 4: "后", 5: "后左", 6: "左", 7: "前左", 24: "上视", 25: "下视"}

print("\n--- 串口与测距参数 ---")
for k in want:
    if k in params:
        v = params[k]
        note = ""
        if k.endswith("_PROTOCOL"):
            note = f"  ({SERIAL_PROTO.get(int(v), '?')})"
        if k == "RNGFND1_ORIENT" and v in ORIENT:
            note = f"  ({ORIENT[int(v)]})"
        print(f"  {k} = {v:g}{note}")
    else:
        print(f"  {k} = 不存在（懒加载未激活）")

# DISTANCE_SENSOR 汇总
print("\n--- DISTANCE_SENSOR 监听（8s 窗口）---")
if ds_msgs:
    ds = ds_msgs[-1]
    dists = [m.current_distance for m in ds_msgs]
    print(f"  帧数: {len(ds_msgs)} | latest: {ds.current_distance}cm (min {min(dists)} / max {max(dists)})")
    # sensor_id 在部分 pymavlink 版本的 distance_sensor 里没有（新版字段名漂移），容错取
    sid = getattr(ds, "sensor_id", getattr(ds, "id", "?"))
    print(f"  方向: {ORIENT.get(ds.orientation, ds.orientation)} | 类型: {ds.type} | id: {sid} | 方差: {ds.covariance}")
    print(f"  换算: {ds.current_distance/100:.2f} m")
else:
    print("  无数据——飞控侧没有任何测距仪在出数")

# 结论
print("\n--- 结论 ---")
t = params.get("RNGFND1_TYPE", -1)
if t == 0:
    print("  RNGFND1_TYPE=0：测距仪功能禁用（装机前预期状态），DISTANCE_SENSOR 不会出数")
elif t < 0:
    print("  RNGFND1_TYPE 读取失败")
else:
    print(f"  RNGFND1_TYPE={t:g}：" + {20: "TFmini 兼容串口协议（MT-01P 主路线）", 10: "MAVLink 测距仪", 25: "Benewake I2C（勿与串口版混用）"}.get(int(t), "其他类型"))
if params.get("SERIAL2_PROTOCOL", 0) == 2:
    print("  SERIAL2_PROTOCOL=2（TELEM2=MAVLink2，留给 Jetson）：TELEM2 上按 TFmini 协议发的字节会被当 MAVLink 解析，测距不出数")
if params.get("SERIAL5_PROTOCOL", 0) == 11 and t == 0:
    print("  SERIAL5_PROTOCOL=11（TELEM3=Rangefinder）已就位，但 RNGFND1_TYPE=0 未启用——需两遍导入激活（中间重启）")

master.close()
