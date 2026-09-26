# -*- coding: utf-8 -*-
"""拉取飞控最新 DataFlash 日志的头部 FMT 表 + 尾部 MSG/ERR 人类文本行（排障用，只读）。

用途：解锁被拒但 STATUSTEXT 无报文这类「沉默失败」时，日志里的 MSG/ERR 行
是唯一权威记录（EKF3 初始化失败、Arming 拒绝原因等）。
用法：python fc_log_tail.py [COM口] [尾部字节数，默认 65536]
"""
import sys
import time

from pymavlink import mavutil

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM17"
TAIL = int(sys.argv[2]) if len(sys.argv) > 2 else 65536

m = mavutil.mavlink_connection(PORT, baud=115200, timeout=5)
m.wait_heartbeat(timeout=15)
print(f"心跳 OK（target {m.target_system}/{m.target_component}）")


def fetch(log_id, ofs, count):
    """按偏移拉取日志数据，返回重组字节（拿不齐就返回已得部分）"""
    chunks = {}
    end_ofs = ofs + count
    p = ofs
    while p < end_ofs:
        n = min(8192, end_ofs - p)
        m.mav.log_request_data_send(m.target_system, m.target_component, log_id, p, n)
        t_end = time.time() + 8
        got = False
        while time.time() < t_end and not got:
            msg = m.recv_match(type="LOG_DATA", blocking=False)
            if msg:
                mo = msg.ofs if hasattr(msg, "ofs") else msg.of_log_data
                chunks[mo] = bytes(msg.data)[: msg.count]
                if mo == p:
                    got = True
            time.sleep(0.02)
        if not got:
            print(f"WARN: 偏移 {p} 超时")
            break
        p += n
    return b"".join(chunks[k] for k in sorted(chunks))


def parse(blob, fmts, records):
    """DataFlash 块流解析：[len u8][type u8][data]，len 含头两字节"""
    i, n = 0, len(blob)
    while i + 2 <= n:
        ln = blob[i]
        tp = blob[i + 1]
        if ln < 3 or i + ln > n:
            i += 1
            continue
        data = blob[i + 2 : i + ln]
        if tp == 0x80:
            if len(data) >= 3:
                ftype = data[0]
                if ftype not in fmts:
                    name = data[2:6].split(b"\x00")[0].decode("ascii", "replace")
                    fmts[ftype] = name
        elif tp in fmts:
            name = fmts[tp]
            if name == "MSG" and len(records["MSG"]) < 500:
                records["MSG"].append(data.split(b"\x00")[0].decode("ascii", "replace"))
            elif name == "ERR" and len(records["ERR"]) < 200:
                records["ERR"].append(data.hex())
        i += ln


# 1) 枚举日志
m.mav.log_request_list_send(m.target_system, m.target_component, 0, 999)
logs = {}
end = time.time() + 10
while time.time() < end:
    msg = m.recv_match(type="LOG_ENTRY", blocking=False)
    if msg:
        logs[msg.id] = (msg.size, msg.num_logs, msg.last_log_num)
        end = time.time() + 2
if not logs:
    print("ERROR: 没有任何日志（LOG_DISARMED 开了吗？）")
    sys.exit(1)
newest = max(logs)
size = logs[newest][0]
print(f"共 {logs[newest][1]} 个日志（last #{logs[newest][2]}），最新 #{newest}（{size} 字节）")

# 2) 头部建 FMT 表
fmts = {}
head = fetch(newest, 0, 16384)
parse(head, fmts, {"MSG": [], "ERR": []})
print(f"FMT 表：{len(fmts)} 种消息类型，含 MSG={'MSG' in fmts.values()} ERR={'ERR' in fmts.values()}")

# 3) 尾部解码
tail_len = min(TAIL, size)
start = size - tail_len
print(f"拉取尾部 {tail_len} 字节（起点 {start}）...")
tail = fetch(newest, start, tail_len)
records = {"MSG": [], "ERR": []}
parse(tail, fmts, records)
print(f"取回 {len(tail)} 字节\n")

print(f"--- MSG 文本行（共 {len(records['MSG'])} 行，最后 45 行）---")
for line in records["MSG"][-45:]:
    print(" ", line)
print(f"--- ERR 原始 hex（subsys+ecode+时间，共 {len(records['ERR'])} 条）---")
for line in records["ERR"][-15:]:
    print(" ", line)
m.close()
