# -*- coding: utf-8 -*-
"""bench 机架帧白筒预标注（正式版，2026-10-02 定稿；口径与流程见同目录《标注要求-白筒seg.md》）。
流程：deliver barrel seg 模型出提案
  -> 白色占比滤误检（掩码外沿环带 S<60&V>55 占比）
  -> 圆形提案：射线搜索贴口沿 + 鲁棒椭圆拟合（口径=筒口外沿，不含筒身）
  -> 长条提案（侧倒筒）：box 内白色连通域取整段筒身轮廓（difficult=True）
  -> 补漏：蓝垫区域白色连通域补检（侧视大筒等模型漏检）
  -> X-AnyLabeling json + 可视化
"""
import cv2, numpy as np, json, os, sys, glob

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.normpath(os.path.join(HERE, '..', 'deliver_work', 'deliver_barrel', 'best.pt'))

def white_score(hsv):
    V = hsv[:, :, 2].astype(np.float32)
    S = hsv[:, :, 1].astype(np.float32)
    return V - 0.8 * S

def point_in(poly, p):
    return cv2.pointPolygonTest(poly.astype(np.float32), (float(p[0]), float(p[1])), False) >= 0

def ray_radius(poly, center, k=72):
    poly = poly.astype(np.float32)
    c = np.array(center, np.float32)
    r = np.zeros(k)
    for i in range(k):
        th = 2 * np.pi * i / k
        d = np.array([np.cos(th), np.sin(th)], np.float32)
        lo, hi = 0.0, 1.0
        while point_in(poly, c + d * hi):
            hi *= 1.5
            if hi > 5000:
                break
        for _ in range(24):
            mid = (lo + hi) / 2
            if point_in(poly, c + d * mid):
                lo = mid
            else:
                hi = mid
        r[i] = lo
    return r

def refine_ellipse(img, poly):
    """把模型多边形精修成贴筒口外沿的椭圆；返回 (点集, 方式)"""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    ws = white_score(hsv)
    h, w = ws.shape
    poly = poly.astype(np.float32)
    center = poly.mean(0)
    r0 = ray_radius(poly, center)
    base = np.median(r0[r0 > 0])
    pts = []
    for i in range(len(r0)):
        th = 2 * np.pi * i / len(r0)
        d = np.array([np.cos(th), np.sin(th)], np.float32)
        lo = max(1.0, 0.85 * r0[i])
        hi = min(np.hypot(w, h), 1.15 * r0[i] + 6)
        ts = np.arange(hi, lo, -1.0)
        ps = np.clip(np.round(center + d[None, :] * ts[:, None]).astype(int), 0, [w - 1, h - 1])
        sc = ws[ps[:, 1], ps[:, 0]]
        tmax = sc.max()
        if tmax < 55:
            pts.append(center + d * r0[i]); continue
        th_ = max(35.0, 0.5 * tmax)
        idx = np.argmax(sc >= th_)
        pts.append(center + d * ts[idx])
    pts = np.array(pts, np.float32)
    try:
        for _ in range(2):
            e = cv2.fitEllipse(pts)
            (ecx, ecy), (d1, d2), ang = e
            a, b = d1 / 2, d2 / 2
            t = np.arctan2(pts[:, 1] - ecy, pts[:, 0] - ecx)
            re = (a * b) / np.sqrt((b * np.cos(t - np.deg2rad(ang))) ** 2 + (a * np.sin(t - np.deg2rad(ang))) ** 2)
            rr = np.hypot(pts[:, 0] - ecx, pts[:, 1] - ecy)
            res = np.abs(rr - re)
            keep = res < max(3.0, 2.5 * np.median(res))
            if keep.all():
                break
            pts = pts[keep]
        e = cv2.fitEllipse(pts)
        (ecx, ecy), (d1, d2), ang = e
        if not (0.5 * base < (d1 + d2) / 4 < 1.6 * base):
            return poly, 'keep'
        th = np.linspace(0, 2 * np.pi, 49)[:-1]
        ct, st = np.cos(th), np.sin(th)
        xs = d1 / 2 * ct * np.cos(np.deg2rad(ang)) - d2 / 2 * st * np.sin(np.deg2rad(ang)) + ecx
        ys = d1 / 2 * ct * np.sin(np.deg2rad(ang)) + d2 / 2 * st * np.cos(np.deg2rad(ang)) + ecy
        return np.stack([xs, ys], 1), 'ellipse'
    except cv2.error:
        return poly, 'keep'

def _white_mask_roi(hsv_roi, v_ref=None):
    """ROI 内白色掩码：V 阈自适应（相对环境），S 上限放宽到 85 以纳入偏蓝灰的暗筒身"""
    S, V = hsv_roi[:, :, 1].astype(np.int32), hsv_roi[:, :, 2].astype(np.int32)
    if v_ref is None:
        v_ref = 0.55 * np.percentile(V, 45)  # 筒身常比垫/地面暗，只卡亮度下限
    return ((S < 110) & (V > max(55, v_ref))).astype(np.uint8)

def whole_body_refine(img, box):
    """侧倒/侧视筒：box 内白色连通域 -> 整段筒身轮廓多边形"""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    H, W = img.shape[:2]
    x1, y1, x2, y2 = [int(v) for v in box]
    pad = int(1.00 * max(x2 - x1, y2 - y1)) + 8
    x1, y1 = max(0, x1 - pad), max(0, y1 - pad)
    x2, y2 = min(W, x2 + pad), min(H, y2 + pad)
    wm = _white_mask_roi(hsv[y1:y2, x1:x2])
    wm = cv2.morphologyEx(wm, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(wm, 8)
    if n < 2:
        return None
    i = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
    comp = (lab == i).astype(np.uint8)
    if stats[i, cv2.CC_STAT_AREA] < 400:
        return None
    cnts, _ = cv2.findContours(comp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    c = max(cnts, key=cv2.contourArea)
    eps = 3.0
    ap = cv2.approxPolyDP(c, eps, True).reshape(-1, 2) + [x1, y1]
    if len(ap) < 6:
        ap = cv2.approxPolyDP(c, eps * 0.5, True).reshape(-1, 2) + [x1, y1]
    if len(ap) < 6:
        return None
    return ap.astype(np.float32)

def mask_whiteness(img, mask):
    """掩码【外沿环带】低饱和（白沿）占比：S<60 且 V>55"""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = (np.asarray(mask) > 0).astype(np.uint8)
    k = max(3, int(np.sqrt(mask.sum()) * 0.18))
    inner = cv2.erode(mask, np.ones((k, k), np.uint8))
    ring = (mask > 0) & (inner == 0)
    if ring.sum() < 30:
        ring = mask > 0
    S = hsv[:, :, 1][ring].astype(np.float32)
    V = hsv[:, :, 2][ring].astype(np.float32)
    return float(((S < 60) & (V > 55)).mean())

def extend_body(img, ellipse_pts):
    """口沿椭圆是否连着长筒身（侧倒/侧视）：附着白色区域最大维 >= 1.4x 口沿长轴
    -> 返回整段筒身轮廓；否则 None（站立筒，保持口沿口径）"""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    H, W = img.shape[:2]
    em = np.zeros((H, W), np.uint8)
    cv2.fillPoly(em, [ellipse_pts.astype(np.int32)], 1)
    ex, ey, ew, eh = cv2.boundingRect(em)
    major = float(max(ew, eh))
    pad = int(2.0 * major) + 20
    x1, y1 = max(0, ex - pad), max(0, ey - pad)
    x2, y2 = min(W, ex + ew + pad), min(H, ey + eh + pad)
    roi = hsv[y1:y2, x1:x2]
    faceV = np.percentile(roi[em[y1:y2, x1:x2] > 0][:, 2], 60) if (em[y1:y2, x1:x2] > 0).any() else 120.0
    S, V = roi[:, :, 1].astype(np.int32), roi[:, :, 2].astype(np.int32)
    Hu = roi[:, :, 0].astype(np.int32)
    vth = max(62.0, 0.60 * faceV)
    yellow = (Hu > 18) & (Hu < 40) & (S > 90)  # 卷尺
    wm = ((S < 110) & (V > vth) & (~yellow)).astype(np.uint8)
    wm = cv2.morphologyEx(wm, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(wm, 8)
    if n < 2:
        return None
    touch = cv2.dilate(em[y1:y2, x1:x2], np.ones((21, 21), np.uint8))
    emA = cv2.countNonZero(em)
    area_cap = (2.2 * major) ** 2  # 筒身+地面粘连的大 blob 会被拦下
    best, best_ext = None, 0.0
    fcy = ey + eh / 2.0  # 口沿中心 y（图向下为正）
    for i in range(1, n):
        bx, by, bw_, bh_ = stats[i, :4]
        if not ((lab == i) & (touch > 0)).any():  # 必须与口沿相邻
            continue
        ext = max(bw_, bh_) / max(1.0, major)
        below = (stats[i, cv2.CC_STAT_TOP] + bh_ / 2.0) - fcy
        if below > 0.35 * major:  # 筒身在口沿正下方 = 站立筒，保持只标口沿
            continue
        if ext > best_ext and 0.4 * emA < stats[i, cv2.CC_STAT_AREA] <= area_cap:
            best, best_ext = i, ext
    if best is None or best_ext < 1.4:
        return None
    region = cv2.bitwise_or((lab == best).astype(np.uint8), em[y1:y2, x1:x2])
    cnts, _ = cv2.findContours(region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    c = max(cnts, key=cv2.contourArea)
    ap = cv2.approxPolyDP(c, 3.0, True).reshape(-1, 2) + [x1, y1]
    return ap.astype(np.float32) if len(ap) >= 6 else None

def mat_detect(img, existing):
    """补漏检：蓝垫 bbox（含垫上筒洞）内白色连通域 -> 候选筒；与已有形状去重"""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    H, W = img.shape[:2]
    S, V, Hu = hsv[:, :, 1].astype(np.int32), hsv[:, :, 2].astype(np.int32), hsv[:, :, 0].astype(np.int32)
    blue = ((Hu > 95) & (Hu < 130) & (S > 70) & (V > 40)).astype(np.uint8)
    blue = cv2.morphologyEx(blue, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    nb, labb, sb, _ = cv2.connectedComponentsWithStats(blue, 8)
    out = []
    for i in range(1, nb):
        if sb[i, cv2.CC_STAT_AREA] < 5000:
            continue
        bx, by, bw, bh = sb[i, :4]
        pad = 25
        x1, y1 = max(0, bx - pad), max(0, by - pad)
        x2, y2 = min(W, bx + bw + pad), min(H, by + bh + pad)
        matV = V[(labb == i) & (S > 70)]
        if matV.size < 1000:
            continue
        vth = np.percentile(matV, 75) + 15
        yellow = (Hu > 18) & (Hu < 40) & (S > 80)  # 卷尺
        white = ((S < 100) & (V > vth) & (~yellow)).astype(np.uint8)[y1:y2, x1:x2]
        white = cv2.morphologyEx(white, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        white = cv2.morphologyEx(white, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
        nw, labw, sw, _ = cv2.connectedComponentsWithStats(white, 8)
        for j in range(1, nw):
            a = sw[j, cv2.CC_STAT_AREA]
            if a < 500 or a > 90000:
                continue
            cnts, _ = cv2.findContours((labw == j).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            c = max(cnts, key=cv2.contourArea)
            cbx, cby, cbw, cbh = cv2.boundingRect(c)
            cbx += x1; cby += y1
            cx, cy = cbx + cbw / 2, cby + cbh / 2
            dup = False
            for s in existing:
                px = np.array(s['points'])
                ex1, ey1 = px.min(0); ex2, ey2 = px.max(0)
                if abs(cx - (ex1 + ex2) / 2) < 0.6 * (ex2 - ex1 + cbw) and abs(cy - (ey1 + ey2) / 2) < 0.6 * (ey2 - ey1 + cbh):
                    dup = True; break
            if dup:
                continue
            # 蓝垫封闭性：从裁剪框边界沿「非蓝非候选」区域泛洪，够不着候选(9px) = 被垫子包住的筒；
            # 垫缘条带/牌边地面与外界直连，泛洪直接到达 -> 排除
            compm = (labw == j).astype(np.uint8)
            ch, cw = compm.shape
            walkable = ((compm == 0) & (blue[y1:y2, x1:x2] == 0)).astype(np.uint8)
            outside = np.zeros((ch + 2, cw + 2), np.uint8)
            ff = walkable.copy()
            for seed in [(0, 0), (cw - 1, 0), (0, ch - 1), (cw - 1, ch - 1), (cw // 2, 0), (cw // 2, ch - 1), (0, ch // 2), (cw - 1, ch // 2)]:
                if walkable[seed[1], seed[0]] == 1 and ff[seed[1], seed[0]] == 1:
                    cv2.floodFill(ff, outside, seed, 2)
            reach = cv2.dilate((ff == 2).astype(np.uint8), np.ones((19, 19), np.uint8))
            if cv2.countNonZero(cv2.bitwise_and(reach, compm)) > 0.04 * a:
                continue
            elong = max(cbw, cbh) / max(1, min(cbw, cbh))
            if elong <= 1.35:
                # 环形判据（仅口沿候选）：筒口白环实心度低；贴纸/白纸块实心（~0.9）。
                # matbody（整段筒身）是实心长条，豁免
                hh = cv2.convexHull(c)
                if a / max(1.0, cv2.contourArea(hh)) > 0.78:
                    continue
            if elong > 1.35:
                if a < 2000:
                    continue  # 长条小斑块（垫缘条带/卷尺端头）：两个分支都不收
                cd = cv2.dilate((labw == j).astype(np.uint8), np.ones((19, 19), np.uint8))
                cnts2, _ = cv2.findContours(cd, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                c = max(cnts2, key=cv2.contourArea)
                ap = cv2.approxPolyDP(c, 3.0, True).reshape(-1, 2).astype(np.float32) + [x1, y1]
                if len(ap) >= 6:
                    sbr = cv2.boundingRect(c)
                    out.append({'label': 'barrel', 'points': ap, 'how': 'matbody', 'difficult': True,
                                'sam_box': [sbr[0] + x1 - 12, sbr[1] + y1 - 12, sbr[0] + x1 + sbr[2] + 12, sbr[1] + y1 + sbr[3] + 12]})
            else:
                e = cv2.fitEllipse(c)
                (ecx, ecy), (d1, d2), ang = e
                th = np.linspace(0, 2 * np.pi, 49)[:-1]
                ct, st = np.cos(th), np.sin(th)
                xs = d1 / 2 * ct * np.cos(np.deg2rad(ang)) - d2 / 2 * st * np.sin(np.deg2rad(ang)) + ecx
                ys = d1 / 2 * ct * np.sin(np.deg2rad(ang)) + d2 / 2 * st * np.cos(np.deg2rad(ang)) + ecy
                out.append({'label': 'barrel', 'points': np.stack([xs, ys], 1) + [x1, y1], 'how': 'matmouth', 'difficult': False})
    return out

def sam_lying_check(sam, img, box, ellipse_pts, major):
    """SAM 判侧倒：包含口沿中心的掩码域，延伸>=1.4x口沿长轴且面积<=3.2x major^2 -> 整段筒身多边形"""
    try:
        r = sam(img, bboxes=[[float(v) for v in box]], verbose=False)[0]
    except Exception:
        return None
    if r.masks is None:
        return None
    mk = cv2.resize(r.masks.data[0].cpu().numpy(), (img.shape[1], img.shape[0]),
                    interpolation=cv2.INTER_NEAREST) > 0.5
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mk.astype(np.uint8), 8)
    if n < 2:
        return None
    # 与口沿(膨胀)相交的所有 SAM 连通域取并集（SAM 掩码常把端面/筒壁打碎）
    em = np.zeros(img.shape[:2], np.uint8)
    cv2.fillPoly(em, [ellipse_pts.astype(np.int32)], 1)
    touch = cv2.dilate(em, np.ones((21, 21), np.uint8)) > 0
    U = np.zeros_like(mk, np.uint8)
    for i in range(1, n):
        if (lab == i)[touch].any():
            U[lab == i] = 1
    xs, ys = np.where(U > 0)
    if len(xs) == 0:
        return None
    ext = max(xs.max() - xs.min(), ys.max() - ys.min()) / max(1.0, major)
    if ext < 1.4 or len(xs) > 3.2 * major * major:
        return None  # 站立筒（SAM 抓了垫子）或无延伸
    # 并集 ∩ 白色掩码（切掉地面阴影尾巴）∪ 口沿
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    S, V = hsv[:, :, 1].astype(np.int32), hsv[:, :, 2].astype(np.int32)
    region = cv2.bitwise_or(cv2.bitwise_and(U, ((S < 125) & (V > 72)).astype(np.uint8)), em)
    cnts, _ = cv2.findContours(region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    c = max(cnts, key=cv2.contourArea)
    ap = cv2.approxPolyDP(c, 2.5, True).reshape(-1, 2)
    return ap.astype(np.float32) if len(ap) >= 6 else None

def sam_body(sam, img, box, keep_xy):
    """SAM box 提示取整段筒身：取包含 keep_xy 的连通域 -> 多边形"""
    try:
        r = sam(img, bboxes=[[float(v) for v in box]], verbose=False)[0]
    except Exception:
        return None
    if r.masks is None:
        return None
    mk = cv2.resize(r.masks.data[0].cpu().numpy(), (img.shape[1], img.shape[0]),
                    interpolation=cv2.INTER_NEAREST) > 0.5
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mk.astype(np.uint8), 8)
    if n < 2:
        return None
    kx, ky = int(keep_xy[0]), int(keep_xy[1])
    best = None
    for i in range(1, n):
        if lab[ky, kx] == i:
            best = i; break
    if best is None:  # 中心不在任何掩码里（罕见），退最大域
        best = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if stats[best, cv2.CC_STAT_AREA] < 1500:
        return None
    cnts, _ = cv2.findContours((lab == best).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    c = max(cnts, key=cv2.contourArea)
    ap = cv2.approxPolyDP(c, 2.5, True).reshape(-1, 2)
    return ap.astype(np.float32) if len(ap) >= 6 else None

def expand_box(box, img_shape):
    """SAM 提示框：贴图像边的框朝画内方向扩 1.5x 长边，另一轴 ±0.4x；不贴边的四边各扩 1.2x 长边"""
    H, W = img_shape[:2]
    x1, y1, x2, y2 = [float(v) for v in box]
    md = max(x2 - x1, y2 - y1)
    at_left, at_right = x1 <= 2, x2 >= W - 3
    at_top, at_bot = y1 <= 2, y2 >= H - 3
    if at_left or at_right or at_top or at_bot:
        # 贴边轴：离边方向扩 1.5x（筒身方向）；另一轴 ±0.4x
        x1 = 0.0 if at_left else max(0.0, x1 - (0.4 if (at_top or at_bot) else 1.5) * md)
        x2 = float(W) if at_right else min(W - 1.0, x2 + (0.4 if (at_top or at_bot) else 1.5) * md)
        y1 = 0.0 if at_top else max(0.0, y1 - (1.5 if (at_top or at_bot) else 0.4) * md)
        y2 = float(H) if at_bot else min(H - 1.0, y2 + (1.5 if (at_top or at_bot) else 0.4) * md)
    else:
        x1, y1 = max(0.0, x1 - 1.2 * md), max(0.0, y1 - 1.2 * md)
        x2, y2 = min(W - 1.0, x2 + 1.2 * md), min(H - 1.0, y2 + 1.2 * md)
    return [x1, y1, x2, y2]

def mat_mask_of(img):
    """蓝垫掩码（含垫上物体留下的洞，用 CLOSE 大核填合后膨胀）"""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    S, V, Hu = hsv[:, :, 1].astype(np.int32), hsv[:, :, 2].astype(np.int32), hsv[:, :, 0].astype(np.int32)
    blue = ((Hu > 95) & (Hu < 130) & (S > 70) & (V > 40)).astype(np.uint8)
    blue = cv2.morphologyEx(blue, cv2.MORPH_CLOSE, np.ones((75, 75), np.uint8))
    return cv2.dilate(blue, np.ones((13, 13), np.uint8))

def on_mat(shape_pts, mm, mm_near):
    """范围过滤：'on'=在垫上 / 'near'=垫边弱接触(标难) / 'off'=不在垫上(丢弃)"""
    msk = np.zeros(mm.shape, np.uint8)
    cv2.fillPoly(msk, [shape_pts.astype(np.int32)], 1)
    a = cv2.countNonZero(msk)
    if a == 0:
        return 'off'
    if cv2.countNonZero(cv2.bitwise_and(msk, mm)) >= max(400, 0.02 * a):
        return 'on'
    if cv2.countNonZero(cv2.bitwise_and(msk, mm_near)) >= 0.25 * a:
        return 'near'
    return 'off' 

def annotate_image(model, path, conf=0.5, min_white=0.35, visualize=True, sam=None):
    img = cv2.imread(path)
    if img is None:
        return None, None, []
    r = model(img, verbose=False, conf=conf)[0]
    mm = mat_mask_of(img)
    shapes, dbg = [], []
    if r.masks is not None:
        for poly, box, cf in zip(r.masks.xy, r.boxes.xyxy.cpu().numpy(), r.boxes.conf.cpu().numpy()):
            poly = np.array(poly, np.float32)
            m = np.zeros(img.shape[:2], np.uint8)
            cv2.fillPoly(m, [poly.astype(np.int32)], 1)
            wr = mask_whiteness(img, m)
            if wr < min_white:
                dbg.append(('rej-white%.2f' % wr, float(cf)))
                continue
            xs, ys = poly[:, 0], poly[:, 1]
            mbw, mbh = xs.max() - xs.min(), ys.max() - ys.min()  # 模型掩码长宽比
            if max(mbw, mbh) / max(1, min(mbw, mbh)) > 1.5 and min(mbw, mbh) > 40:
                # 侧倒/侧视：整段筒身（SAM box 提示优先，阈值法兜底）
                ap, how = None, 'body'
                if sam is not None:
                    ap = sam_body(sam, img, expand_box(box, img.shape), poly.mean(0))
                    if ap is not None:
                        how = 'sambody'
                if ap is None:
                    ap = whole_body_refine(img, box, seed_mask=m)
                if ap is None:
                    ap, how = refine_ellipse(img, poly); how = 'bodyfail-' + how
                shapes.append({'label': 'barrel', 'points': ap, 'conf': float(cf), 'how': how, 'difficult': True})
            else:
                pts, how = refine_ellipse(img, poly)
                ext = np.array(pts)
                maxdim = (ext.max(0) - ext.min(0)).max()
                if maxdim < 30:
                    dbg.append(('rej-small%.0f' % maxdim, float(cf))); continue
                diff = how != 'ellipse'  # 椭圆拟合失败的不规则形，留人工复核
                # SAM 侧倒探测：掩码域延伸 >=1.4 且面积合理(<=3.2x major^2) -> 整段筒身；
                # 站立筒 SAM 会把垫子一起抓进来（面积爆表），自动落回只标口沿
                if how == 'ellipse' and sam is not None:
                    ex_, ey_, ew_, eh_ = cv2.boundingRect(pts.astype(np.int32))
                    major_ = float(max(ew_, eh_))
                    md_ = max(ew_, eh_)
                    eb_ = [max(0, ex_ - 1.2 * md_), max(0, ey_ - 1.2 * md_),
                           min(img.shape[1] - 1, ex_ + ew_ + 1.2 * md_), min(img.shape[0] - 1, ey_ + eh_ + 1.2 * md_)]
                    body = sam_lying_check(sam, img, eb_, pts, major_)
                    if body is not None:
                        pts, how, diff = body, 'samlying', True
                shapes.append({'label': 'barrel', 'points': pts, 'conf': float(cf), 'how': how, 'difficult': diff})
            dbg.append((shapes[-1]['how'], float(cf)))
    for s in shapes:
        s['points'] = clip_poly_img(s['points'], img.shape[1], img.shape[0])
    for s in mat_detect(img, shapes):
        pts = s['points']
        if s['how'] == 'matbody' and sam is not None and 'sam_box' in s:
            sb = s['sam_box']
            ap = sam_body(sam, img, sb, [(sb[0] + sb[2]) / 2, (sb[1] + sb[3]) / 2])
            if ap is not None:
                pts = ap; s['how'] = 'sammatbody'
        shapes.append({'label': 'barrel', 'points': pts, 'conf': 0.0, 'how': s['how'], 'difficult': s['difficult']})
        dbg.append((s['how'], 0.0))
    mm_near = cv2.dilate(mm, np.ones((35, 35), np.uint8))
    kept = []
    for s in shapes:
        rel = on_mat(np.array(s['points']), mm, mm_near)
        if rel == 'off':
            dbg.append(('scope-off', 0.0)); continue
        if rel == 'near':
            s['difficult'] = True; s['how'] = 'near-' + s['how']
        kept.append(s)
    shapes = kept
    dbg.append(('mat-filter', len(shapes)))
    vis = None
    if visualize:
        vis = img.copy()
        for s in shapes:
            col = (0, 0, 255) if not s['difficult'] else (0, 140, 255)
            cv2.polylines(vis, [s['points'].astype(np.int32)], True, col, 2)
            x, y = s['points'].min(0)
            tag = 'barrel %.2f%s' % (s['conf'], ' *' if s['difficult'] else '')
            cv2.putText(vis, tag, (int(x), int(y) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)
    return shapes, vis, dbg

def clip_poly_img(pts, iw, ih):
    """多边形按图幅裁剪（Sutherland-Hodgman）：半出画筒出画边贴图像边，杜绝越界坐标"""
    x_edges = [(0.0, lambda p: p[0]), (float(iw), lambda p: iw - p[0])]
    y_edges = [(0.0, lambda p: p[1]), (float(ih), lambda p: ih - p[1])]
    poly = np.asarray(pts, np.float64)
    for limit, dist in x_edges + y_edges:
        if len(poly) == 0:
            break
        out = []
        d = np.array([dist(q) for q in poly])
        for i in range(len(poly)):
            j = (i + 1) % len(poly)
            pi, pj = poly[i], poly[j]
            di, dj = d[i], d[j]
            if di >= 0:
                out.append(pi)
            if (di >= 0) != (dj >= 0):
                t = di / (di - dj)
                out.append(pi + (pj - pi) * t)
        poly = np.array(out)
    return poly.astype(np.float32)

def to_xanylabeling(shapes, img_path, iw, ih):
    ss = []
    for s in shapes:
        ss.append({
            "label": s['label'], "score": None,
            "points": [[round(float(x), 2), round(float(y), 2)] for x, y in s['points']],
            "group_id": None, "description": "", "difficult": bool(s.get('difficult', False)),
            "shape_type": "polygon", "flags": {}, "attributes": {}, "kie_linking": []
        })
    return {
        "version": "4.0.6", "flags": {}, "checked": False, "shapes": ss,
        "imagePath": os.path.basename(img_path), "imageData": None,
        "imageHeight": ih, "imageWidth": iw, "description": ""
    }

def imwrite_u(path, img, q=92):
    ok, buf = cv2.imencode(os.path.splitext(path)[1], img, [cv2.IMWRITE_JPEG_QUALITY, q])
    if ok:
        with open(path, 'wb') as f:
            f.write(buf.tobytes())
    return ok

def run_batch(files, outdir, model=None, sam=None):
    from ultralytics import YOLO
    if model is None:
        model = YOLO(MODEL)
    os.makedirs(outdir, exist_ok=True)
    report = []
    for path in files:
        shapes, vis, dbg = annotate_image(model, path, sam=sam)
        if shapes is None:
            report.append((path, 'READ-FAIL', [])); continue
        base = os.path.splitext(os.path.basename(path))[0]
        if vis is not None:
            imwrite_u(os.path.join(outdir, 'annot_' + base + '.jpg'), vis)
        img = cv2.imread(path)
        j = to_xanylabeling(shapes, path, img.shape[1], img.shape[0])
        json.dump(j, open(os.path.join(outdir, base + '.json'), 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
        report.append((os.path.basename(path), [(s['how'], round(s['conf'], 2), 'D' if s['difficult'] else '') for s in shapes], dbg))
    return report

if __name__ == '__main__':
    args = [a for a in sys.argv[1:]]
    outdir = '_preview_annot'
    if args and not args[0].startswith('-'):
        outdir = args.pop(0) if os.path.isdir(args[0]) or True else outdir
    data_root = os.path.normpath(os.path.join(HERE, '..'))
    files = args if args else sorted(glob.glob(os.path.join(data_root, 'images', '*.jpg')))
    from ultralytics import YOLO, SAM
    model = YOLO(MODEL)
    sam = SAM(os.path.join(data_root, 'mobile_sam.pt'))
    for path, got, dbg in run_batch(files, outdir, model, sam):
        print(path, '->', got)
