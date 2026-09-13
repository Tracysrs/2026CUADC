# 侦察判读融合输出（AI 判读；不在飞控关键路径上，组员据此填单）
std_msgs/Header header           # stamp = 融合窗口末帧的取帧时刻（不是发布时刻）
uint32 viewpoint_seq             # 对应 /cuadc/recon/capture_request 的 point.x 序号
ReconMarker[] markers            # 本窗口结论，可空数组 = 本窗口无任何结论
