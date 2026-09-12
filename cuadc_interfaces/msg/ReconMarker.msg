# 单个物理标识的判读结论（语义见 10_机载代码/接口契约.md §5）
# class_id 取值 = 附件11 的 10 类（0..9，与训练集 data.yaml 顺序一致，09-12 勘正）：
#   0 爆炸品 1 不燃气体 2 刺激性 3 放射性物品 4 腐蚀品
#   5 生物危害 6 遇湿易燃物品 7 有毒品 8 自燃物品 9 易燃
uint32 marker_index      # 本架次内稳定序号（按首次出现顺序，0 起）
int32 class_id           # 0..9；-1 = 拒识留空（留空 0 分，错填 -100，宁空勿错）
float32 confidence       # 融合窗口内该类置信度中位数 [0,1]
float32 margin           # top1-top2 中位数（类别竞争度）
uint32 frames            # 支持该结论的帧数
bool ambiguous           # true = 两类票数竞争被拒识（地面站须醒目提示转人眼备份）
string evidence_path     # 证据目录（机载相对路径，可为空串）
