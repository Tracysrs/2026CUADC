# 台架 A/B 影子复测卡（v3.1 / v3.2 换模终判）

> 2026-10-01 备置。**这是 v3.1/v3.2 能否换掉生产 v2 的唯一终判**——09-28 v4 教训：
> 训练域高分 ≠ 机载可用，换模前必须台架靶标 A/B。全程走**影子 engine**（`~/v3_shadow/`），
> 生产 `~/cuadc_models/`（v2，best.engine SHA 964cd588…）零触碰；测完不过就维持 v2，账面零损失。

## 一、判据清单（全过才换模，任一败即维持 v2）

| # | 判据 | 通过线 |
|---|---|---|
| 1 | **新摆位橙「1.5」牌**（爆炸品，v2 色移病主诉） | shadow 判 **c0** 且 conf ≥0.8 **×6/6 帧**；同帧 v2 预期仍判易燃（复现病灶，A/B 意义所在） |
| 2 | **双牌同框**（橙 1.5 + 红易燃相邻） | 红橙**类别不互串**（各归各类，≥6 帧） |
| 3 | **12 目标扫测回归**（每牌 6 帧，v2 与 shadow 双 engine 同测） | v2 战绩=九中一失（唯一失=爆炸品）；shadow 须**爆炸品转正且其余九类不塌**（允许 ≤1 类单帧抖动，中位置信 ≥0.8 口径） |
| 4 | **空景负样本**（撤走全部牌）×24 帧 | shadow 最高置信 **<0.7**，零窗级误报 |
| 5 | **推理基准**（bench_trt.py 200 帧） | **≥25FPS**（v2 对照 56FPS） |

**拍板**：全过 → 执行「七、换模包」；任一败 → 生产维持 v2，败项数字入台账 + 总编，回去定补数据方向。

## 二、前置检查（到场先做，5 分钟）

- [ ] `ssh jetson` 通（hosts 已改 10.243.76.217；不通先解决网络）
- [ ] 生产态核对：`cat ~/cuadc_models/SHA256SUMS.txt` → best.engine = **964cd588…**（v2 现役）
- [ ] 磁盘 ≥2GB：`df -h ~`
- [ ] 台架道具十类牌 + 双牌摆位胶带；相机节点在跑（`nohup` 标准拉起，注意 pgrep 自匹配坑）

## 三、影子构建（每模型约 10 分钟）

```bash
# 本机（Windows）上传四件：
scp 01_视觉感知/数据与训练/deliver_v31/best.pt jetson:~/bench_ab/best_v31.pt
scp 01_视觉感知/数据与训练/deliver_v32/best.pt jetson:~/bench_ab/best_v32.pt
scp 03_机载软件/scripts/live_ab.py 03_机载软件/scripts/bench_ab_setup.sh jetson:~/bench_ab/

# Jetson 上（各约 10 分钟，v3.1/v3.2 各一次）：
cd ~/bench_ab
bash bench_ab_setup.sh ~/bench_ab/best_v31.pt     # → ~/v3_shadow/best_v31.engine，SHA 记台账
bash bench_ab_setup.sh ~/bench_ab/best_v32.pt     # → ~/v3_shadow/best_v32.engine
# 脚本自动把 ~/v3_shadow/best_v3.engine 软链到最新构建者；
# 换 A/B 对象：ln -sfn ~/v3_shadow/best_v31.engine ~/v3_shadow/best_v3.engine
```

记下打印的两个 SHA-256（回填台账用）。

## 四、同屏目检（live_ab，上屏=v2 生产 / 下屏=shadow）

```bash
nohup python3 ~/bench_ab/live_ab.py > /tmp/live_ab.log 2>&1 &
# 浏览器开 http://<jetson-ip>:8080
```

## 五、靶标 A/B（判据 1/2，用 live_ab 目检 + kacha 存证）

1. **新摆位单牌**：把橙 1.5 牌**换个与训练集不同的摆位/角度**放台架 → 采 6 帧
   （`~/kacha 6` 或 live_ab 截屏），记 shadow 类别与中位置信 ×6。
2. **双牌同框**：橙 1.5 + 红易燃相邻摆 → 6 帧，记两牌各自类别 ×6。
3. 每轮切换 v31/v32：`ln -sfn` 软链后 live_ab 无需重启（engine 路径不变）。

## 六、12 目标扫测 + 空景（判据 3/4）

- 逐牌报名 → `~/test_one_target.py` 采 6 帧双 engine 聚合（脚本在机 `~/`；
  跑前把对照 engine 指到 `~/v3_shadow/best_v3.engine`，v2 侧仍读生产件），
  **抓帧人眼验牌**防张冠李戴（09-28「有毒品」轮教训）。
- 空景：撤走全部牌采 24 帧，记 shadow 最高置信。

## 七、基准 + 收尾

- [ ] `python3 <scripts>/bench_trt.py ~/v3_shadow/best_v32.engine 200`（v31 同）→ ≥25FPS
- [ ] `pkill -f live_ab.py` 释放 GPU
- [ ] 数字填进下面的记录表 → 总编新节 + 台账双镜像
- [ ] **拍板执行**：过 → 「换模包」；败 → 维持 v2

## 记录表（现场填）

| 判据 | v2 | v3.1 | v3.2 | 判定 |
|---|---|---|---|---|
| 1.5 牌判 0 conf（×6） | 易燃 0.9x（病灶） | | | |
| 双牌互串 | — | | | |
| 12 目标扫测 | 九中一失 | | | |
| 空景最高置信（24 帧） | 0.4~0.6 级误报被门控兜住 | | | |
| FPS（200 帧） | 56 | | | |

## 八、换模包（判据全过后才执行，一次一批）

1. `bash deploy_trt_jetson.sh ~/bench_ab/best_v32.pt recon` → 生产槽位重建 + bench 基准；
2. **SHA 三处回填**：打印的 SHA 回 `cuadc_perception/config/hazard_recon_params.yaml`
   （机上 src/install 两份 + 仓内同文件），重启 `cuadc-perception`；
3. 融合开关默认翻转（**外推系 + min_frames 4 + imgsz 832** 配方，832 需按训练强绑定原则
   另建 832 engine）+ SSOT §4.4 同步 + 「默认关等同旧版」回归测试；
4. SHA256SUMS / 台账双镜像 / 总编收尾；**旧 v2 三件先 archive 再删**（09-28 v4 归档同款流程）。

## 红线

- **全程不写 `~/cuadc_models/`**——影子只进 `~/v3_shadow/`；生产 best.engine 谁都不许 mv；
- 比赛日**绝不可现场构建 engine**（10 分钟构建 + 首帧冷启动秒级，上场前必须黑帧预热）；
- 测完影子件留 `~/v3_shadow/` 不删（回归抽查还要用），生产维持 v2 直到判据全过。
