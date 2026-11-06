# 安路 FPGA 正式参赛源码公开候选

本目录以正式软件发布 `unified_mode_osd_20261027` 为来源，来源快照为本机正式软件发布副本（`unified_mode_osd_20261027`）。

依据：`evidence/peng_original_reports/FINAL_RELEASE.json` 标记该版本为 `FINAL_SOFTWARE_VERIFIED_PENDING_HARDWARE`，A/B bit 身份为 A `53448c26...`、B `60ec3203...`。该记录明确表示软件验证完成、硬件验收未完成，因此本候选不宣称最终 bit 已完成实板验收。

## 工程入口

- A：`board_a_rgb_source/board_a_rgb_source.al`
- B：`board_b_cartoon_clean/board_b_cartoon_clean.al`
- 器件：PH1P35MDG324-3；TD 版本记录为 6.2.168116。

## 保留内容

- A 板 CNN 检测、目标位置跟踪、ROI、后处理和 OSD RTL；
- B 板区域/运动显示和视频链路 RTL；
- A/B 顶层、ADC/SDC 约束、板间打包/接收模块；
- 正式发布中实际引用的部分图像处理和 AE 控制模块；
- CNN 检测、跟踪和后处理 RTL；模型权重、偏置和量化参数因缺少独立再分发授权证据未公开。

## 构建限制

这是公开源码候选，不是可独立完整构建包。正式工程依赖的安路加密 MIPI/HDMI/DDR HDL、IP Generator 配置、模型权重和部分 MiLianKe/官方基础模块未复制到本目录。需要从安路官方 TD/IP Generator 环境及有授权的原工程取得这些依赖，再按 `.al` 中的相对路径恢复后构建。

本候选没有包含 C10、P06、P08 实验候选、Runs、数据库、日志、缓存、bitstream、视频或调试材料。

## 功能状态

CNN 检测、`cnn_live_tracker.v` 跟踪、OSD、ISP、板间视频和 B 板运动/区域显示均存在于正式软件发布输入清单，并有离线软件/构建记录。该发布记录的 `hardware_tested` 为 false；真实摄像头识别精度、自动跟随、机械方向和实板闭环不能由本目录宣称通过。

正式 A/B 清单去重后有 93 个工程输入；本候选保留其中 50 个可公开源码/配置引用，43 个因加密、生成或第三方授权不明而排除。完整排除范围见 `EXCLUDED_SCOPE.csv`。\n\n文件来源、分类和 SHA256 见 `SOURCE_SCOPE.md`、`THIRD_PARTY_DEPENDENCIES.md`、`SOURCE_MANIFEST.csv` 和 `SHA256SUMS.txt`。


