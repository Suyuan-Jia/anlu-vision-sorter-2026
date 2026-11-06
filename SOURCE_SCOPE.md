# 来源范围与分类

来源版本：`unified_mode_osd_20261027`；来源快照：`P10_PENG_SERVO_20261028\baseline\anlogic_fpga`。

| 范围 | 处理 |
|---|---|
| `board_a_rgb_source/rtl/cnn/*` | 正式 A 工程实际引用的 CNN 检测、跟踪、ROI、后处理、OSD 和模型参数 |
| `board_b_cartoon_clean/rtl/*` | 正式 B 工程实际引用的区域/运动/视频显示 RTL |
| `split_common/*` | 正式 A/B 工程实际引用的板间传输与 HUD 模块 |
| A/B 顶层、`.al`、`.adc`、`.sdc` | 正式工程入口和约束 |
| `user_source/hdl_source/uics500_cfg/sc500cs_ae_*` | 正式 A 工程引用的 AE/传感器控制模块，未发现受限标记 |
| 其他保留 `user_source` 文件 | 正式工程引用且未发现加密/版权标记的集成功能候选；作者归属不以此文件单独证明 |

作者归属只依据现有工程和发布清单作工程范围判断，不等同于法律版权证明。
