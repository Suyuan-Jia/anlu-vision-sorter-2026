# 基于安路 PH1P35 的实时数字图像处理系统

这是安路 FPGA 参赛作品的正式公开源码。系统运行在 PH1P35MDG324-3 器件上，由 A、B 两块 FPGA 协同完成实时图像采集、处理、检测、跟踪和显示。

## 作品功能

- MIPI 摄像头采集、RAW 图像处理、ISP 和 AWB；
- CNN 目标检测、ROI 处理、目标位置跟踪和 OSD 叠加；
- 边缘、区域和运动显示；
- A/B 板间视频传输、DDR 帧缓存和 HDMI 输出；
- 安路 Logo、状态信息和实时画面显示。

## 工程入口

- A 板工程：`board_a_rgb_source/board_a_rgb_source.al`
- B 板工程：`board_b_cartoon_clean/board_b_cartoon_clean.al`
- 器件：PH1P35MDG324-3
- TD：6.2.168116

## 公开范围

仓库包含正式作品中可公开的 RTL、顶层工程文件、ADC/SDC 约束、板间链路模块和 AE 控制模块。CNN 权重、偏置和量化参数因缺少独立再分发授权证据未公开。

## 构建限制

工程不能仅凭本仓库独立完成 TD 构建。正式工程依赖的安路加密 MIPI、HDMI 和 DDR HDL、IP Generator 配置、模型参数以及部分官方或第三方基础模块未包含在仓库中。构建时需要从安路官方 TD/IP Generator 环境和有授权的原工程获取这些依赖，并恢复到 `.al` 文件要求的相对路径。

## 验证状态

CNN 检测、目标跟踪、OSD、ISP、板间视频和 B 板运动/区域显示均已完成软件或离线构建验证。正式发布记录尚未完成最终实板验收，因此真实摄像头识别精度、自动跟随、机械方向和整板闭环仍需在硬件上确认。

仓库不包含实验分支、Runs、数据库、日志、缓存、bitstream、视频或调试材料。文件来源、排除范围和 SHA256 清单见 `SOURCE_SCOPE.md`、`THIRD_PARTY_DEPENDENCIES.md`、`EXCLUDED_SCOPE.csv`、`SOURCE_MANIFEST.csv` 和 `SHA256SUMS.txt`。
