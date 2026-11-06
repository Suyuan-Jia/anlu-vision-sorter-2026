# 未纳入公开副本的必要依赖

以下依赖在正式 `.al` 工程中被引用，但本轮不复制其本体：

- `user_source/hdl_source/mipi_dphy_rx/`：安路加密 MIPI HDL；
- `user_source/hdl_source/ph1p35_ddr/`：安路 DDR/IP Generator 生成内容；
- `user_source/ip_source/`：安路 IP 配置、生成 RTL 和工具数据库；
- 含 `pragma protect`、Anlogic 加密标记或 MiLianKe 版权/学习用途声明的文件；
- 未能取得独立再分发授权的官方例程和生成包装器。

获取方式：使用安路官方 TD/IP Generator 和有授权的原工程依赖，在本地恢复到 `.al` 所需相对路径。公开候选因此不能单独完成完整 TD 构建。

- `board_a_rgb_source/rtl/cnn/generated/`：正式 CNN 使用的权重、偏置和量化参数；来源可追溯到正式软件发布，但未找到独立公开再分发授权，本候选不包含这些文件。
