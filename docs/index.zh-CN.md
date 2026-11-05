[English](index.md) | 简体中文

# LiteArm VR 文档

本页汇总仓库总览、快速上手和组件级文档。组件文档是各自内部细节的权威来源。

## 从这里开始

- [仓库总览](../README.zh-CN.md)
- [上位机操作手册](host-application.zh-CN.md)
- [快速上手](../QUICKSTART.zh-CN.md)
- [从源码安装](setup.zh-CN.md)
- [打包与发布](../packaging/README.zh-CN.md)

## 组件

### 桥（bridge）

- [部署](../litearm-vr-bridge/docs/DEPLOY.md)
- [设计](../litearm-vr-bridge/docs/DESIGN.md)
- [跟随服务兼容性](../litearm-vr-bridge/docs/FOLLOW-SERVER-COMPAT.md)
- [真机接入](../litearm-vr-bridge/docs/REAL-ARM-BRINGUP.md)

### 遥操（teleop）

- [架构](../litearm-vr-teleop/teleop/docs/architecture.md)
- [开发](../litearm-vr-teleop/teleop/docs/development.md)
- [环境搭建](../litearm-vr-teleop/teleop/docs/environment-setup.md)
- [使用指南](../litearm-vr-teleop/teleop/docs/user-guide.md)
- [排障](../litearm-vr-teleop/teleop/docs/troubleshooting.md)
- [锚点跟踪](../litearm-vr-teleop/teleop/docs/anchored-tracking.md)

### 跟随（follow）

- [夹爪控制环](../litearm-vr-follow/docs/gripper-control-loop-rewrite.md)
- [夹爪抖动分析](../litearm-vr-follow/docs/gripper-jitter-analysis.md)

### CAN DKMS

- [vendored 驱动](../third_party/litearm-can-dkms/VENDORED.md)

## 项目约定

- [第三方声明](../THIRD_PARTY_NOTICES.md)
- [许可证](../LICENSE)
