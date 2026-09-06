# 上位机与训练资源

## 当前 Windows 主机：本轮只读检测

- 系统：Windows 11 专业版，64 位，版本 10.0.26200。
- CPU：Intel Core i7-14700KF，20 核、28 逻辑处理器。
- 内存：系统报告约 31.8 GiB。
- GPU：NVIDIA GeForce RTX 4070 Ti SUPER。
- 显存：nvidia-smi 报告 16,376 MiB，约 16 GiB。
- NVIDIA 驱动：610.88。
- 采集方式：Windows CIM 与 nvidia-smi。未进行推理性能、相机兼容或仿真基准测试。

## M4 Mac：用户新增支持要求

- 要求支持作为上位机运行机器人控制、交互及可部署的感知/决策模型。
- 用户确认统一内存为 48 GB；具体 Mac 型号、M4/M4 Pro/M4 Max 版本及 macOS 版本尚未知，部署时检测，不从内存容量推断芯片型号。
- 共同的通信协议、命令与状态数据格式应与 Windows 一致。
- 推理后端按模型与平台分别验证；不能将 Windows CUDA 依赖直接视为 macOS 可用，也不承诺两端相同性能。
- RealSense 的采集优先由机器人侧完成，上位机接收数据；直接 USB 接 Mac 的驱动兼容是另一项待验证需求。
- 当前“支持”是设计目标，尚未开发或测试。云端训练与 Mac 上位机支持分开，不推断需要在 Mac 上运行 Isaac Sim/Isaac Lab 训练。

## 云端训练

用户计划 AutoDL 租卡，优先 RTX 4090；详见 `20_training/compute_plan.md`。未开启实例。
