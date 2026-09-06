# roboCreator

用于保存机器人从需求、设计、制造到标定、仿真、训练和真机迭代的提示词与项目产物。

## 开始使用

在新的对话中发送：

> 请读取 `G:\Golands\roboCreator\prompts\robot_master_prompt.md` 并按提示词执行。项目目录为 `G:\Golands\roboCreator`。先读取项目状态，每次只问我一个问题，已有足够信息的工作直接完成并保存产物。

主提示词：[robot_master_prompt.md](prompts/robot_master_prompt.md)。当前状态：[project_state.md](project_state.md)。

## 目录

- `prompts/`：主提示词。
- `00_inputs/`：用户概念图、需求、尺寸、数据手册及其他原始输入。
- `01_requirements/` 至 `25_iterations/`：与提示词 25 个阶段一一对应的过程文件与结果。
- `26_plm/`、`27_mes/`：贯穿设计/制造的本地产品追溯与制造执行模板，尚未部署企业平台。
- `tools/`：本项目工具、依赖和 MCP 安装记录。
- `deliverables/`：经过检查的阶段交付包和发布版本。

当前已推进到整机包装与流程v0.18。最新报告：[整机与接触评审](deliverables/body_review_v18.md)。电子器件重排已消除该立姿候选的所建电子相关重叠，肩臂/腿部仍有冲突。CAE/PLM/MES已纳入主流程；[新增软件/MCP先询问](tools/software_policy.md)。工程验证、设计发布和实际生产均未通过，历史证据保留，最新状态见 `project_state.md`。

检查点规则见 [CHECKPOINT_POLICY.md](CHECKPOINT_POLICY.md)，已验证全项目快照索引位于 `checkpoints/index.jsonl`。

## Git 与大文件恢复

目标仓库为 `git@github.com:letianxing/roboCreator.git`，主分支 `main`。历史检查点、CAD、图像及较大求解文件通过Git LFS保留，代码/说明/小型参数文件正常跟踪。仅忽略可再生成的Python缓存；空阶段目录用 `.gitkeep` 保留。

在已有正确GitHub身份及Git LFS的环境中：

```sh
git clone git@github.com:letianxing/roboCreator.git
cd roboCreator
git lfs pull
git lfs fsck
```

本机的SSH身份选择属于仓库本地配置，不随克隆复制；不会上传私钥。GitHub网页下载ZIP未必包含实际LFS对象，应使用Git＋LFS恢复。新增软件/MCP仍遵守先询问的项目规则。

本次范围与记录见 [全项目归档记录](deliverables/archive_and_push_record.md)。
