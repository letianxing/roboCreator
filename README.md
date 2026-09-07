# roboCreator

用于保存机器人从需求、设计、制造到标定、仿真、训练和真机迭代的提示词与项目产物。

最新研究：[v0.28颈部轴系与线缆报告](deliverables/neck_review_v28.md) · [预览](09_cad/neck_hardware_v28_final/neck_review.png)。厂家几何与联动已建立，但线缆、组合姿态、顶部尺寸及实际支架仍未闭合，不是制造定型。用户已要求提交/推送本轮及此前未提交进度，实际结果以Git回执为准。

最新研究：[v0.27摄像头预选与视野/安装检查](deliverables/camera_review_v27.md) · [视野示意](09_cad/camera_v27/fov_comparison.png)。D436仅为优先工程候选，仍缺近距模式/实物/双平台和真实颈部联动验证，暂不采购；完整整机基线仍为v26。

最新：[v0.26 STEP修复报告](deliverables/step_repair_review_v26.md) · [当前整机FreeCAD](09_cad/step_repair_v26/repaired/stand.FCStd) · [已回读有效的整机STEP](09_cad/step_repair_v26/repaired/stand.step)。4姿态交换格式门已通过，旧失败保留；不代表制造/CAE或真实动作通过。

最新细化：[v0.25可拆托盘/线束固定报告](deliverables/uac_mount_review_v25.md) · [当前研究CAD](09_cad/uac_mount_v25_service/stand.FCStd) · [预览](09_cad/uac_mount_v25_service/mount_review.png)。参考结构已建并完成4姿态及离散维修空间检查；真实PCB/线材、紧固/材料/CAE与制造未放行。新增[摄像头FOV—任务—机械综合选型要求](08_electrical/camera_selection_requirements.md)已保存。

最新机械研究：[v0.24安装空间与走线报告](deliverables/uac_packaging_review_v24.md)，[研究CAD](09_cad/uac_space_v24/routed_stand.FCStd)，[预览](09_cad/uac_space_v24/space_review.png)。胸前横放及上方走线预留通过4姿态空间检查；实物板卡尺寸、安装支架、真实线缆及CAE仍未验证，不是制造模型。无需驱动板插在主机上继续本阶段。

## 开始使用

在新的对话中发送：

> 请读取 `G:\Golands\roboCreator\prompts\robot_master_prompt.md` 并按提示词执行。项目目录为 `G:\Golands\roboCreator`。先读取项目状态，每次只问我一个问题，已有足够信息的工作直接完成并保存产物。

主提示词：[robot_master_prompt.md](prompts/robot_master_prompt.md)。当前状态：[project_state.md](project_state.md)。

2026-09-07已恢复开发，MA-USB8照片与单板USB检查已归档。USB/COM7正常，音频Code10；已保存[描述符诊断](08_electrical/mausb8_usb_diagnosis.md)。阵列尚未接到驱动板，收音/定位未测；用户没有尺子，机械尺寸待核实。续接入口：[RESUME.md](RESUME.md)；[安装结构集成报告](deliverables/carrier_review_v23.md)；历史[暂停记录](deliverables/pause_record_v23.md)。

## 目录

- `prompts/`：主提示词。
- `00_inputs/`：用户概念图、需求、尺寸、数据手册及其他原始输入。
- `01_requirements/` 至 `25_iterations/`：与提示词 25 个阶段一一对应的过程文件与结果。
- `26_plm/`、`27_mes/`：贯穿设计/制造的本地产品追溯与制造执行模板，尚未部署企业平台。
- `tools/`：本项目工具、依赖和 MCP 安装记录。
- `deliverables/`：经过检查的阶段交付包和发布版本。

当前v0.23参考安装件已集成至整机，101个CAD部件对象、约4.125kg模型质量/额度，13个局部姿态及站姿通过本轮几何/竖直静态必要条件检查；高位余量和部分间隙仍小。UAC裸板尚未建模，完整起身、制造、结构CAE及真机验证仍待完成。优先复用FreeCAD FEM＋Gmsh＋CalculiX；PLM/MES后置；[新增软件/MCP先询问](tools/software_policy.md)。历史失败和原CAD保留，最新状态见 `project_state.md`。

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
