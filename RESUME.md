# roboCreator 新终端续接入口

最后更新：2026-09-08。项目根目录：`G:\Golands\roboCreator`。当前分支：`main`，远端：`git@github.com:letianxing/roboCreator.git`。

Git续接：工程提交`af665c73225fa888676404e10983ed03a2d699cd`已成功推送，远端main已核对一致；211个LFS对象约8GB完成上传。回执`deliverables/git_push_receipt_20260908_v28.json`及本页随后单独提交，最新回执提交号以`git log`/`git ls-remote`为准。新终端继续v28未通过项，不重做已完成研究。

最新v28：先读`deliverables/neck_review_v28.md`、`03_system/neck_interfaces_v28.json`和`11_verification/neck_v28_checks.json`。厂家X330几何和两轴链已建立，27裸几何姿态17通过、中央低头0–50°采样通过；两条恒长线缆均失败，顶部占位全区间峰值401.036mm。不能把文件夹`neck_hardware_v28_final`当制造定型。下一步承力支架/真实装配相位、分段线缆、顶部尺寸与耦合安全路径。用户已要求本轮保存后提交推送全部进度，实际结果查推送回执/Git远端。

最新研究v27：先读`deliverables/camera_review_v27.md`。D436仅为优先工程候选，Gemini335为紧凑备选，未采购/冻结。48站姿名义壳体场景15通过/33碰撞；当前相机俯视需要真实颈轴与头甲联动，不能将绕相机中心的试探旋转当关节ROM。D436质量/功耗/图纸/近距模式和M4支持仍待证。下一步颈部—头甲—相机—线缆联合设计，采购暂缓。研究CAD在`09_cad/camera_v27/`；完整整机基线仍是下方v26。

最新入口（v26优先）：`deliverables/step_repair_review_v26.md`及`09_cad/step_repair_v26/repaired/stand.FCStd`/同名STEP。四个姿态全部118对象/362实体，修订STEP回读和8根连杆详细BOP检查通过；仅重建原尺寸RollRod，其余116对象保留。旧v25 STEP错误作为历史保留，当前交换格式门已通过，制造/CAE/实物动作仍未放行。

最新机械入口：`deliverables/uac_mount_review_v25.md`、`09_cad/uac_mount_v25_service/stand.FCStd`。v25已补参考可拆托盘、框架、软衬绑带和接口/工具避让；净增CAD假设质量19.998g，模型/额度约4.145233kg。4姿态配对、4工具杆和16站姿拆卸采样通过局部几何检查，不是实物安装/CAE/制造通过。真实板卡/线材/预紧仍缺，下一步继续实际接口、USB防拉脱及载荷。新增摄像头FOV/任务/机械协同选型要求见`08_electrical/camera_selection_requirements.md`，不得把原相机包络当选定产品。无需插USB。

历史导出注意：v25整机STEP回读错误见旧检查文件；不要选错旧文件。v26在独立目录修复并通过回读，见`11_verification/step_repair_v26_checks.json`，仍不是制造放行。上游重建后须执行v26修复脚本与检查，不覆盖旧失败文件。

最新信息优先：用户已连接MA-USB8单板USB，麦克风阵列未接。USB/COM7正常枚举，但MicArray音频Code10；原始描述符已确认异步OUT缺少Windows要求的显式反馈端点。详见`08_electrical/mausb8_usb_diagnosis.md`。用户没有尺子，机械尺寸仍待核实；不重复询问到货/型号或要求再次插USB。

## 新终端如何开始

在新终端进入仓库后，依次读取：

1. `RESUME.md`（本文件）
2. `AGENTS.md`（持续协作与验证规则）
3. `project_state.md`（最新状态优先于历史条目）
4. `prompts/robot_master_prompt.md`（完整25阶段流程）
5. `deliverables/pause_record_v23.md`（当前暂停点）

若是新克隆：

```powershell
git clone git@github.com:letianxing/roboCreator.git
Set-Location roboCreator
git lfs pull
git lfs fsck
git status -sb
```

本机仓库的SSH身份选择是本地配置，不会随Git仓库复制；不得读取或输出私钥。GitHub网页ZIP可能不包含LFS实际内容，应使用Git和Git LFS。

对AI说：

> 请读取 `G:\Golands\roboCreator\RESUME.md`、`AGENTS.md`、`project_state.md` 和最新阶段报告继续。MA-USB8单板已连接、阵列未接，COM7正常但音频Code10，描述符诊断已保存。用户没有尺子，不重复询问已知信息，不覆盖历史失败或用户改动；先核对兼容性方案再考虑固件变更。

## 当前状态

- 2026-09-07已恢复开发且麦克风到货；2026-09-06的暂停已经结束。
- 历史v0.22：114–120mm身体参考高度的13个局部姿态和站姿，通过该轮几何、名义2mm器件间隙及竖直静态必要条件检查。后续v23/v24见本页最新机械入口；完整起身、动态稳定、制造、结构CAE和真机均未通过。
- v0.23已完成参考安装结构的整机集成修订：101个部件对象、约4.125235kg模型质量/额度，13个局部姿态和站姿通过本轮几何/竖直静态必要条件检查。当前CAD在`09_cad/body_v23_fit/`，报告`deliverables/carrier_review_v23.md`。未包括新UAC裸板，真实接口、预紧/强度及完整起身仍待验证。
- 2026-09-06历史暂停快照为`checkpoints/20260906_171548_806826_v0.23-paused/project.zip`，SHA-256 `a6c55b0a2cc1c84b143ce6b5358ab7b8d843f2fbe643eb5fd66795bf186b993b`。最新已验证快照请读`checkpoints/index.jsonl`最后一条；新增进度不能用历史暂停快照代替。
- v0.21–v0.23暂停前工程提交为`808339ed62dead917a43b67eab5005aa316bfd28`；推送回执提交为`9f0af22c30edbf641734c05e82d07831fc9ddda2`。2026-09-07通过本地Git校正了此前文档中抄错的工程提交完整哈希。新终端始终以Git日志核对最新版本。
- 原型阶段PLM/MES后置；新增开源软件或MCP前必须先询问。现有FreeCAD FEM、Gmsh、CalculiX可继续使用，无需为当前暂停点安装或重启。

## 当前续接顺序

以v28报告为首：优先解决线缆、支架与顶部占位，并对耦合动作继续验证；不要再次仅旋转相机，或将中央俯仰通过扩展到任意转头俯仰组合。70/38mm曲线只是假设自由端弯曲段，未包含完整USB引出/扭转与固定。v27的D436/M4和实物资料门槛继续有效。

先读v27相机报告/决策/检查文件，再继续定义真实两轴颈部与头甲/相机刚性关系、光心和线缆。当前X40/俯45°包络通过不能跳过中间30°失败，也不能将D436848模式195mm推断当实测。M4直连功能未验证；新增SDK/工具仍先询问。其他实物UAC与整机验证门槛继续保留。

0. 先读v26 STEP修复报告/检查，再读v25结构报告。当前118对象整机FCStd及有效STEP在`09_cad/step_repair_v26/repaired/`。托盘/框架是参考件，PCB适配仍未完成；R6/Ø4/217.699mm仍是线束代理。下一步实物接口、USB防拉脱、螺母防掉落/预紧、材料/载荷或摄像头联合选型。上游CAD变化须重跑v26后处理及验证，不复用已过期证据。

1. 单板USB检查已经完成。先读`08_electrical/mausb8_usb_diagnosis.md`：Windows音频初始化失败，COM7正常。后续核对厂商支持的USB音频兼容方案和阵列版本；需要变更固件时先形成具体方案，当前没有刷写授权或已验证镜像适配结论。
2. 尺寸及质量留待机械图或后续实测，用户目前没有尺子；旧阵列PDF与实际版本未对应，不能直接照旧孔距制造。
3. 更新真实麦克风及独立UAC驱动板模型、FPC/USB与质量；v24只增加独立空间预留和质量敏感性，并未完成真实UAC板与安装件建模。
4. 使用`09_cad/body_v23_fit/`安装结构回归局部姿态和站姿，再落实实物配合/预紧及CAE。最初暂停草案与失败集成均保留为对照。
5. 改善高位支撑余量并连接换掌、落脚、膝卸载至站立；之后才对对应承力结构运行FreeCAD/Gmsh/CalculiX CAE。

## 关键文件

- v0.22报告：`deliverables/layout_motion_review_v22.md`
- 当前v0.23报告：`deliverables/carrier_review_v23.md`
- 实物接收记录：`08_electrical/sipeed_arrival_review.md`
- 暂停记录：`deliverables/pause_record_v23.md`
- v0.23参数/脚本：`03_system/electronics_carrier_v23.json`、`09_cad/carrier_v23.py`
- v0.23草案CAD：`09_cad/body_v23/carrier_components.FCStd`及STEP
- 当前冲突：`11_verification/conflict_register.md`
- 快照规则：`CHECKPOINT_POLICY.md`

## 工作区边界

历史v0.7/v0.8踝部CAD和自动备份存在额外工作区变更，已在暂停记录中标明为“保留但未验证”。本次用户要求提交当前全部进度，因此这些字节级变化会随当前工作区一起存档；不得据此宣称旧阶段设计重新通过。不要使用`git reset --hard`、`git checkout --`等方式清除它们。

恢复时先运行只读检查：`git status -sb`、`git log -3 --oneline`、`git lfs fsck`。若工作区已有新改动，视为用户工作并保留。每组验证通过后继续按`CHECKPOINT_POLICY.md`保存；失败和未运行项同样记录，不能把CAD可打开写成可制造或真机能力通过。
