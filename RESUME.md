# roboCreator 新终端续接入口

最后更新：2026-09-06。项目根目录：`G:\Golands\roboCreator`。当前分支：`main`，远端：`git@github.com:letianxing/roboCreator.git`。

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

> 请读取 `G:\Golands\roboCreator\RESUME.md`、`AGENTS.md`、`project_state.md` 和 `deliverables\pause_record_v23.md`，按已有证据继续，不重复已确认问题，不覆盖历史失败或未验证的用户改动。麦克风到货前保持暂停；到货后我会提供实物照片。

## 当前状态

- 用户要求暂停开发，因为已购买的Sipeed 6+1麦克风尚未到货。
- 最新完成验证阶段是v0.22：114–120mm身体参考高度的13个局部姿态和站姿，通过该轮几何、名义2mm器件间隙及竖直静态必要条件检查。完整起身、动态稳定、制造、结构CAE和真机均未通过。
- v0.23仅完成20个安装结构草案零件的参数化生成和原生CAD重开检查，包括电池托架/防脱、板卡/电源固定、简化紧固件、骨盆板修改、护线圈及线束空间探针；尚未装入整机或做碰撞、载荷、CAE、布线和制造验证。
- 最新已验证快照：`checkpoints/20260906_171548_806826_v0.23-paused/project.zip`，SHA-256 `a6c55b0a2cc1c84b143ce6b5358ab7b8d843f2fbe643eb5fd66795bf186b993b`。以`checkpoints/index.jsonl`为权威索引。
- v0.21–v0.23工程进度及本续接入口已推送为`808339e1429cd18b94fc9b0e91160468374301fd`；推送回执可能位于其后的提交。新终端始终以`git log -1 --oneline`和`git ls-remote origin refs/heads/main`核对最终最新值。
- 原型阶段PLM/MES后置；新增开源软件或MCP前必须先询问。现有FreeCAD FEM、Gmsh、CalculiX可继续使用，无需为当前暂停点安装或重启。

## 麦克风到货后的恢复顺序

1. 确认收到的实际套餐：圆形6+1阵列，以及是否包含MA-USB8或其他接口板。
2. 取得实物正面、背面、侧面和带尺照片；记录外径、厚度、安装孔径/孔距、连接器类型与出线方向。
3. 更新`08_electrical/sensor_inventory.md`及真实几何占位；不得用商品图尺寸冒充实测。
4. 继续v0.23：把托架、固定件、连接器和线束加入整机；核对新增/替换质量，复查v0.22局部运动区间与站姿。
5. 改善高位支撑余量并连接换掌、落脚、膝卸载至站立；之后才对对应承力结构运行FreeCAD/Gmsh/CalculiX CAE。

## 关键文件

- v0.22报告：`deliverables/layout_motion_review_v22.md`
- 暂停记录：`deliverables/pause_record_v23.md`
- v0.23参数/脚本：`03_system/electronics_carrier_v23.json`、`09_cad/carrier_v23.py`
- v0.23草案CAD：`09_cad/body_v23/carrier_components.FCStd`及STEP
- 当前冲突：`11_verification/conflict_register.md`
- 快照规则：`CHECKPOINT_POLICY.md`

## 工作区边界

历史v0.7/v0.8踝部CAD和自动备份存在额外工作区变更，已在暂停记录中标明为“保留但未验证”。本次用户要求提交当前全部进度，因此这些字节级变化会随当前工作区一起存档；不得据此宣称旧阶段设计重新通过。不要使用`git reset --hard`、`git checkout --`等方式清除它们。

恢复时先运行只读检查：`git status -sb`、`git log -3 --oneline`、`git lfs fsck`。若工作区已有新改动，视为用户工作并保留。每组验证通过后继续按`CHECKPOINT_POLICY.md`保存；失败和未运行项同样记录，不能把CAD可打开写成可制造或真机能力通过。
