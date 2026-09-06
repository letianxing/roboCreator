# roboCreator 的 CAD / CAE / PLM / MES 工作流

用户于2026-09-06明确要求把CAE、PLM、MES纳入后续大步骤。项目仍处于设计研究；本目录先落实可追溯的本地工作流，不代表已部署企业PLM/MES服务。

## 职责与交付

| 领域 | 本项目用途 | 必须保存的产物 |
|---|---|---|
| CAD | 参数化几何、装配接口、关节/线束/外观空间 | FCStd、STEP、参数源、接口与装配关系、版本 |
| CAE | 验证受力、变形、接触冲击、热及多体动力学 | 载荷工况、材料与条件来源、求解输入/版本、网格/时间步收敛、结果、通过范围与实测关联 |
| PLM | 管理产品定义、版本、变更和发布 | 需求登记、零件身份/修订、EBOM、EBOM→MBOM映射、CAD/CAE关联、ECR/ECO、受控发布与实物配置 |
| MES | 记录制造执行与质量 | 工单、工序、打印/加工参数、材料批次、设备/操作者、检验、不合格/返工、装配/完工、序列号 |

CAE不只在CAD结束时做一次。每次改尺寸、材料、驱动或质量，都要判断哪些结构/热/动力学结果失效。已有v0.9–v0.11有限元属于特定历史局部研究，不能自动支持当前v0.18几何。

PLM也不是末尾单独一步，它将25个技术阶段串起来。MES对应14制造与15装配的实际执行，采购价格/订单仍关联13 BOM；不将MES等同于采购或ERP。

## 最小记录模型

`requirement_id → item_id@revision → CAD artifact@hash → CAE case@hash → release_id → work_order → operation → inspection → serial_number → calibration/model_version`

其中item_id是稳定的产品/装配/零件身份；`v0.18`是研究模型版本，不是每个零件的正式修订号。一个研究包可能含多件；在零件边界尚未定型时，登记为study，不虚构完整EBOM。

编号约定：`RC-REQ-####`需求、`RC-STUDY-####`研究、`RC-ASM-####`装配、`RC-PRT-####`零件、`RC-CAE-####`计算工况、`RC-ECR-####`变更请求、`RC-ECO-####`变更实施、`RC-REL-####`发布、`RC-WO-####`工单、`RC-NCR-####`不合格。实物序列号在装配领料/开工时分配，当前保持null。

生命周期：`draft → in_review → released → superseded/withdrawn`。研究检查点和哈希备份不会自动把记录推进到released。CAE有独立的 `not_run / numerical_check_passed / validated_for_scope / failed`，数值自洽不等于工程验证。

## 发布前的工作

1. 确定零件/装配边界，建立EBOM的稳定身份、数量和修订引用。
2. 对需求建立CAD/CAE/实测的证据映射；未满足项有明确处理决定。
3. 制造包绑定STEP/图纸/工艺/公差/材料/检验计划及哈希；MBOM可分解为可生产和可采购项。
4. 设计审核通过后形成不可变发布记录，才允许把MES工单从草稿变为可下发。
5. 执行中发现尺寸/装配问题，先记录NCR；影响设计时走ECR/ECO，保留原工单和原设计，不回写篡改已经制造的版本。

## 上海外包打印的首台样机

先建立工序模板：资料确认 → 材料/设备/方向确认 → 打印或机加工 → 外观与尺寸检验 → 不合格处理 → 装配 → 电气检查 → 标定。实际承接店、机型、材料批次、报价和检验值目前未知，因此模板不填实测结果。

PETG/PLA材料名不能替代牌号、干燥状态、打印方向和层间性能。工单应引用经设计发布的具体工艺，不能依据当前研究STEP直接安排生产。

MES完成品配置必须关联实际安装的电机、固件、传感器、材料/零件批次和序列号，后续Calibration、System Identification、Policy试验均引用该配置。

## 当前已实施的最小层

- `26_plm/registry.json`：登记本轮需求、研究对象与变更状态。
- `26_plm/artifact_manifest_v18.json`：由脚本生成的本轮文件哈希及来源关系。
- `11_verification/cae_plan_v18.json`：当前需要的CAE工况及尚缺的输入。
- `27_mes/work_order_template.json`：可复用但未下发的工单模板。
- `tools/validate_lifecycle.py`：检查本轮引用文件、哈希与未发布状态，不替代工程审核。

已有checkpoints继续用作恢复备份。权限、多人签审、供应商门户、实时设备采集、排产等尚未实现；若项目需要这些能力，再基于实际需求选择平台。
