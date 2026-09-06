# ROBOTIS XM/H430 带 idler 参考模型

- 2026-09-06由官方手册 Drawings / X430 / x-430_idle.stp 入口取得：https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/
- 官方下载入口：https://www.robotis.com/service/download.php?no=158
- 入口返回HTML，页面脚本中的实际文件地址：https://www.dropbox.com/scl/fi/5h3gp05sd9kjgtma5nuw1/XM_H-430_idler.stp?rlkey=09db2f2uz3b9lkacbp31tsu6r&dl=1
- 下载的是STEP，不是把HTML存为模型。文件头标识 `XM_H-430_IDLER_ASM`。FreeCAD读取为18个有效实体。
- 原生包络：X[-14.25,14.25]，Y[-35.25,11.25]，Z[-19,23.7]mm，即28.5×46.5×42.7mm。带附件包络不能与本体标称28.5×46.5×34mm混同。
- 多个同轴圆柱表明参考轴线沿原生Z、X=Y=0；本轮以该轴线放置包络，沿轴零点不是已核准的输出面/螺纹深度基准。
- 这是XM/H430共用机械参考，不保证所有电气SKU和附件组合完全等同。不得拿82g电机本体质量代表带全部idler/螺钉的总成质量。附件须单列称重/查表。
- 本轮保存完整模型，并用其保守外接盒进行空间搜索；不将厂家模型的实体体积直接乘单一密度作为实际质量。
- 原始文件SHA-256及实际FreeCAD版本见 `09_cad/modules_v15/packaging_results.json`。只读取厂商数据，未执行下载页面的脚本。
