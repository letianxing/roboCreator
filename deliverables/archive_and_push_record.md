# roboCreator 全项目归档与 Git 推送记录

完成状态：2026-09-06已将全部归档内容推送至目标仓库main。首次内容提交 `7dcfe7bcf639c111351cd9849e0f26b0df8a349b` 已与远端main逐字核对一致，736个项目文件全部纳入，296个LFS路径对应288个唯一大文件对象（约4.95GB）。本记录及机器可读回执随后单独提交。

回执：`deliverables/git_push_receipt_20260906.json`。首次上传期间GitHub关闭了Git的SSH连接，但LFS对象上传全部完成；添加本仓库SSH保活设置后重试，复用远端对象并成功创建main及upstream。`git lfs fsck`、`git fsck --full`均通过。

日期：2026-09-06。用户明确要求保存、记录当前全部项目内容，并推送到 `git@github.com:letianxing/roboCreator.git` 的 `main` 分支。

## 保存范围

- 主提示词、需求、项目状态、工具使用规则和后续计划。
- v0.1至v0.18研究过程中的参数、脚本、CAD/STEP/STL、预览、运动学/动力学结果、有限元网格和求解日志、失败/待验证记录。
- CAE计划、本地PLM登记与哈希、MES草稿模板。
- 现有全部历史检查点ZIP、manifest和校验记录；这些快照不是制造发布。
- 本轮新增归档清单和Git/LFS配置。

仅省略可再生成的Python字节码/`__pycache__`和Git自身的`.git`内部目录。所有CAD备份、仿真日志、输入及历史快照均保留。`.gitignore`没有排除项目成果目录。

## 大文件策略

初次盘点约4.519GB，其中历史检查点约3.189GB；本轮归档会继续增加文件。Git LFS 3.6.1已经安装，本轮只做仓库本地配置，不安装新软件或新增MCP。

CAD、图像、网格/求解大文件、render_mesh和ZIP快照通过Git LFS上传。普通Git保存代码、说明和结构化小文件；`.gitattributes`关闭自动换行转换，以保留原文件哈希。

GitHub普通Git拒绝超过100MiB的文件；这些历史ZIP约347–488MB，需要LFS。依据：[GitHub大文件规则](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github)、[Git LFS说明](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-git-large-file-storage)。未购买额外配额，也不会自动修改计费设置。

克隆后需在有Git LFS的环境运行 `git lfs pull` 获取实际大文件；仓库网页中的LFS指针本身不是完整CAD/ZIP。GitHub网页“Download ZIP”是否带LFS对象取决于仓库设置，不应作为唯一恢复方式。

## 当前远程诊断

指定地址保持 `git@github.com:letianxing/roboCreator.git`。首次默认身份为 `letianxing1994`，读仓库返回 `Repository not found`。用户确认目标账号为 `letianxing`；检查已有SSH配置后找到对应身份，认证已成功，目标仓库可访问且尚无refs。

Git只在本项目 `.git/ssh_config` 与 `core.sshCommand` 中选择正确的已有身份；私钥位于项目外，不读取或输出其内容，也不随仓库上传。未修改全局SSH配置。G盘不记录文件所有者，Git已针对本项目绝对路径增加safe.directory例外，没有放开其他目录。

准备阶段记录：最初远程推送尚未完成；现在已由推送响应和远端main的commit核对确认成功，详见顶部与回执。未新建仓库、未修改仓库可见性。

归档前检查覆盖719个项目文件和4335个ZIP成员，8份历史快照的SHA-256与其已验证索引一致。已配置的已知私钥/令牌特征未命中；此扫描不保证覆盖所有可能的凭据格式。完整清单在 `11_verification/git_archive_inventory.json`，清单之后新增的本次快照、占位文件及推送回执同样会纳入Git提交。

## 当前工程状态

最新研究版本为v0.18：整机肩臂/腿部仍有干涉，当前CAE、设计发布及实际制造未通过。下一步是处理肩臂空间和膝接触结构，形成可连接的起身阶段；不是直接安排制造。

用户新增规则持续有效：引入开源软件或新增/配置MCP前先说明并询问。项目当前已有Git和Git LFS，使用它们完成本次明确授权的提交推送。
