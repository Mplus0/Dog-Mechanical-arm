# Legacy archive

本目录保存 `red_block_grasp_mplus0` 建立时继承的旧实现，供新流程开发期间对照、回退和行为核验。

- `action_flow_v1/`：原抓取、放置、任务通信和机械臂执行流程快照。
- 归档文件不属于 Python 包，不由 `setup.py` 安装，也不是正式运行入口。
- 不应直接在归档副本中开发新功能或修改现场参数。
- 新实现应写入正式的 `red_block_grasp_mplus0/`、`launch/` 和 `config/` 目录。

在新动作流程完成 RDK X5 构建、分阶段真机验证和回退验证以前，不删除此档案。
