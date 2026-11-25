# UR5e 模型来源

- 上游：https://github.com/google-deepmind/mujoco_menagerie/tree/main/universal_robots_ur5e
- 下载版本：370c43965323c0944b433b202aac5058b5f907a9
- 准备日期：2026-11-16
- `universal_robots_ur5e/` 保留上游资源、说明和 BSD-3-Clause LICENSE。2026-11-17，用户将中文注释内容放入 scene.xml、ur5e.xml；已修正引用以沿用原文件名和目录结构，物理与显示参数未改。两份 XML 现为已加注释的衍生版本，并非逐字未修改的上游原文。
- `inspect_ur5e.py`、`move_shoulder.py` 为本学习目录新增的中文教学脚本，由助手协助实现，用户亲手运行验证；不属于上游模型原文件。
- 2026-11-17 按用户要求清理重复的 `annotated_models/` 学习副本；后续直接维护实际使用文件中的中文注释。模型版权和许可证仍保留于 `universal_robots_ur5e/LICENSE`。
