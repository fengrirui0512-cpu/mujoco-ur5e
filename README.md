# mujoco-ur5e-control

基于 MuJoCo 的 UR5e 机械臂控制学习项目，逐步记录模型检查、单关节轨迹跟随及实验结果。
当前已验证肩部回转的最终保持读数；ROS 2、多关节规划和真机部署是后续计划，尚未实现。

## 环境与运行

本地已验证：Linux、Python 3.14.4、MuJoCo 3.17.0。
本机继续使用现有 `/home/fengrirui/mujoco-env` 环境，无须重新安装。

在本项目目录执行：

```bash
cd /home/fengrirui/mujoco-learning/08_robot
/home/fengrirui/mujoco-env/bin/python inspect_ur5e.py
/home/fengrirui/mujoco-env/bin/python move_shoulder.py
```

第一条静态查看；关闭其窗口后再运行第二条。关闭运动窗口可结束脚本。
其他电脑可以在自己的虚拟环境中根据 `requirements.txt` 安装依赖，随后使用
`python inspect_ur5e.py` 或 `python move_shoulder.py`；查看器需要可用的图形桌面和 OpenGL。
无图形界面时可给脚本加 `--headless`，只检查模型或计算 10 秒运动。

## 当前实验

- 从上游 `home` 姿态开始，前 2 秒保持，随后 4 秒将 `shoulder_pan` 目标增加 30°，然后保持。
- 其余五个执行器的目标不变；实际关节仍受到重力和动力学耦合影响。
- 轨迹采用半余弦插值，每步写入 `data.ctrl`，由模型内的伺服器产生力矩，调用 `mj_step` 推进仿真。
- 2026-11-17 用户运行截图：17.5～22 秒肩部目标、实际均显示 -60.00°，误差显示 0.00°。
- 上述数值为显示精度下的最终读数，不表示整个运动过程无误差或已完成真机验证。
- 运行和修改记录见 `EXPERIMENTS.md`。

## 来源与自己的工作

机器人结构和网格来自 Google DeepMind MuJoCo Menagerie；固定版本见 `MODEL_SOURCE.md`。
保留上游目录与许可证。当前中文教学脚本由助手协助编写和注释，用户亲手运行并反馈结果；
后续将通过 Git 提交记录实验修改与验证，不把上游模型列为自行建模成果。

## 自己运行的脚本

- `inspect_ur5e.py`：静态查看 home 姿态，打印六个关节的信息，不推进仿真。
- `move_shoulder.py`：推进仿真，只改变肩部回转目标；前 2 秒保持、随后 4 秒转动、之后保持。
- `MODEL_SOURCE.md`：记录模型的下载来源和固定版本，方便复现和后续发布。

使用 `/home/fengrirui/mujoco-env/bin/python` 执行脚本；脚本按自身位置查找模型资源。

## 上游模型文件：universal_robots_ur5e/

| 文件 | 用途 |
|---|---|
| `scene.xml` | 场景入口：引入 ur5e.xml，并添加地面、灯光和背景。Python 加载它。 |
| `ur5e.xml` | 机器人本体：连杆、质量惯量、关节、执行器、限制和 home 姿态。 |
| `assets/` | 外观网格资源，如 OBJ 文件；XML 会引用它们，不能随意删除或移动。 |
| `ur5e.png` | 上游提供的模型示意图，不参与物理仿真。 |
| `README.md` | 上游英文说明，介绍模型来源和转换过程。 |
| `CHANGELOG.md` | 上游模型的修改历史。 |
| `LICENSE` | 模型许可证；分享项目时需按许可证保留声明。 |

先关注 Python 脚本、scene.xml 和 ur5e.xml，网格文件暂时不需要逐个阅读。
上游模型的参数保持不变，scene.xml 和 ur5e.xml 已补充中文注释，文件名及资源目录结构沿用原版。自己的控制功能放在外层 Python 脚本中。

## 直接阅读带注释的实际文件

- `universal_robots_ur5e/scene.xml`：讲解场景入口、include、相机、灯光、纹理与地面。
- `universal_robots_ur5e/ur5e.xml`：讲解参数模板、连杆层级、局部坐标、关节、惯量、碰撞体、执行器及 home 姿态。

以上就是 Python 实际加载的模型文件，不需要替换或改名。重复的 annotated_models 文件夹已清理。
阅读顺序建议从 scene.xml 的 include 开始，再看 ur5e.xml 的 actuator 和 keyframe，后续按实验逐段学习。
后续代码和模型直接在实际使用的文件中补充详细中文注释，不单独备份无注释版本；不同实验的有效成果继续保留。
上游声明和许可证保留于 `universal_robots_ur5e/LICENSE`。
属性含义可查阅 [MuJoCo XML Reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html)。
