"""第一步：加载 UR5e，打印关节信息，并静态查看 home 姿态。

运行：/home/fengrirui/mujoco-env/bin/python 08_robot/inspect_ur5e.py
加 --headless 只打印信息；关闭查看器结束。此步不推进动力学仿真。
"""
# 阅读顺序：解析启动参数 → 加载模型 → 恢复 home → 打印信息 → 静态查看。
# Path 负责文件路径；argparse 读取命令行参数；math 换算角度；time 控制刷新间隔。
from pathlib import Path
import argparse
import math
import time
import mujoco

# __doc__ 是文件顶部的说明文字，运行脚本加 --help 可以查看。
# store_true 表示：带 --headless 时为 True，不带时为 False。
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--headless", action="store_true", help="仅检查模型，不打开窗口")
args = parser.parse_args()

# scene.xml 包含机器人、地面和灯光；相对路径根据脚本位置解析。
# __file__ 是当前脚本文件；resolve() 转绝对路径；parent 取所在文件夹。
# / 在 Path 对象之间表示拼接路径，因此从其他目录启动也能找到模型。
path = Path(__file__).resolve().parent / "universal_robots_ur5e" / "scene.xml"
# MjModel 保存模型结构、质量、关节限制等；MjData 保存某一时刻的状态和控制。
# from_xml_path 会解析 scene.xml 及其 include 引用，并加载所需网格资源。
model = mujoco.MjModel.from_xml_path(str(path))
data = mujoco.MjData(model)

# keyframe 是模型作者保存的姿态快照。home 同时设置关节位置和控制输入。
# 使用名字查找，而不假定 home 一定是第 0 个关键帧。
home_id = model.key("home").id
mujoco.mj_resetDataKeyframe(model, data, home_id)
mujoco.mj_forward(model, data)  # 根据关节角计算各连杆位置，不增加仿真时间。

# 本模型的六个转动关节各提供一个自由度，所以 njnt 和 nv 都为 6。
# 其他模型可能有球关节、自由关节，关节数与自由度数不一定相等。
print(f"关节数 njnt={model.njnt}，速度自由度 nv={model.nv}，执行器数 nu={model.nu}")
for joint_id in range(model.njnt):
    joint = model.joint(joint_id)
    # qposadr 是此关节的位置在 qpos 数组中的起点；不能泛化为与关节 ID 相同。
    # 本模型全是 hinge 转动关节，每个关节占一个位置值，单位 rad。
    angle = math.degrees(data.qpos[model.jnt_qposadr[joint_id]])
    # jnt_range 中的两个值为下限、上限，转为度方便人阅读。
    # jnt_limited 指示是否启用限制；未启用时不能把这两个数当作有效边界。
    low, high = [math.degrees(v) for v in model.jnt_range[joint_id]]
    limit = f"[{low:.1f}°, {high:.1f}°]" if model.jnt_limited[joint_id] else "未启用"
    # :22s 将名称补齐到 22 字符；:7.1f 将角度按宽度 7、保留一位小数输出。
    print(f"{joint_id}: {joint.name:22s} home角度={angle:7.1f}°，关节限制={limit}")

# jnt_range 描述关节范围；执行器的 ctrlrange、forcerange 是另两类限制，
# 后续控制时再区分。本步不修改任何参数，也不调用 mj_step。
if not args.headless:
    # launch_passive 创建被动查看器，仿真是否前进由 Python 决定。
    # with 在退出时释放窗口资源；is_running() 在关闭窗口后变为 False。
    import mujoco.viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            viewer.sync()  # 保持窗口响应，支持旋转视角和缩放。
            time.sleep(0.02)  # 静态查看无需高速刷新，减小 CPU 占用。
