"""让 UR5e 肩部回转关节从 home 的约 -90° 平滑转到约 -60°。

运行：/home/fengrirui/mujoco-env/bin/python 08_robot/move_shoulder.py
关闭窗口结束；--headless 无窗口计算 10 秒，供检查使用。
原模型不修改，其余五个执行器始终接收 home 目标。
"""
# 整体流程：加载 home → 找关节和执行器 → 检查目标范围 → 生成轨迹 → 循环仿真。
# 本文件控制的是目标角度，不直接改写运动中的实际关节位置。
from pathlib import Path
import argparse
import math
import time
import mujoco

# argparse 读取启动选项；__doc__ 让顶部说明显示在 --help 帮助中。
# action="store_true" 表示仅在命令带 --headless 时启用无窗口模式。
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--headless", action="store_true", help="不打开窗口，计算 10 秒")
args = parser.parse_args()
# 路径从脚本所在文件夹计算，避免终端当前目录不同导致找不到模型。
path = Path(__file__).resolve().parent / "universal_robots_ur5e" / "scene.xml"
# model 保存结构和物理参数，data 保存实际位置、速度、时间和控制输入。
# home 是上游预设的 keyframe，恢复时同时初始化 qpos、qvel 和 ctrl 等数据。
model = mujoco.MjModel.from_xml_path(str(path))
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)

# 关节名称带 _joint，执行器名称不带；二者分别负责状态和控制输入。
# 使用名字查询，避免依赖“第一个关节恰好对应第一个执行器”的假设。
joint_id = model.joint("shoulder_pan_joint").id
servo_id = model.actuator("shoulder_pan").id
# 关节 ID 是关节列表下标，qpos_address 是它在位置数组中的起始下标。
# 本模型一个转动关节占一个 qpos；不把这种对应关系泛化到所有机器人。
qpos_address = model.jnt_qposadr[joint_id]
home_ctrl = data.ctrl.copy()  # 保存全部六个初始目标，避免被后续赋值修改。
start_angle = float(home_ctrl[servo_id])  # 内部位置和控制单位均为 rad。
turn_angle = math.radians(30.0)  # 本实验只增加 30°，不改变控制增益。
end_angle = start_angle + turn_angle

# 同时核对关节范围和执行器指令范围；它们的含义不同。
# 本次轨迹在起点与终点之间单调变化，端点合法即可覆盖全部目标。
for enabled, limits, label in (
    (model.jnt_limited[joint_id], model.jnt_range[joint_id], "关节"),
    (model.actuator_ctrllimited[servo_id], model.actuator_ctrlrange[servo_id], "执行器"),
):
    # enabled 为真才检查；all 要求起点和终点都在 [下限, 上限] 内。
    # 这只检查目标角度，不保证实际运动绝无超调，也不做碰撞路径规划。
    if enabled and not all(limits[0] <= a <= limits[1] for a in (start_angle, end_angle)):
        raise SystemExit(f"目标超过{label}范围，请减小 turn_angle。")


def command(sim_time):
    """前 2 秒保持；随后 4 秒平滑转动；6 秒以后保持新目标。"""
    # sim_time 是仿真秒数。减 2 表示延迟启动，除以 4 表示转动持续 4 秒。
    # max 和 min 把进度限制在 0～1：启动前为 0，结束后保持 1。
    progress = min(1.0, max(0.0, (sim_time - 2.0) / 4.0))
    # 半余弦插值让运动的起点和终点目标速度为零，避免突然跳变目标。
    # progress 为 0、0.5、1 时，blend 分别为 0、0.5、1；cos 的输入单位为 rad。
    blend = (1.0 - math.cos(math.pi * progress)) / 2.0
    return start_angle + turn_angle * blend


def set_target():
    # 每一步恢复其余关节的 home 目标，只更新肩部回转这一项。
    # 保持目标不代表实际角度完全不动：重力和关节之间的耦合仍然存在。
    # [:] 表示写入数组的所有元素。ctrl 是执行器输入；本模型以 rad 表示位置目标。
    # XML 内的伺服器根据目标、实际角度和速度计算力矩，并受力矩范围限制。
    data.ctrl[:] = home_ctrl
    data.ctrl[servo_id] = command(data.time)


def report():
    # 同一时刻计算目标并读取实际值，显示时才从 rad 转成 °。
    # 误差统一为“目标减实际”；flush=True 让终端及时显示，不等缓冲区写满。
    target = math.degrees(command(data.time))
    actual = math.degrees(data.qpos[qpos_address])
    print(f"时间 {data.time:5.2f} s，肩部目标 {target:7.2f}°，"
          f"实际 {actual:7.2f}°，误差 {target - actual:6.2f}°", flush=True)


def step():
    set_target()
    mujoco.mj_step(model, data)  # 与静态脚本的主要区别：真正推进物理仿真。
    mujoco.mj_forward(model, data)  # 更新刚体坐标供画面显示，不再推进时间。


# 进入循环前先设置控制并更新空间坐标，使第一帧就是正确的 home 姿态。
set_target()
mujoco.mj_forward(model, data)
if args.headless:
    # 无窗口计算不等待现实时间。步数=仿真时长/每步时长，round 转为整数。
    report()
    for index in range(round(10.0 / model.opt.timestep)):
        step()
        # index 从 0 开始，index+1 才是已完成步数；% 为取余，每 2 秒打印一次。
        if (index + 1) % round(2.0 / model.opt.timestep) == 0:
            report()
else:
    # 被动查看器只显示状态，必须由下面的 step() 主动推进仿真。
    import mujoco.viewer
    next_report = 0.0
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            # perf_counter 是现实中的高精度计时，与 data.time 仿真时间分开使用。
            wall_start = time.perf_counter()
            if data.time >= next_report:
                report()
                next_report += 0.5  # 每半秒仿真时间输出，避免每步打印刷满终端。
            step()
            viewer.sync()  # 将计算结果同步给窗口，并处理查看器交互。
            # 尽量按真实速度播放；机器算得慢时，画面可能慢于仿真设定。
            time.sleep(max(0.0, model.opt.timestep - (time.perf_counter() - wall_start)))

# 要重看完整过程，关闭窗口后重新运行。此实验尚未实现查看器 Reset 后重播。
