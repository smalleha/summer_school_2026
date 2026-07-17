import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import numpy as np
import time
from scipy.spatial.transform import Rotation as R

# ===================== 1. 配置参数 =====================
URDF_PATH = "/home/agilex/agilex_open_class_ws/src/agilex_open_class/piper/piper_description/urdf/piper_description.urdf"
PACKAGE_DIRS = ["/home/agilex/agilex_open_class_ws/src/agilex_open_class/piper"]

EE_FRAME_NAME = "link6"
TOTAL_TIME = 8.0
FREQ = 100
DT = 1.0 / FREQ

# ===================== 圆弧运动参数 =====================
# 圆心位置
CENTER = np.array([0.3, 0.0, 0.25])
# 圆弧半径
RADIUS = 0.12
# 起始角度（度）
START_ANGLE = 0
# 结束角度（度）
END_ANGLE = 180
# 圆弧所在平面：'xy' / 'xz' / 'yz'
PLANE = "yz"

# 姿态保持不变
USE_CURRENT_EE_ROTATION = True
EE_FIXED_QUAT = np.array([0.0, 1.0, 0.0, 0.0])

# ===================== 2. 加载URDF模型 =====================
model, collision_model, visual_model = pin.buildModelsFromUrdf(
    URDF_PATH,
    package_dirs=PACKAGE_DIRS,
)
data = model.createData()
ee_frame_id = model.getFrameId(EE_FRAME_NAME)

if ee_frame_id >= model.nframes:
    raise ValueError(f"末端执行器帧 {EE_FRAME_NAME} 未在URDF中找到！")

# ===================== 3. 初始化可视化 =====================
viz = MeshcatVisualizer(model, collision_model, visual_model)
viz.initViewer(open=True)
viz.loadViewerModel()
viz.display(pin.neutral(model))

# ===================== 4. 辅助函数 =====================
def get_ee_pose(q):
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacement(model, data, ee_frame_id)
    pose = data.oMf[ee_frame_id]
    return pose.translation, pose.rotation
 
def ik_solve(target_pos, target_rot, q_init):
    q = q_init.copy()
    eps = 1e-3
    IT_MAX = 1000
    damp = 1e-4
    step_size = 0.1
    converged = False
    err_norm = None

    for _ in range(IT_MAX):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacement(model, data, ee_frame_id)
        M = data.oMf[ee_frame_id]

        err_pos = target_pos - M.translation
        err_rot = pin.log3(target_rot @ M.rotation.T)
        err = np.concatenate([err_pos, err_rot])
        err_norm = np.linalg.norm(err)

        if err_norm < eps:
            converged = True
            break

        J = pin.computeFrameJacobian(
            model, data, q, ee_frame_id, pin.LOCAL_WORLD_ALIGNED
        )
        J_T = J.T
        v = J_T @ np.linalg.solve(J @ J_T + damp * np.eye(6), err)
        q = pin.integrate(model, q, step_size * v)
        q = np.clip(q, model.lowerPositionLimit, model.upperPositionLimit)

    return q, converged, err_norm

def circular_trajectory(t, t_total, center, radius, start_angle, end_angle, plane="xy"):
    """生成平滑圆弧轨迹（五次多项式角速度）"""
    s = np.clip(t / t_total, 0, 1)
    s_smooth = 10 * s**3 - 15 * s**4 + 6 * s**5

    angle_start = np.deg2rad(start_angle)
    angle_end = np.deg2rad(end_angle)
    angle = angle_start + s_smooth * (angle_end - angle_start)

    target = center.copy()

    if plane == "xy":
        target[0] += radius * np.cos(angle)
        target[1] += radius * np.sin(angle)
    elif plane == "xz":
        target[0] += radius * np.cos(angle)
        target[2] += radius * np.sin(angle)
    elif plane == "yz":
        target[1] += radius * np.cos(angle)
        target[2] += radius * np.sin(angle)

    return target

# ===================== 5. 主控制循环：圆弧运动 =====================
q_current = pin.neutral(model)
start_pos, start_rot = get_ee_pose(q_current)

if USE_CURRENT_EE_ROTATION:
    target_rot = start_rot
else:
    target_rot = R.from_quat(EE_FIXED_QUAT).as_matrix()

print("开始圆弧运动，可视化窗口已打开！")
for t in np.arange(0, TOTAL_TIME + DT, DT):
    # 生成圆弧目标点
    target_pos = circular_trajectory(
        t, TOTAL_TIME, CENTER, RADIUS, START_ANGLE, END_ANGLE, PLANE
    )

    # 逆解
    q_target, converged, err_norm = ik_solve(target_pos, target_rot, q_current)
    if not converged:
        print(f"IK未收敛，t={t:.2f}s，误差={err_norm:.4f}")
        break

    q_current = q_target
    viz.display(q_current)
    time.sleep(DT)

print("圆弧运动完成！")
input("按回车退出...")

