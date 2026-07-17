import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import numpy as np
import time
from scipy.spatial.transform import Rotation as R

# ===================== 1. 配置参数 =====================
URDF_PATH = "/home/agilex/agilex_open_class_ws/src/agilex_open_class/piper/piper_description/urdf/piper_description.urdf"
PACKAGE_DIRS = ["/home/agilex/agilex_open_class_ws/src/agilex_open_class/piper"]
EE_FRAME_NAME = "link6"

# 控制参数
FREQ = 100
DT = 1.0 / FREQ
# 为不同阶段设置不同的运动时间
TIME_TO_START = 3.0  # 从零位到起点的准备时间
TIME_OFFSET_MOVE = 5.0 # 偏移运动时间
TIME_TO_HOME = 3.0   # 回到零位的回退时间

# 直线运动起点和终点（相对当前末端位置的偏移量，单位：米）
START_OFFSET = np.array([0.2, 0.0, 0.1])  # 起点偏移
END_OFFSET = np.array([0.2, 0.2, 0.1])    # 终点偏移

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
    """正向运动学：获取末端执行器位姿"""
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacement(model, data, ee_frame_id)
    pose = data.oMf[ee_frame_id]
    return pose.translation, pose.rotation

def ik_solve(target_pos, target_rot, q_init):
    """逆运动学：求解关节角度"""
    q = q_init.copy()
    eps = 1e-3
    IT_MAX = 1000
    damp = 1e-4
    step_size = 0.1

    err_norm = None
    converged = False
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
        
        J = pin.computeFrameJacobian(model, data, q, ee_frame_id, pin.LOCAL_WORLD_ALIGNED)
        J_T = J.T
        v = J_T @ np.linalg.solve(J @ J_T + damp * np.eye(6), err)
        q = pin.integrate(model, q, step_size * v)
        q = np.clip(q, model.lowerPositionLimit, model.upperPositionLimit)

    return q, converged, err_norm

def linear_trajectory(t, t_total, start, end):
    """直线轨迹插值（五次多项式平滑运动）"""
    s = np.clip(t / t_total, 0, 1)
    s_smooth = 10 * s**3 - 15 * s**4 + 6 * s**5
    return start + s_smooth * (end - start)

def execute_cartesian_move(q_start, pos_start, pos_end, target_rot, duration, stage_name=""):
    """
    执行单段笛卡尔空间直线运动
    """
    print(f"[{stage_name}] 开始执行...")
    q_current = q_start.copy()
    
    for t in np.arange(0, duration + DT, DT):
        target_pos = linear_trajectory(t, duration, pos_start, pos_end)
        q_target, converged, err_norm = ik_solve(target_pos, target_rot, q_current)
        
        if not converged:
            print(f"[{stage_name}] IK未收敛，停止运动。t={t:.3f}s, 误差={err_norm:.6f}")
            return q_current, False
        
        q_current = q_target
        viz.display(q_current)
        time.sleep(DT)
        
    print(f"[{stage_name}] 执行完成！")
    return q_current, True

# ===================== 5. 主控制循环 =====================
def main():
    # 记录 Zero 位姿状态
    q_zero = pin.neutral(model)
    zero_pos, zero_rot = get_ee_pose(q_zero)
    zero_pos = zero_pos.copy()
    zero_rot = zero_rot.copy()

    if USE_CURRENT_EE_ROTATION:
        target_rot = zero_rot
    else:
        target_rot = R.from_quat(EE_FIXED_QUAT).as_matrix()

    # 计算目标路点
    start_offset_pos = zero_pos + START_OFFSET
    end_offset_pos = zero_pos + END_OFFSET

    print(f"Zero 位姿: {np.round(zero_pos, 4)}")
    print(f"起点偏移 (Start Offset): {np.round(start_offset_pos, 4)}")
    print(f"终点偏移 (End Offset): {np.round(end_offset_pos, 4)}")
    print("可视化窗口已打开，等待2秒后开始运动序列...")
    time.sleep(2.0)

    # 阶段 1: Zero -> Start Offset
    q_current, success = execute_cartesian_move(
        q_start=q_zero, 
        pos_start=zero_pos, 
        pos_end=start_offset_pos, 
        target_rot=target_rot, 
        duration=TIME_TO_START, 
        stage_name="阶段1: Zero -> 起点"
    )
    if not success: return
    time.sleep(0.5) # 停顿一下，方便观察

    # 阶段 2: Start Offset -> End Offset
    q_current, success = execute_cartesian_move(
        q_start=q_current, 
        pos_start=start_offset_pos, 
        pos_end=end_offset_pos, 
        target_rot=target_rot, 
        duration=TIME_OFFSET_MOVE, 
        stage_name="阶段2: 偏移运动 (起点 -> 终点)"
    )
    if not success: return
    time.sleep(0.5)
    # 阶段 3: End Offset -> Start Offset
    q_current, success = execute_cartesian_move(
        q_start=q_current, 
        pos_start=end_offset_pos, 
        pos_end=start_offset_pos, 
        target_rot=target_rot, 
        duration=TIME_OFFSET_MOVE, 
        stage_name="阶段3: 偏移运动 (终点 -> 起点)"
    )
    if not success: return
    time.sleep(0.5)

    # 阶段 4: Start Offset -> Zero
    q_current, success = execute_cartesian_move(
        q_start=q_current, 
        pos_start=start_offset_pos, 
        pos_end=zero_pos, 
        target_rot=target_rot, 
        duration=TIME_TO_HOME, 
        stage_name="阶段4: 起点 -> Zero 回退"
    )
    if not success: return

    print("\n所有运动序列执行完毕！")

if __name__ == "__main__":
    main()
    input("按回车退出...")