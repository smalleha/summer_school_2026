import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import numpy as np
import time

# =====================================================
# 1. 参数
# =====================================================

URDF_PATH = "/home/agilex/agilex_open_class_ws/src/agilex_open_class/piper/piper_description/urdf/piper_description.urdf"

PACKAGE_DIRS = [
    "/home/agilex/agilex_open_class_ws/src/agilex_open_class/piper"
]

EE_FRAME_NAME = "link6"

FREQ = 200
DT = 1.0 / FREQ

T1 = 3.0
T2 = 5.0
T3 = 3.0
T_ALL = T1 + T2 + T3

OFFSET_LINE1 = np.array([0.0, 0.0, 0.08])
ARC_CENTER_OFFSET = np.array([0.22, 0.0, 0.08])
ARC_RADIUS = 0.10

ARC_START_DEG = -90
ARC_END_DEG = 90
ARC_PLANE = "yz"


# =====================================================
# 2. 建模
# =====================================================

model, collision_model, visual_model = pin.buildModelsFromUrdf(
    URDF_PATH,
    package_dirs=PACKAGE_DIRS
)

data = model.createData()
ee_frame_id = model.getFrameId(EE_FRAME_NAME)

if ee_frame_id >= model.nframes:
    raise RuntimeError("EE frame not found")

viz = MeshcatVisualizer(model, collision_model, visual_model)
viz.initViewer(open=True)
viz.loadViewerModel()

q0 = pin.neutral(model)
viz.display(q0)


# =====================================================
# 3. S曲线
# =====================================================

def smooth_step(s):
    s = np.clip(s, 0.0, 1.0)
    return 10*s**3 - 15*s**4 + 6*s**5


# =====================================================
# 4. FK
# =====================================================

def get_ee_pose(q):
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacement(model, data, ee_frame_id)
    M = data.oMf[ee_frame_id]
    return M.translation.copy(), M.rotation.copy()


# =====================================================
# 5. IK（Position Only）
# =====================================================

def ik_solve(target_pos, q_init):
    q = q_init.copy()

    IT_MAX = 50
    eps = 1e-4

    for _ in range(IT_MAX):

        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacement(model, data, ee_frame_id)

        M = data.oMf[ee_frame_id]

        err = target_pos - M.translation
        err_norm = np.linalg.norm(err)

        if err_norm < eps:
            break

        J = pin.computeFrameJacobian(
            model,
            data,
            q,
            ee_frame_id,
            pin.LOCAL_WORLD_ALIGNED
        )[:3, :]

        damp = 1e-4 + min(err_norm * 1e-3, 1e-2)

        JJt = J @ J.T

        v = J.T @ np.linalg.solve(
            JJt + damp * np.eye(3),
            err
        )

        alpha = min(0.5, 0.2 / (err_norm + 1e-6))

        q = pin.integrate(model, q, alpha * v)

        q = np.clip(
            q,
            model.lowerPositionLimit,
            model.upperPositionLimit
        )

    return q


# =====================================================
# 6. 圆弧函数
# =====================================================

def arc_point(center, angle):

    p = center.copy()

    if ARC_PLANE == "yz":
        p[1] += ARC_RADIUS * np.cos(angle)
        p[2] += ARC_RADIUS * np.sin(angle)

    elif ARC_PLANE == "xy":
        p[0] += ARC_RADIUS * np.cos(angle)
        p[1] += ARC_RADIUS * np.sin(angle)

    elif ARC_PLANE == "xz":
        p[0] += ARC_RADIUS * np.cos(angle)
        p[2] += ARC_RADIUS * np.sin(angle)

    return p


# =====================================================
# 7. 关键修复：保证连续性
# =====================================================

def compute_arc_start(center):
    return arc_point(center, np.deg2rad(ARC_START_DEG))


def compute_arc_end(center):
    return arc_point(center, np.deg2rad(ARC_END_DEG))


# =====================================================
# 8. 初始化关键点
# =====================================================

q_cur = q0.copy()

p0, _ = get_ee_pose(q_cur)

arc_center = p0 + ARC_CENTER_OFFSET

p1 = compute_arc_start(arc_center)   # ✅ 关键修复：直线终点=圆弧起点

p_arc_end = compute_arc_end(arc_center)

p_end = p0.copy()


# =====================================================
# 9. 轨迹生成
# =====================================================

def get_des_pos(t):

    if t <= T1:
        s = smooth_step(t / T1)
        return p0 + s * (p1 - p0)

    elif t <= T1 + T2:
        t2 = t - T1
        s = smooth_step(t2 / T2)

        ang0 = np.deg2rad(ARC_START_DEG)
        ang1 = np.deg2rad(ARC_END_DEG)
        ang = ang0 + s * (ang1 - ang0)

        return arc_point(arc_center, ang)

    else:
        t3 = t - T1 - T2
        s = smooth_step(t3 / T3)
        return p_arc_end + s * (p_end - p_arc_end)


# =====================================================
# 10. 预生成轨迹
# =====================================================

print("生成轨迹...")

traj = []

for t in np.arange(0, T_ALL + DT, DT):
    traj.append(get_des_pos(t))

print("轨迹点数:", len(traj))


# =====================================================
# 11. IK轨迹
# =====================================================

print("计算IK轨迹...")

joint_traj = []
q_seed = q0.copy()

for i, p in enumerate(traj):
    q_seed = ik_solve(p, q_seed)
    joint_traj.append(q_seed.copy())


print("IK完成")


# =====================================================
# 12. 播放（只执行一次）
# =====================================================

print("开始播放...")

for q in joint_traj:
    viz.display(q)
    time.sleep(DT)

print("执行完成")