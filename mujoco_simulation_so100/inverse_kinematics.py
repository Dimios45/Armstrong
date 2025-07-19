import mujoco
import mujoco.viewer
import numpy as np
import time

# ========== Load Model ==========
model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)

# Joint names as per MJCF
joint_names = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"]

# Joint limits in degrees (same order)
joint_limits_deg = {
    "Rotation": (-110, 110),
    "Pitch": (-190, 10),
    "Elbow": (-10, 180),
    "Wrist_Pitch": (-95, 95),
    "Wrist_Roll": (-160, 160),
    "Jaw": (-10, 100)
}

# Joint limits in radians
joint_limits_rad = {j: np.radians(lim) for j, lim in joint_limits_deg.items()}

# ========== FK ==========
def forward_kinematics(q):
    for i, name in enumerate(joint_names):
        joint_id = model.joint(name).qposadr
        data.qpos[joint_id] = q[i]
    mujoco.mj_forward(model, data)
    return data.body("Moving_Jaw").xpos.copy()

# ========== IK ==========
def inverse_kinematics(target_pos, initial_q, max_iters=100, lr=0.5, tol=1e-4):
    q = initial_q.copy()
    for _ in range(max_iters):
        current_pos = forward_kinematics(q)
        error = target_pos - current_pos
        if np.linalg.norm(error) < tol:
            break

        J = np.zeros((3, len(joint_names)))
        eps = 1e-6
        for i in range(len(joint_names)):
            dq = np.zeros_like(q)
            dq[i] = eps
            pos_plus = forward_kinematics(q + dq)
            pos_minus = forward_kinematics(q - dq)
            J[:, i] = (pos_plus - pos_minus) / (2 * eps)

        dq = lr * np.linalg.pinv(J) @ error
        q += dq

        # Enforce joint limits
        for i, name in enumerate(joint_names):
            low, high = joint_limits_rad[name]
            q[i] = np.clip(q[i], low, high)
    return q

# ========== Animate ==========
def interpolate_and_animate(q_start, q_end, n_steps=100, hold_time=3.0):
    trajectory = np.linspace(q_start, q_end, n_steps)
    for q in trajectory:
        for i, name in enumerate(joint_names):
            joint_id = model.joint(name).qposadr
            data.qpos[joint_id] = q[i]
        mujoco.mj_forward(model, data)
        viewer.sync()
        time.sleep(1/60)

    # Hold final pose
    for _ in range(int(hold_time * 60)):
        mujoco.mj_forward(model, data)
        viewer.sync()
        time.sleep(1/60)

# ========== Get User Input ==========
def get_xyz_input(n_targets=3):
    positions = []
    for i in range(n_targets):
        while True:
            try:
                inp = input(f"Enter target position {i+1} (x y z in meters): ")
                x, y, z = map(float, inp.strip().split())
                positions.append(np.array([x, y, z]))
                break
            except ValueError:
                print("❌ Invalid input. Please enter 3 space-separated numbers.")
    return positions

# ========== MAIN ==========
if __name__ == "__main__":
    print("🚀 Enter 3 target (x, y, z) positions in meters to move robot arm via IK.")
    targets = get_xyz_input(3)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        q_current = np.zeros(len(joint_names))  # start at zeros

        for i, target in enumerate(targets):
            print(f"\n🎯 Solving IK for target {i+1}: {target}")
            q_solution = inverse_kinematics(target, q_current)
            print(f"✅ Joint angles (deg): {np.degrees(q_solution)}")
            interpolate_and_animate(q_current, q_solution)
            q_current = q_solution.copy()

        print("\n✅ Animation complete. Viewer still active.")
        while viewer.is_running():
            mujoco.mj_forward(model, data)
            viewer.sync()

