
import mujoco
import mujoco.viewer
import numpy as np
import time

# Load your scene (which includes so_arm100.xml)
model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)

# Joint names and limits
joint_names = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"]
joint_limits_deg = [
    (-110, 110),     # Rotation
    (-190, 10),      # Pitch
    (-10, 180),      # Elbow
    (-95, 95),       # Wrist_Pitch
    (-160, 160),     # Wrist_Roll
    (-10, 100),      # Jaw
]

# Helper to convert degrees to radians
def deg_to_rad(deg_list):
    return np.radians(deg_list)

# Base pose
base_pose_deg = [0, -90, 90, 90, -90, 0]
base_pose_rad = deg_to_rad(base_pose_deg)

# Ask for one pose
def get_user_pose(label=""):
    user_pose_deg = []
    print(f"\n=== Enter Joint Angles for {label} Pose ===")
    for i, name in enumerate(joint_names):
        min_deg, max_deg = joint_limits_deg[i]
        while True:
            try:
                val = float(input(f"Enter {name} (range {min_deg}° to {max_deg}°): "))
                if min_deg <= val <= max_deg:
                    user_pose_deg.append(val)
                    break
                else:
                    print(f"❌ Out of range. Must be between {min_deg}° and {max_deg}°.")
            except ValueError:
                print("❌ Invalid input. Please enter a number.")
    return deg_to_rad(user_pose_deg)

# Animate transition between two poses
def interpolate_and_animate(start_pose, end_pose, n_steps=100, sleep=1/60.0):
    trajectory = np.linspace(start_pose, end_pose, n_steps)
    for step in trajectory:
        for i, name in enumerate(joint_names):
            joint_id = model.joint(name).qposadr
            data.qpos[joint_id] = step[i]
        mujoco.mj_forward(model, data)
        viewer.sync()
        time.sleep(sleep)

# Hold pose for specified duration
def hold_pose(pose_rad, hold_time=5.0, frame_rate=60):
    for _ in range(int(hold_time * frame_rate)):
        for i, name in enumerate(joint_names):
            joint_id = model.joint(name).qposadr
            data.qpos[joint_id] = pose_rad[i]
        mujoco.mj_forward(model, data)
        viewer.sync()
        time.sleep(1.0 / frame_rate)

# Run MuJoCo viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("🎮 MuJoCo Viewer Launched.")

    # Collect 3 poses from the user
    pose1 = get_user_pose("Position 1")
    pose2 = get_user_pose("Position 2")
    pose3 = get_user_pose("Position 3")

    # Animate transitions
    print("\n🔁 Moving from base to Position 1...")
    interpolate_and_animate(base_pose_rad, pose1)
    hold_pose(pose1)

    print("🔁 Moving from Position 1 to Position 2...")
    interpolate_and_animate(pose1, pose2)
    hold_pose(pose2)

    print("🔁 Moving from Position 2 to Position 3...")
    interpolate_and_animate(pose2, pose3)
    hold_pose(pose3)

    print("✅ Final pose reached. Viewer will remain open.")
    while viewer.is_running():
        mujoco.mj_forward(model, data)
        viewer.sync()

