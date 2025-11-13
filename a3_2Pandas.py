"""
a3_2Pandas.py

Two Franka Panda Robots moving Rod. 
Panda 1 (RIGHT) = leader → you move its joints with sliders.
Panda 2 (LEFT)  = follower → solves IK to grab the rods far end.
"""

# imports
import time
from math import pi
import numpy as np

import roboticstoolbox as rtb
from spatialmath import SE3
import swift
import spatialgeometry as sg


# visualization 
viz = swift.Swift()
viz.launch(realtime=True)


# Robots 
bot_right = rtb.models.Panda()  # RIGHT robot = leader (Panda 1)
bot_left  = rtb.models.Panda()  # LEFT  robot = follower (Panda 2)

# Bases:
# - Left robot sits a bit left and back.
# - Right robot is mirrored to face left (Rz(pi)).
bot_left.base  = SE3(-0.50, -0.10, 0.0)
bot_right.base = SE3(+0.50,  0.00, 0.0) * SE3.Rz(pi)

print("Right (leader) base:", bot_right.base.t.round(3))
print("Left  (follower) base:", bot_left.base.t.round(3))

viz.add(bot_right)
viz.add(bot_left)

# visualize axrs
viz.add(sg.Axes(length=0.15, pose=SE3()))
viz.add(sg.Axes(length=0.15, pose=bot_right.base))
viz.add(sg.Axes(length=0.15, pose=bot_left.base))


# Tool frame
# Keep tool frames at the flange 
bot_right.tool = SE3(0, 0, 0)
bot_left.tool  = SE3(0, 0, 0)


# initial joint angles, covnerted degrees to radian 
leader_q_deg = [0, -17, 0, -126, 0, 115, 45]
bot_right.q = np.deg2rad(leader_q_deg).astype(float)
# Follower starts wherever; it will be set by IK on the first call.


# Rod
ROD_LENGTH_M = 0.30
ROD_RADIUS_M = 0.01

rod_geom = sg.Cylinder(
    radius=float(ROD_RADIUS_M),
    length=float(ROD_LENGTH_M),
    color=[0.8, 0.2, 0.2, 1.0],  
)
viz.add(rod_geom)


# Gripper width
GRIPPER_WIDTH_M = 0.005
bot_right.grippers[0].q = [GRIPPER_WIDTH_M, GRIPPER_WIDTH_M]
bot_left.grippers[0].q = [GRIPPER_WIDTH_M, GRIPPER_WIDTH_M]

# Visual contact offsets and  spacing
CONTACT_Z_RIGHT = -0.01   # Leader depth
CONTACT_Z_LEFT  = -0.01   # Follower depth 

# shrinks the gap (brings follower closer)
PULL_IN = 0.55  


# Helpers - skew matric
def _skew(v):
    """Return 3x3 skew-symmetric matrix of a 3-vector."""
    x, y, z = v
    return np.array([[0, -z,  y],
                     [z,  0, -x],
                     [-y, x,  0]], dtype=float)

def rot_align_world_z_to(vec_world):
    """
    Build rotation R such that R @ [0,0,1] = unit(vec_world) to align the cylinder axis.
    """
    z_axis = np.array([0.0, 0.0, 1.0], dtype=float)
    v = np.asarray(vec_world, dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-12:
        return np.eye(3)
    v = v / n

    # If already aligned (or opposite)
    if np.allclose(v, z_axis, atol=1e-9):
        return np.eye(3)
    if np.allclose(v, -z_axis, atol=1e-9):
        return SE3.Rx(np.pi).R

    k = np.cross(z_axis, v)         # rotation axis (not unit)
    c = float(z_axis @ v)           # cos(theta)
    K = _skew(k)
    s2 = (np.linalg.norm(k) ** 2)   # sin^2(theta)
    return np.eye(3) + K + (K @ K) * ((1.0 - c) / s2)

def rot_from_x_and_down(x_dir_world, roll_bias_rad=0.0):
    """
    Make a rotation with:
      - +X aligned to x_dir_world (unit),
      - +Z as “down-ish” (close to world -Z) to keep pads approximately level,
      - optional extra roll around X (roll_bias_rad).
    """
    x = np.asarray(x_dir_world, dtype=float)
    x /= np.linalg.norm(x)

    z_want = np.array([0.0, 0.0, -1.0])  # world -Z (“down”)
    z = z_want - (z_want @ x) * x        # remove any component along x
    if np.linalg.norm(z) < 1e-9:
        # x is vertical; just pick a horizontal z and orthogonalize
        z = np.array([0.0, -1.0, 0.0]) - (np.array([0.0, -1.0, 0.0]) @ x) * x
    z /= np.linalg.norm(z)

    y = np.cross(z, x)
    y /= np.linalg.norm(y)

    R = np.column_stack((x, y, z))

    return R


# keep follower on far end + place the rod 
def update_follower_and_rod():
    """
    1. Read the RIGHT robot tool pose (leader).
    2. Define the leader contact frame slightly below the tool using CONTACT_Z_RIGHT
    3. Define the desired follower contact target along the rod axis from the leader contact
       (distance = ROD_LENGTH_M - PULL_IN).
    4. Solve IK for the LEFT robot to reach that target.
    5. Recompute both contacts and place the rod so its other stays connected at leader contact.
    """
    # leader tool
    T_tool_right = bot_right.fkine(bot_right.q)  # returns SE3 
    # leader contact (push rod  down by CONTACT_Z_RIGHT)
    T_contact_right = T_tool_right * SE3(0, 0, -CONTACT_Z_RIGHT)
    p_right = T_contact_right.t

    # follower desired contact:
    target_span = ROD_LENGTH_M - PULL_IN
    T_contact_left_des = T_contact_right * SE3.Tx(target_span)

    # follower +X should point back toward the leader (so grippers oppose each other)
    x_rod_dir = T_contact_left_des.t - p_right
    x_rod_dir /= np.linalg.norm(x_rod_dir)
    R_tool_left_des = rot_from_x_and_down(-x_rod_dir, roll_bias_rad=0.0)

    # undo the follower’s fingertip Z offset to get the actual tool target
    T_tool_left_des = SE3.Rt(R_tool_left_des, T_contact_left_des.t) * SE3(0, 0, +CONTACT_Z_LEFT)

    # IK for LEFT robot (try full 6D, then position-only fallback)
    sol = None
    try:
        sol = bot_left.ikine_LM(T_tool_left_des, q0=bot_left.q, ilimit=200, slimit=50, tol=1e-6)
        if getattr(sol, "success", False):
            bot_left.q = sol.q
    except Exception:
        pass

    if not getattr(sol, "success", False):
        try:
            sol = bot_left.ikine_LM(
                T_tool_left_des, q0=bot_left.q,
                mask=[1,1,1,0,0,0],  # position only
                ilimit=250, slimit=80, tol=1e-6
            )
            if getattr(sol, "success", False):
                bot_left.q = sol.q
        except Exception:
            pass

    # Place the rod so its near end stays at the leader contact, and the far end matches the follower
    T_tool_left = bot_left.fkine(bot_left.q)
    T_contact_left = T_tool_left * SE3(0, 0, -CONTACT_Z_LEFT)
    p_left = T_contact_left.t

    axis_vec = p_left - p_right
    axis_len = np.linalg.norm(axis_vec)
    if axis_len < 1e-9:
        # skip this frame 
        return

    axis_unit = axis_vec / axis_len

    # Keep the rod endend connected to the right gripper, center is midway for a centered cylinder
    rod_center_world = p_right + (ROD_LENGTH_M / 2.0) * axis_unit
    R_cyl = rot_align_world_z_to(axis_unit)  # cylinder local +Z → world axis direction

    T_rod = np.eye(4)
    T_rod[:3, :3] = R_cyl
    T_rod[:3, 3]  = rod_center_world

    # SE3 
    rod_geom.T = SE3(T_rod)


# run once so follower attaches to rod and the rod shows up correctly
update_follower_and_rod()


# Sliders for leader joints 
def on_slider_move(joint_index, value_deg):
    """
    Updates one leader joint (in degrees), then recomputes follower IK and rod pose.
    """
    q_now = bot_right.q.copy()
    q_now[joint_index] = np.deg2rad(float(value_deg))
    bot_right.q = q_now
    update_follower_and_rod()

# 7 sliders (one per joint)
slider_joint_idx = 0
for link in bot_right.links:
    if link.isjoint:
        qmin = float(np.rad2deg(link.qlim[0]))
        qmax = float(np.rad2deg(link.qlim[1]))
        qval = float(np.rad2deg(bot_right.q[slider_joint_idx]))

        # Careful with lambdas in loops: capture the current index with default arg (idx=...)
        viz.add(
            swift.Slider(
                lambda val, idx=slider_joint_idx: on_slider_move(idx, val),
                min=round(qmin, 2),
                max=round(qmax, 2),
                step=1,
                value=round(qval, 2),
                desc=f"Panda1 Joint{slider_joint_idx+1}",
                unit="&#176;",
            )
        )

        slider_joint_idx += 1
        if slider_joint_idx >= 7:
            break


#Main
try:
    while True:
        viz.step(0.02)   # advance sim
        time.sleep(0.01) 
except KeyboardInterrupt:
    print("Simulation ended.")
