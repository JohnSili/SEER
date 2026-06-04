import numpy as np
import time
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def min_time(way_points, max_j_vel=np.pi):
    diff = abs(np.diff(way_points, axis=0))
    max_delta_q = np.max(diff, axis=1)
    t_max = 3.0/2.0 * max_delta_q / max_j_vel
    T = np.round(np.sum(t_max), 3)
    return T
# def compute_quintic_coeffs(p0, v0, a0, pf, vf, af, T):
#     M = np.array([
#         [0, 0, 0, 0, 0, 1],
#         [T**5, T**4, T**3, T**2, T, 1],
#         [0, 0, 0, 0, 1, 0],
#         [5*T**4, 4*T**3, 3*T**2, 2*T, 1, 0],
#         [0, 0, 0, 2, 0, 0],
#         [20*T**3, 12*T**2, 6*T, 2, 0, 0]
#     ])
#     b = np.array([p0, pf, v0, vf, a0, af])
#     return np.linalg.solve(M, b)

def plot_data(trajectory_positions, joint_states=None):
    trajectory_positions = list(zip(*trajectory_positions))

    joint_states = list(zip(*joint_states)) if joint_states is not None else None

    plt.figure(figsize=(12, 8))
    for i in range(7):
        plt.subplot(3, 3, i + 1)
        plt.plot(trajectory_positions[i], label="Trajectory Position")

        if joint_states is not None:
            plt.plot(joint_states[i], label="Joint State", linestyle="--")
        
        plt.title(f"Joint {i + 1}")
        plt.xlabel("Time Step")
        plt.ylabel("Position")
        plt.legend()
        plt.grid()
    plt.tight_layout()
    plt.show()

# def quintic_scalar(t, T):
#     tau = t / T
#     return 10*tau**3 - 15*tau**4 + 6*tau**5


# def generate_trajectory(p0_list, pf_list, v0_list, vf_list, a0_list, af_list, T, dt):
#     n = len(p0_list)
#     steps = int(T / dt)
#     trajectory = []

#     # shortest angular displacement
#     delta = [
#         wrap_to_pi(pf - p0)
#         for p0, pf in zip(p0_list, pf_list)
#     ]

#     for t_step in range(steps):
#         t = t_step * dt
#         s = quintic_scalar(t, T)

#         pos_t = [
#             p0_list[i] + s * delta[i]
#             for i in range(n)
#         ]

#         trajectory.append(np.round(pos_t, 3))

#     return trajectory


def wrap_to_pi(angle):
    return (angle + np.pi) % (2*np.pi) - np.pi


def generate_trajectory(waypoints, T=5.0, dt=0.01):
    """
    Generate a smooth joint trajectory through waypoints.

    Parameters
    ----------
    waypoints : array-like, shape (N_waypoints, N_joints)
        Joint configurations in radians.

    T : float
        Total trajectory duration [s].

    dt : float
        Sampling time [s].

    Returns
    -------
    t : ndarray, shape (N_samples,)
        Time vector.

    q : ndarray, shape (N_samples, N_joints)
        Joint positions.

    dq : ndarray, shape (N_samples, N_joints)
        Joint velocities.

    ddq : ndarray, shape (N_samples, N_joints)
        Joint accelerations.
    """

    waypoints = np.asarray(waypoints, dtype=float)

    if waypoints.ndim != 2:
        raise ValueError(
            "waypoints must have shape (N_waypoints, N_joints)"
        )

    n_waypoints, n_joints = waypoints.shape

    if n_waypoints < 2:
        raise ValueError(
            "At least two waypoints are required."
        )

    # Uniform waypoint timing
    t_wp = np.linspace(0.0, T, n_waypoints)

    # Sample times
    t = np.arange(0.0, T + dt, dt)

    # Unwrap angles for each joint independently
    waypoints_unwrapped = np.unwrap(waypoints, axis=0)

    q = np.zeros((len(t), n_joints))
    dq = np.zeros_like(q)
    ddq = np.zeros_like(q)

    c_splines = []
    for j in range(n_joints):
        spline = CubicSpline(
            t_wp,
            waypoints_unwrapped[:, j],
            bc_type="clamped"  # zero start/end velocity
        )

        c_splines.append(spline)
        
        q[:, j] = np.round(spline(t), 3)
        dq[:, j] = np.round(spline(t, 1), 3)
        ddq[:, j] = np.round(spline(t, 2), 3)

    # Wrap positions back to [-pi, pi]
    q = (q + np.pi) % (2 * np.pi) - np.pi

    return t, q, dq, ddq, c_splines

# def generate_trajectory(p0_list, pf_list, v0_list, vf_list, a0_list, af_list, T, dt):
#     n = len(p0_list)
#     steps = int(T / dt)
#     trajectory = []

#     # make final angles continuous w.r.t initial ones
#     pf_list_wrapped = [
#         p0 + wrap_to_pi(pf - p0)
#         for p0, pf in zip(p0_list, pf_list)
#     ]

#     for t_step in range(steps):
#         t = t_step * dt
#         pos_t = []

#         for i in range(n):
#             coeffs = compute_quintic_coeffs(
#                 p0_list[i], v0_list[i], a0_list[i],
#                 pf_list_wrapped[i], vf_list[i], af_list[i], T
#             )

#             p_t = np.polyval(coeffs[::-1], t)
#             pos_t.append(p_t)

#         trajectory.append(pos_t)

#     return trajectory

def main():
    # Settings
    T = 2.0  # seconds
    dt = 0.01  # control interval

    # Boundry conditions
    v0 = [0.0] * 7
    a0 = [0.0] * 7
    vf = [0.0] * 7
    af = [0.0] * 7

    zero_joints = [0] * 7
    target_joints = [2.094384716064944, 1.5707999719966386, -1.5707823052517573, 1.7452778929327073, -8.508799682803165e-06, 0.7853949276567239, 5.992112452678286e-07]

    print("Program Start")
    initial_joints = zero_joints
    print("Computing trajectory")
    
    waypoints = np.array([zero_joints, target_joints, zero_joints])

    _, q, _, _ = generate_trajectory(waypoints, T, dt)

    plot_data(q)    
    return
    robot = ArmPlanningAPI("192.168.3.170")
    
    robot.controlLeft(zero_joints)
    robot.waitMove()

    time.sleep(0.5)
    
    print("Program Start")
    initial_joints = robot.getLeftJoint()
    print("Initial joints: ", initial_joints)
    print("Computing trajectory")
    trajectory = generate_trajectory( initial_joints, target_joints, 
                                     v0, vf, 
                                     a0, af, 
                                     T, dt)

    trajectory_positions = []
    joint_states = []

    print("Activating real time for left arm")
    robot.switch_real_time_mode("left_arm")
    print("Excuting Trajectory")
    for pos in trajectory:
        start_time = time.perf_counter()
        joints = robot.getLeftJoint()
        robot.left_real_time_trajectory(pos)
        trajectory_positions.append(pos)
        joint_states.append(joints)
        elapsed = time.perf_counter() - start_time
        if elapsed < dt:
            time.sleep(dt - elapsed)

    print("Deactivating real time")
    robot.out_switch_real_time_mode()
    print("Excution End")
    plot_data(trajectory_positions, joint_states)

if __name__ == "__main__":
    main()