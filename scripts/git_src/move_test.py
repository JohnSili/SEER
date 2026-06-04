from robotsim import RobotSim
from generate_trajectory import generate_trajectory, plot_data, min_time
from arm_planning_api import ArmPlanningAPI
import time
import numpy as np

def angle_wrap(q):
    return np.round((q + np.pi) % (2 * np.pi) - np.pi, 3)

def main():
    # Settings
    T_des = 3 # seconds
    dt = 0.01  # control interval
    eps = 0.01 

    zero_joints = [0] * 7
    target_joints = [2.094384716064944, 1.5707999719966386, -1.5707823052517573, 1.7452778929327073, -8.508799682803165e-06, 0.7853949276567239, 5.992112452678286e-07]
    waypoints = np.array([zero_joints, target_joints, zero_joints])
    
    T = max(T_des, min_time(way_points=waypoints)) - 0.45 
    
    # check the trajectory in simulation
    robot = RobotSim()
    print("Moving robot")
    robot.move_jack(0.3)

    time.sleep(1)

    print("Computing trajectory")
    waypoints = np.array([zero_joints, target_joints, zero_joints])

    t, _, _, _, c_splines = generate_trajectory(waypoints, T, dt)
    
    trajectory_positions = []
    joint_states = []
    
    print("Simulation Start")
    try:
        # --- 2. Execution loop with time‑based lookup ---
        physics_dt = 1/240

        next_control = 0.0
        start = time.perf_counter()
        counter = 0
        while True:
            t = time.perf_counter() - start
            if t >= next_control:
                q_actual = robot.get_left_joints()
                joint_states.append(np.round(q_actual, 3))
                
                q_desired = [angle_wrap(c_splines[j](min(t,T)))
                            for j in range(7)]
                robot.move_left_arm(q_desired)
                next_control += dt

                trajectory_positions.append(q_desired)
                err = np.linalg.norm(np.array(q_desired) - np.array(q_actual))
            
            # Check every 20 frames
            if counter % 40 == 0:
                collision, _, _ = robot.check_self_collision_distance(
                    threshold=0.02,
                    color_collisions=True,
                    visualize=False
                    )
                
                if collision:
                    print("Collosion detected")
                    break
            counter += 1

            robot.sync()
            robot.step()
            time.sleep(physics_dt)

            if t > T and err<0.001:
                break
        print(f"Trajectory excuted in {t} second\
              \nDesigned time is {T_des} second\
              \nSSE {np.round(err,3)} meter")
        time.sleep(2)
    finally:
        print("Simulation finished")
        try:
            robot.disconnect()
        finally:
            pass
    plot_data(trajectory_positions, joint_states)
    
    if collision:
        print("Trajectory is not valid, collosion detection")
        return
    else:
        print("Excuting trajectory")

    # print("Program Start")
    
    # robotAPI = ArmPlanningAPI("192.168.3.170")

    # print("Deactivating real time")
    # robotAPI.out_switch_real_time_mode()

    # time.sleep(0.1)
    # print("Robot move to zero postition")
    # robotAPI.controlLeft(zero_joints, vel = 0.35)
    # robotAPI.waitMove()

    # time.sleep(0.1)
    
    # initial_joints = robotAPI.getLeftJoint()
    # print("Initial joints: ", initial_joints)

    # trajectory_positions = []
    # joint_states = []

    # print("Activating real time for left arm")
    # robotAPI.switch_real_time_mode("left_arm")
    # print("Excuting Trajectory")

    # # --- Execution loop with time‑based lookup ---
    # next_control = 0.0
    # start = time.perf_counter()
    # while True:
    #     t = time.perf_counter() - start
    #     if t >= next_control:
    #         q_actual = robotAPI.getLeftJoint()
    #         joint_states.append(np.round(q_actual, 3))
                
    #         q_desired = [angle_wrap(c_splines[j](min(t,T)))
    #                     for j in range(7)]
    #         robotAPI.left_real_time_trajectory(q_desired)

    #         trajectory_positions.append(q_desired)
    #         err = np.linalg.norm(np.array(q_desired) - np.array(q_actual))
    #         next_control += dt

    #         if t > T and err<eps:
    #             break

    # print(f"Trajectory excuted in {t} second\
    #         \nDesigned time is {T_des} second\
    #         \nSSE {np.round(err,3)} meter")
        
    # time.sleep(0.1)

    # print("Deactivating real time")
    # robotAPI.out_switch_real_time_mode()
    # print("Excution End")
    # plot_data(trajectory_positions, joint_states)

if __name__ == '__main__':
    main()