import pybullet as p
import time
import numpy as np

class RobotSim:
    def __init__(self, 
                 urdf_path="urdfs/urdf/robot.urdf", fixed_base=True,
                 q_key=ord('q')):

        # Connect with physics engine
        self.connect()

        # Load URDF
        print("Loading robot model file")
        self.robot_id = p.loadURDF(urdf_path, useFixedBase=fixed_base)
        self.num_joints = p.getNumJoints(self.robot_id)

        # Helper: get all actuated joint indices (ignore fixed joints)
        self.joint_indices, self.joint_names = self.get_movable_joints()

        print(f"Model has {self.num_joints} joints with {len(self.joint_indices)} moveable joints.")

        # Getting joints limits
        self.joint_indices, self.lower_limits, self.upper_limits = self.joints_limits()

        # Mapping moviable joints names to pybullet indecies
        self.names2ind()

        # Setting robot controller as position controller
        for idx in self.joint_indices:
            p.setJointMotorControl2(
                self.robot_id, idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=0.0,          # initial target
                positionGain=0.2,            # increase if arms are heavy
                velocityGain=0.0,            # can add damping later
                maxVelocity=10.0             # optional limit
            )

        # self.setup_collision_filters()
        # Ressting robot to initial position
        print("Set robot in initial position")
        self.reset()

        # Define quit key code
        self.q_key = q_key

        self.colored_links = set()
        self.colored_self_links = set()

        self.normal_color = [0.8, 0.82, 0.93, 1]
        self.collision_color = [1, 0, 0, 1]

    def get_movable_joints(self):
        joint_indices = []
        joint_names = []
        self.link_name_to_idx = {}
        self.link_idx_to_name = {}
        self.parent_of = {}

        for i in range(self.num_joints):
            info = p.getJointInfo(self.robot_id, i)
            joint_type = info[2]
            joint_name = info[1].decode('utf-8')
            link_name  = info[12].decode('utf-8')
            
            self.link_idx_to_name[i] = link_name
            
            self.link_name_to_idx[link_name] = i
            self.link_name_to_idx[joint_name] = i

            if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
                joint_indices.append(i)
                joint_names.append(joint_name)
            
            parent = info[16]
            self.parent_of[i] = parent

        return joint_indices, joint_names
    
    def is_parent_child(self, a, b):
        return (
            self.parent_of.get(a, -1) == b or
            self.parent_of.get(b, -1) == a
        )

    def connect(self):
        #  Connect to PyBullet GUI
        p.connect(p.GUI)
        p.setGravity(0, 0, -9.81)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # optional: hide side panel

    def disconnect(self):
        p.disconnect()

    def reset(self):
        self.robot_q = [0]*len(self.joint_indices)
        self.sync()
        self.step()

    def names2ind(self):
        name_to_index = {}

        for i, name in enumerate(self.joint_names):
                name_to_index[name] = i

        # Define right arm and hand joints (order matters – we'll preserve it)
        self.right_arm_joints = ['arm_right_1', 'arm_right_2', 'arm_right_3', 'arm_right_4',
                                 'arm_right_5', 'arm_right_6', 'arm_right_7']

        self.right_finger_joints = ['right_thumb_1_joint', 'right_thumb_2_joint', 'right_thumb_3_joint',
                                    'right_thumb_4_joint', 'right_index_1_joint', 'right_index_2_joint',
                                    'right_middle_1_joint', 'right_middle_2_joint', 'right_ring_1_joint',
                                    'right_ring_2_joint', 'right_little_1_joint', 'right_little_2_joint']

        self.left_arm_joints = ['arm_left_1', 'arm_left_2', 'arm_left_3', 'arm_left_4',
                                'arm_left_5', 'arm_left_6', 'arm_left_7']

        self.left_finger_joints = ['left_thumb_swing_joint', 'left_thumb_1_joint', 'left_thumb_2_joint',
                                    'left_thumb_3_joint', 'left_index_1_joint', 'left_index_2_joint',
                                    'left_middle_1_joint', 'left_middle_2_joint', 'left_ring_1_joint',
                                    'left_ring_2_joint', 'left_little_1_joint', 'left_little_2_joint']

        # Convert names to indices
        self.right_arm_indices = [name_to_index[n] for n in self.right_arm_joints]
        self.right_finger_indices = [name_to_index[n] for n in self.right_finger_joints]
        self.left_arm_indices = [name_to_index[n] for n in self.left_arm_joints]
        self.left_finger_indices = [name_to_index[n] for n in self.left_finger_joints]
        self.jack_idx = name_to_index['jack']

        LEFT_EXCLUDED_pair = [
            "arm1_6_link",
            "arm1_7_link",
        ]

        EXCLUDED_GROUP = [
            "waist1_link",
            "arm1_1_link",
            "arm2_1_link",
            "arm1_base_link"
        ]

        LEFT_HAND = [
            "arm1_7_link",
            "arm1_ee_link",
            "flange_l_link",
            "base_link_l_hand",
            "left_thumb_swing_joint",
            "left_thumb_1_joint",
            "left_thumb_2_joint",
            "left_thumb_3_joint",
            "left_index_1_joint",
            "left_index_2_joint",
            "left_middle_1_joint",
            "left_middle_2_joint",
            "left_ring_1_joint",
            "left_ring_2_joint",
            "left_little_1_joint",
            "left_little_2_joint",
        ]

        RIGHT_HAND = [
            "arm2_7_link",
            "arm2_ee_link",
            "flange_r_link",
            "base_link_r_hand",
            "right_thumb_1_joint",
            "right_thumb_2_joint",
            "right_thumb_3_joint",
            "right_thumb_4_joint",
            "right_index_1_joint",
            "right_index_2_joint",
            "right_middle_1_joint",
            "right_middle_2_joint",
            "right_ring_1_joint",
            "right_ring_2_joint",
            "right_little_1_joint",
            "right_little_2_joint",
        ]
        self.left_arm_links   = {self.link_name_to_idx[n] for n in self.left_arm_joints}
        self.right_arm_links  = {self.link_name_to_idx[n] for n in self.right_arm_joints}
        self.left_hand_links  = {self.link_name_to_idx[n] for n in LEFT_HAND}
        self.right_hand_links = {self.link_name_to_idx[n] for n in RIGHT_HAND}
        self.excluded_group = {self.link_name_to_idx[n] for n in EXCLUDED_GROUP}
        self.left_excluded_pairs = {self.link_name_to_idx[n] for n in LEFT_EXCLUDED_pair}
        
        self.left_arm_encoder_indices = {self.link_name_to_idx[n] for n in self.left_arm_joints}
        self.right_arm_encoder_indices = {self.link_name_to_idx[n] for n in self.right_arm_joints}

    def get_right_joints(self):
        joints = [
            p for p, _, _, _ in p.getJointStates(self.robot_id, self.right_arm_encoder_indices)
            ]
        return joints
    
    def get_left_joints(self):
        joints = [
            p for p, _, _, _ in p.getJointStates(self.robot_id, self.left_arm_encoder_indices)
            ]
        return joints

    def move_right_arm(self, arm_angles, finger_angles=None):
        for idx, angle in zip(self.right_arm_indices, arm_angles):
            self.robot_q[idx] = angle
            
        if finger_angles is not None:
            for idx, angle in zip(self.right_finger_indices, finger_angles):
                self.robot_q[idx] = angle

    def move_left_arm(self, arm_angles, finger_angles=None):
        for idx, angle in zip(self.left_arm_indices, arm_angles):
            self.robot_q[idx] = angle

        if finger_angles is not None:
            for idx, angle in zip(self.left_finger_indices, finger_angles):
                self.robot_q[idx] = angle

    def move_jack(self, pos):
        self.robot_q[self.jack_idx] = pos

    def sync(self):
        for idx, angle in zip(self.joint_indices, self.robot_q):
            p.setJointMotorControl2(self.robot_id, idx,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=angle)

    def step(self):
        p.stepSimulation()
    
    def q_key_trigger(self):
        keys = p.getKeyboardEvents()
        if self.q_key in keys and keys[self.q_key] & p.KEY_WAS_TRIGGERED:
            print("'q' pressed. Exiting simulation.")
            return True
        return False
    
    def joints_limits(self):
        joint_indices = []
        lower_limits = []
        upper_limits = []

        for i in range(self.num_joints):
            info = p.getJointInfo(self.robot_id, i)
            joint_type = info[2]
            if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
                joint_indices.append(i)
                lower_limits.append(info[8])
                upper_limits.append(info[9])

        return joint_indices, lower_limits, upper_limits
    
    def check_self_collision_distance(
        self,
        threshold=0.02,
        l5_l7_threshold=-0.065,
        verbose=False,
        visualize=False,
        color_collisions=True):

        contacts = p.getClosestPoints(
            bodyA=self.robot_id,
            bodyB=self.robot_id,
            distance=threshold
        )

        filtered_points = []

        monitored_links = (
                self.left_arm_links |
                self.right_arm_links |
                self.left_hand_links |
                self.right_hand_links
            )
        
        colliding_links = set()
        
        info = []

        seen_pairs = set()
        collision_found = False

        left_hand = self.left_hand_links
        right_hand = self.right_hand_links
        excluded = self.excluded_group
        name_map = self.link_idx_to_name

        for c in contacts:

            linkA = c[3]
            linkB = c[4]

            if linkA == -1 or linkB == -1:
                continue
            
            if linkA == linkB:
                continue
            
            pair = (min(linkA, linkB), max(linkA, linkB))

            if pair in seen_pairs:
                continue

            seen_pairs.add(pair)
            # ---------------------------------------------------
            # Ignore parent-child collisions
            # ---------------------------------------------------
            if self.is_parent_child(linkA, linkB):
                continue

            # ---------------------------------------------------
            # Ignore collisions inside same hand
            # ---------------------------------------------------
            if (
                linkA in left_hand and
                linkB in left_hand
            ):
                continue

            if (
                linkA in right_hand and
                linkB in right_hand
            ):
                continue
            
            # # ---------------------------------------------------
            # # Ignore collisions inside same arm
            # # (optional but recommended)
            # # ---------------------------------------------------
            # if (
            #     linkA in self.left_arm_links and
            #     linkB in self.left_arm_links
            # ):
            #     continue

            # if (
            #     linkA in self.right_arm_links and
            #     linkB in self.right_arm_links
            # ):
            #     continue

            if (
                linkA not in monitored_links and
                linkB not in monitored_links
            ):
                continue
            
            # ---------------------------------------------------
            # Ignore collisions in the excluded list
            # ---------------------------------------------------
            if (
                linkA in excluded and
                linkB in excluded
            ):
                continue

            distance = c[8]

            # ---------------------------------------------------
            # Link names
            # ---------------------------------------------------
            name_a = name_map.get(linkA, str(linkA))
            name_b = name_map.get(linkB, str(linkB))

            if (
                name_a == 'arm1_5_link' and name_b == 'arm1_7_link' or
                name_b == 'arm1_5_link' and name_a == 'arm1_7_link' or
                name_a == 'arm2_5_link' and name_b == 'arm2_7_link' or
                name_b == 'arm2_5_link' and name_a == 'arm2_7_link' 
            ):
                if distance > l5_l7_threshold:
                    continue

            filtered_points.append(c)
            # ---------------------------------------------------
            # Print warning
            # ---------------------------------------------------
            if verbose:

                if distance < 0:
                    print(
                        f"[COLLISION] "
                        f"{name_a} <-> {name_b} "
                        f"(penetration={-distance:.4f} m)"
                    )
                else:
                    print(
                        f"[NEAR COLLISION] "
                        f"{name_a} <-> {name_b} "
                        f"(distance={distance:.4f} m)"
                    )

            # ---------------------------------------------------
            # Collosion flag
            # ---------------------------------------------------
            if distance < 0:
                collision_found = True
                info.append((name_a, name_b, distance))
                # Color colliding links
                if color_collisions:
                    colliding_links.update(
                        l for l in (linkA, linkB)
                        if l in monitored_links
                    )

            # ---------------------------------------------------
            # Draw debug line
            # ---------------------------------------------------
            if visualize:

                pos_a = c[5]
                pos_b = c[6]

                color = [1, 0, 0] if distance < 0 else [1, 1, 0]

                p.addUserDebugLine(
                    pos_a,
                    pos_b,
                    color,
                    3,
                    lifeTime=0.05
                )

        if color_collisions:
        # Reset only links that were red but are not colliding now
            for link in self.colored_links - colliding_links:
                try:
                    p.changeVisualShape(self.robot_id, link, rgbaColor=self.normal_color)
                except:
                    pass
            # Set new colliding links to red
            for link in colliding_links - self.colored_links:
                try:
                    p.changeVisualShape(self.robot_id, link, rgbaColor=[1,0,0,1])
                except:
                    pass
            # Update the set of colored links
            self.colored_links = colliding_links

        return collision_found, filtered_points, info
    
    def check_arm_self_collision(self, threshold = 0.02, arm='left'):
        if arm.lower() == 'left':
            monitored_links = self.left_arm_links
        elif arm.lower() == 'right':
            monitored_links = self.right_arm_links
        else:
            print("Monitored Links can be 'Left' of 'Right")
            return None, None
        
        contacts = p.getClosestPoints(
            bodyA=self.robot_id,
            bodyB=self.robot_id,
            distance=threshold
        )

        filtered_points = []  
        info = []
        colliding_links = set()

        for c in contacts:

            linkA = c[3]
            linkB = c[4]

            if linkA == -1 or linkB == -1:
                continue
            
            if linkA == linkB:
                continue

            if (
                linkA not in monitored_links or
                linkB not in monitored_links
            ):
                continue

            # ---------------------------------------------------
            # Ignore collisions in the excluded list
            # ---------------------------------------------------
            if (
                linkA in self.excluded_group and
                linkB in self.excluded_group
            ):
                continue

            # ---------------------------------------------------
            # Ignore parent-child collisions
            # ---------------------------------------------------
            if self.is_parent_child(linkA, linkB):
                continue

            distance = c[8]
            

            # ---------------------------------------------------
            # Link names
            # ---------------------------------------------------
            name_a = self.link_idx_to_name.get(linkA, str(linkA))
            name_b = self.link_idx_to_name.get(linkB, str(linkB))

            if (
                name_a == 'arm1_5_link' and name_b == 'arm1_7_link' or
                name_b == 'arm1_5_link' and name_a == 'arm1_7_link' or
                name_a == 'arm2_5_link' and name_b == 'arm2_7_link' or
                name_b == 'arm2_5_link' and name_a == 'arm2_7_link' 
            ):
                print(name_a, name_b, distance, (distance > -0.065))
                if distance > -0.065:
                    continue
            filtered_points.append(c)

            # ---------------------------------------------------
            # Color colliding links
            # ---------------------------------------------------
            if distance < 0:
                if linkA in monitored_links:
                    colliding_links.add(linkA)
                if linkB in monitored_links:
                    colliding_links.add(linkB)
                info.append((name_a, name_b, distance))

        for link in self.colored_links - colliding_links:
            try:
                p.changeVisualShape(self.robot_id, link, rgbaColor=self.normal_color)
            except:
                pass
        # Set new colliding links to red
        for link in colliding_links - self.colored_links:
            try:
                p.changeVisualShape(self.robot_id, link, rgbaColor=[1,0,0,1])
            except:
                pass
        # Update the set of colored links
        self.colored_links = colliding_links
            
        collision_found = any(c[8] < 0 for c in filtered_points)

        debug = (filtered_points, info)
        return collision_found, debug

def main():
    robot = RobotSim()
    print("Moving robot")
    robot.move_jack(0.3)
    right_posture = [+0.5, -2.0, 0.0, 1.2, 0.0, 0.8, 0.0]
    robot.move_right_arm(right_posture)
    left_posture = [-0.2, -0.5, 0.0, 1.2, 0.0, 1.2, np.deg2rad(90)]
    robot.move_left_arm(left_posture)
    
    loop_time = []
    try:
        counter = 0

        while True: 
            start = time.perf_counter()
            robot.sync()
            robot.step()

            counter += 1

            # Check every 20 frames
            if counter % 20 == 0:

                collision, pts, info = robot.check_self_collision_distance(
                    threshold=0.02,
                    color_collisions=True,
                    visualize=False
                    )
                
                if collision:
                    for v in info:
                        print(v)
                avr_time = np.average(loop_time)
                print(f"Loop time :{(avr_time):.3f} sec")

            time.sleep(1./240.)

            if counter == 1000:
                right_posture = [+0.2, -0.5, 0.0, 1.2, 0.0, 0.8, np.deg2rad(90)]
                robot.move_right_arm(right_posture)
                left_posture = [-0.2, -0.5, 0.0, 1.2, 0.0, 0.8, -np.deg2rad(90)]
                robot.move_left_arm(left_posture)
            if robot.q_key_trigger():
                break
            loop_time.append(time.perf_counter() - start)
        robot.disconnect()
    finally:
        print("Program finished")
if __name__ == "__main__":
    main()