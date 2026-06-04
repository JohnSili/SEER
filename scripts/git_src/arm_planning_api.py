      
import requests
import json
import time
import numpy as np


class ArmPlanningAPI:
    def __init__(self, hostname="localhost", port=8088):
        print("ArmPlanningAPI")
        self.api_path = "/universalPost"
        self.base_url = f"http://{hostname}:{port}"
        print(f"[INFO] Whole URL set to: {self.base_url + self.api_path}",)
    
    def post(self, node: str, data: dict, verbose=0):
        json_data = json.dumps(data)
        params = {
            "node_name": node,
            "service_name": "serviceDispatcher",
            "route_length": len(json_data)
        }
        response = requests.post(self.base_url + self.api_path, json_data, params=params)
        if response.status_code == 200:
            if verbose > 0:
                print("post response:", response.text)
            route_length = response.headers.get('route_length')
            
            if route_length is not None:
                route_length = int(route_length)
                content = response.text
                
                # 将内容分成两部分
                if route_length <= len(content):
                    part1 = content[:route_length]
                    part2 = content[route_length:]
                    
                    # 输出分割后的内容
                    # print("Part 1:", part1)
                    # print("Part 2:", part2)
                    return part2
                else:
                    print("Error: route_length is greater than content length.")
            else:
                print("Error: 'route_length' header not found.")
        else:
            print("Error: Request failed with status code", response.status_code)
        return None
    
    def getRightJoint(self):
        data = {
            "func_name": "getCurrentState",
            "command": "right_joints_pos"
        }
        state = self.post("ArmPlanning", data)
        status_data = json.loads(state)
        arm_state = status_data['status']
        return arm_state["right_joints_pos"]
    
    def getRightEE(self):
        data = {
            "func_name": "getCurrentState",
            "command": "right_ee_pose"
        }
        state = self.post("ArmPlanning", data)
        status_data = json.loads(state)
        arm_state = status_data['status']
        return arm_state["right_ee_pose"]
    
    def getLeftEE(self):
        data = {
            "func_name": "getCurrentState",
            "command": "left_ee_pose"
        }
        state = self.post("ArmPlanning", data)
        status_data = json.loads(state)
        arm_state = status_data['status']
        return arm_state["left_ee_pose"]
    
    def getLeftJoint(self):
        data = {
            "func_name": "getCurrentState",
            "command": "left_joints_pos"
        }
        state = self.post("ArmPlanning", data)
        status_data = json.loads(state)
        arm_state = status_data['status']
        return arm_state["left_joints_pos"]
    
    # jack_pos
    def getJackPos(self):
        data = {
            "func_name": "getCurrentState",
            "command": "jack_pos"
        }
        state = self.post("ArmPlanning", data)
        status_data = json.loads(state)
        return status_data['status']['jack_pos']
    
    # head_pos
    def getHeadPos(self):
        data = {
            "func_name": "getCurrentState",
            "command": "head_joints_pos"
        }
        state = self.post("ArmPlanning", data)
        status_data = json.loads(state)
        return status_data['status']['head_joints_pos']
    
    def getRightVel(self):
        data = {
            "func_name": "getCurrentState",
            "command": "right_joints_vel"
        }
        state = self.post("ArmPlanning", data)
        status_data = json.loads(state)
        arm_state = status_data['status']
        return arm_state["right_joints_vel"]
    
    def getLeftVel(self):
        data = {
            "func_name": "getCurrentState",
            "command": "left_joints_vel"
        }
        state = self.post("ArmPlanning", data)
        status_data = json.loads(state)
        arm_state = status_data['status']
        return arm_state["left_joints_vel"]
    
    def getRightEffort(self):
        data = {
            "func_name": "getCurrentState",
            "command": "right_joints_effort"
        }
        state = self.post("ArmPlanning", data)
        status_data = json.loads(state)
        arm_state = status_data['status']
        return arm_state["right_joints_effort"]
    
    def getLeftEffort(self):
        data = {
            "func_name": "getCurrentState",
            "command": "left_joints_effort"
        }
        state = self.post("ArmPlanning", data)
        status_data = json.loads(state)
        arm_state = status_data['status']
        return arm_state["left_joints_effort"]
    
    def getRightALL(self):
        data = {
            "func_name": "getCurrentState",
            "command": "right_joints_state"
        }
        state = self.post("ArmPlanning", data)
        status_data = json.loads(state)
        arm_state = status_data['status']
        return arm_state["right_joints_state"]
    
    def getLeftALL(self):
        data = {
            "func_name": "getCurrentState",
            "command": "left_joints_state"
        }
        state = self.post("ArmPlanning", data)
        status_data = json.loads(state)
        arm_state = status_data['status']
        return arm_state["left_joints_state"]
    
    def getALL(self):
        data = {
            "func_name": "getCurrentState",
            # "command": "all_joints_state"
            "command": "all_state"
        }
        state = self.post("ArmPlanning", data)
        status_data = json.loads(state)
        arm_state = status_data.get('status', dict())
        return arm_state
    
    def controlJack(self, joint, vel=0.7, acc=0.3):
        data = {
            "func_name": "robotControl",
            "move_group_name": "jack",
            "move_jack": joint,
            "velocity": vel,
            "acceleration": acc
        }
        state = self.post("ArmPlanning", data)
        status_data = json.loads(state)
        arm_state = status_data['status']
        print("controlJack:", status_data)
        return arm_state
    
    # def controlChassis(self, dist=1.0, vx=0.5, vy=0.0, loc_mode=0, task_id=None):
    # # """
    # # 控制AGV底盘移动（基于里程计或定位模式）
    
    # # Args:
    # #     dist (float): 移动距离（m），正数前进，负数后退（loc_mode=0时生效）
    # #     vx (float): X方向速度（m/s），正数前进，负数后退
    # #     vy (float): Y方向速度（m/s），正数左移，负数右移
    # #     loc_mode (int): 0=里程计模式（相对移动），1=定位模式（绝对坐标移动）
    # #     task_id (str): 任务ID（可选），默认自动生成 "odo"+时间戳
    
    # # Returns:
    # #     dict: 底盘执行状态（API返回的JSON数据）
    # # """
    #     if task_id is None:
    #         task_id = "odo" + str(time.time())  # 默认任务ID
    
    #     data = {
    #             "func_name": "updateMoveTaskList",
    #             "move_task_list": [{
    #                 "task_id": task_id,
    #                 "id": "SELF_POSITION",
    #                 "source_id": "SELF_POSITION",
    #                 "skill_name": "GoByOdometer",
    #                 "loc_mode": loc_mode,  # 0=里程计模式，1=定位模式
    #                 "dist": dist,          # 移动距离（m）
    #                 "vx": vx,              # X轴速度（m/s）
    #                 "vy": vy               # Y轴速度（m/s）
    #             }]
    #         }
    
    #     # 发送请求并解析返回状态
    #     # state = self.post("ArmPlanning", data)
    #     # status_data = json.loads(state)
    #     # print("controlChassis:", status_data)
    #     # return status_data
    #     state = self.post("ArmPlanning", data)
    #     status_data = json.loads(state)
    #     chassiss_state = status_data['status']
    #     print("controlChassis:", chassiss_state)
    #     return chassiss_state

    def controlChassisRotation(self, move_angle=1.0, vw=0.17, loc_mode=0, task_id = None):

        task_id = "odo"+str(time.time()) if task_id is None else task_id
        
        self.post(
            node="Tracking",
            data={
                "func_name": "updateMoveTaskList",
                "move_task_list": [{
                    "task_id": task_id,
                    "id": "SELF_POSITION",
                    "source_id": "SELF_POSITION",
                    "skill_name": "GoByOdometer",
                    "loc_mode": loc_mode,
                    "move_angle": move_angle,
                    "vw": vw,
                }]
            }
        )
        return task_id

    def waitMoveChassis(self, task_id=None):
        
        task_id = "odo"+str(time.time()) if task_id is None else task_id

        time.sleep(0.1)
        while True:
            data = {
                    "func_name": "taskStatus",
                    "task_id":task_id,
            }
            state = self.post("Task", data)
            status_data = json.loads(state)
            arm_state = status_data['status']
            print(f"State is: {arm_state}.")
            if arm_state == "RUNNING":
                time.sleep(0.1)
            elif arm_state == "IDLE" or arm_state == 'StatusNone':
                time.sleep(0.1)
                break
            else:
                time.sleep(0.1)

    def controlChassis(self, dist=0.5, vx=0.2, vy=0.0, loc_mode=0, task_id=None):

        task_id = "odo"+str(time.time()) if task_id is None else task_id

        self.post(
            node="Tracking",  # 修正参数名从 node_name 改为 node
            data={
                "func_name": "updateMoveTaskList",
                "move_task_list": [{
                    "task_id": task_id,
                    "id": "SELF_POSITION",
                    "source_id": "SELF_POSITION",
                    "skill_name": "GoByOdometer",
                    "loc_mode": loc_mode,  # 0=里程计模式，1=定位模式
                    "dist": dist,
                    "vx": vx,
                    "vy": vy
                }]
            }
        )

        return task_id
    
    def controlWaist(self, joint, vel=0.5, acc=0.3):
        data = {
            "func_name": "robotControl",
            "move_group_name": "waist",
            "move_waist": joint,
            "velocity": vel,
            "acceleration": acc
            
        }
        state = self.post("ArmPlanning", data)
        status_data = json.loads(state)
        arm_state = status_data['status']
        print("controlWaist:", status_data)
        return arm_state
    
    def controlRight(self, joint):
        data = {
            "func_name": "robotControl",
            "move_group_name": "right_arm",
            "move_right_joints": joint,
            "velocity": 0.5,
            "acceleration": 0.2
            
        }
        state = self.post("ArmPlanning", data)
        status_data = json.loads(state)
        arm_state = status_data['status']
        return arm_state
    
    def controlLeft(self, joint, vel = 0.5, acc = 0.2):
        data = {
            "func_name": "robotControl",
            "move_group_name": "left_arm",
            "move_left_joints": joint,
            "velocity": vel,
            "acceleration": acc
            
        }
        state = self.post("ArmPlanning", data)
        status_data = json.loads(state)
        arm_state = status_data['status']
        return arm_state
    
    def controlHead(self, joint):
        data = {
            "func_name": "robotControl",
            "move_group_name": "head",
            "move_head": joint,
            "velocity": 0.3,
            "acceleration": 0.3
        }
        state = self.post("ArmPlanning", data)
        status_data = json.loads(state)
        arm_state = status_data['status']
        return arm_state
    
    def controlGripper(self, angle=0, velocity=50):
        data = {
            "func_name": "sendControl",
            "band_name": ["Gripper-000", "Gripper-001"],
            "param": [{
                "finger_name": "thumb",
                "bend_angle": angle,
                "velocity": velocity
              }]
        }
        state = self.post("GripManager", data)
        print("controlGripper:", state)
        
    def getGripperPos(self):
        data = {
            "func_name": "getStatus",
            "band_name": ["Gripper-000", "Gripper-001"],
            "finger_names": ["thumb", "index", "middle"]
        }
        state = self.post("GripManager", data)
        print("getGripperPos:", state)
    
    def controlRecognizer(self, reco_file='default.srec', tries=1, task_ctrl=10):
        data = {
            "func_name": "ctrlRecognizer",
            "reco_file": reco_file,
            "task_ctrl": task_ctrl,  # 10:开始识别, 12:暂停识别, 13:结束识别
            "tries": tries
        }
        state = self.post("AppRecognition", data)
        print("controlRecognizer:", state)
        return state
    
    def getRecResult(self, task_id):
        data = {
            "func_name": "getRecResult",
            "task_id": task_id,
        }
        state = self.post("AppRecognition", data)
        print("getRecResult:", state)
        return state
    
    def controlDualArm(self, right_joints, left_joints):
        data = {
            "func_name": "robotControl",
            "move_group_name": "dual_arm",
            "move_right_joints": right_joints,
            "move_left_joints": left_joints,
            "velocity": 1.0,
            "acceleration": 0.8
        }
        state = self.post("ArmPlanning", data)
        status_data = json.loads(state)
        arm_state = status_data['status']
        return arm_state
    
    def controlRightPose(self, pose):
        data = {
            "func_name": "robotControl",
            "move_group_name": "right_arm",
            "move_right_to_pose": pose,
            "velocity": 0.3,
            "acceleration": 0.2
            
        }
        state = self.post("ArmPlanning", data)
        status_data = json.loads(state)
        arm_state = status_data['status']
        return arm_state
    
    def controlLeftPose(self, pose):
        data = {
            "func_name": "robotControl",
            "move_group_name": "left_arm",
            "move_left_to_pose": pose,
            "velocity": 0.3,
            "acceleration": 0.2
        }
        state = self.post("ArmPlanning", data)
        status_data = json.loads(state)
        arm_state = status_data['status']
        return arm_state
    
    def controlDualPose(self, right_pose, left_pose):
        data = {
            "func_name": "robotControl",
            "move_group_name": "dual_arm",
            "move_right_to_pose": right_pose,
            "move_left_to_pose": left_pose,
            "velocity": 0.3,
            "acceleration": 0.2
        }
        state = self.post("ArmPlanning", data)
        status_data = json.loads(state)
        arm_state = status_data['status']
        return arm_state
    
    def controlHeadDualArm(self, head_joints, right_joints, left_joints):
        data = {
            "func_name": "robotControl",
            "move_group_name": "head_dual_arm",
            "move_head": head_joints,
            "move_right_joints": right_joints,
            "move_left_joints": left_joints,
            "velocity": 1.3,
            "acceleration": 0.8
        }
        state = self.post("ArmPlanning", data)
        status_data = json.loads(state)
        arm_state = status_data['status']
        return arm_state
    
    def setJackZero(self):
        data = {
            "func_name": "robotControl",
            "jack_motor_set_zero": "set_jack_zero"
        }
        state = self.post("ArmPlanning", data)
        status_data = json.loads(state)
        arm_state = status_data['status']
        return arm_state
    
    def openArmCollision(self):
        data = {
            "func_name": "robotControl",
            "set_arm_collision": True
        }
        state = self.post("ArmPlanning", data)
        status_data = json.loads(state)
        arm_state = status_data['status']
        return arm_state
    
    def closeArmCollision(self):
        data = {
            "func_name": "robotControl",
            "set_arm_collision": False
        }
        state = self.post("ArmPlanning", data)
        status_data = json.loads(state)
        arm_state = status_data['status']
        return arm_state
    
    def waitMove(self, verbose=False):
        time.sleep(0.1)
        while True:
            data = {
                "func_name": "getCurrentState",
                "command": "get_arm_state"
            }
            state = self.post("ArmPlanning", data)
            status_data = json.loads(state)
            arm_state = status_data['status']
            
            if verbose: 
                print("arm_state:", arm_state)
            
            if arm_state["get_arm_state"] == "RUNNING":
                time.sleep(0.1)
            elif arm_state["get_arm_state"] == "IDLE":
                print("arm_state:", arm_state)
                time.sleep(0.1)
                break
            else:
                print("arm_state:", arm_state)

    def isMoving(self):
        data = {
            "func_name": "getCurrentState",
            "command": "get_arm_state"
        }
        state = self.post("ArmPlanning", data)
        status_data = json.loads(state)
        arm_state = status_data['status']
        
        if arm_state["get_arm_state"] == "RUNNING":
            return True
        elif arm_state["get_arm_state"] == "IDLE":
            return False
        else:
            return None
    
    # arm_name [str] : right_arm left_arm dual_arm
    def dragModeRecord(self, arm_name, trajectory_name):
        data = {
            "func_name": "robotControl",
            "drag_teach_mode": True,
            "arm_name": arm_name,
            "record": 1,
            "trajectory_name": trajectory_name
        }
        state = self.post("ArmPlanning", data)
    
    def dragMode(self, arm_name):
        data = {
            "func_name": "robotControl",
            "drag_teach_mode": True,
            "arm_name": arm_name,
            "record": 0,
            "trajectory_name": ""
        }
        state = self.post("ArmPlanning", data)
    
    def outdragMode(self):
        data = {
            "func_name": "robotControl",
            "drag_teach_mode": False
        }
        state = self.post("ArmPlanning", data)
    
    def dragReplay(self, arm_name, trajectory_name):
        data = {
            "func_name": "robotControl",
            "replay_drag_trajectory": arm_name,
            "trajectory_name": trajectory_name
        }
        state = self.post("ArmPlanning", data)
    
    def switch_real_time_mode(self, group_name):
        data = {
            "func_name": "robotControl",
            "group_name": group_name,
            "real_time_mode": True
        }
        self.post("ArmPlanning", data)
    
    def out_switch_real_time_mode(self):
        data = {
            "func_name": "robotControl",
            "real_time_mode": False
        }
        self.post("ArmPlanning", data)
    
    def left_real_time_trajectory(self, joints):
        data = {
            "func_name": "robotControl",
            "left_joints_cmd": joints,
        }
        self.post("ArmPlanning", data)
    
    def right_real_time_trajectory(self, joints):
        data = {
            "func_name": "robotControl",
            "right_joints_cmd": joints,
        }
        self.post("ArmPlanning", data)
    
    def dual_real_time_trajectory(self, joints):
        data = {
            "func_name": "robotControl",
            "dual_joints_cmd": joints,
        }
        self.post("ArmPlanning", data)
    
    def ExcitationTrajectory(self):
        data = {
            "func_name": "robotControl",
            "excitation_trajectory": True,
            "amplitude_sine": [
                [0.0769846767654865, 0.147750568199432, -0.492885886306759, 0.754678179626206, 1.01656270825411,
                 -0.285096624439121, -0.285277675580051, -0.540229560803672],
                [0.959907505401296, 0.0131979540981403, -0.0688816292272123, 0.112777184224310, 0.0408516722490407,
                 0.110979433478667, -0.245987008070171, 0.302476082577314],
                [3.26086165051952, -0.0901713880476215, 0.265516499266602, 0.951845037208443, 0.175356630297162,
                 -0.0927240045921166, -1.05937626012659, 0.0603984170158273],
                [-0.137188635302063, 0.0541532061425681, 0.287292201503073, -0.0328102627422568, 0.137432344697275,
                 -0.975665399973818, -0.0337343512866358, 0.639704967072883],
                [-1.32339613309102, 0.329238069422633, -0.432408597386512, 0.343230343149307, -0.658926874699022,
                 -0.373181589736196, -0.431374194000121, 0.755583891356382],
                [-0.444523086070529, 0.0673385796463732, -0.0871776037012712, -0.0413150691340519, -0.0587552665515011,
                 0.233375577987157, 0.0920726799874045, -0.189019550602337],
                [0.554805639337095, 0.122284775639480, 0.264282047870730, 0.0604394594696638, 0.325377991045677,
                 -0.0477909123183056, -0.628553240384681, -0.0597264646286733]
            ],
            "amplitude_cosine": [
                [0.439896638727794, 0.0511963694962525, 0.198926640037784, -0.229891176143516, -0.266253537894379,
                 0.262971483928943, -0.0453812600252814, 0.0741505151462745],
                [0.945430708758303, 0.163554648086067, 0.182586905427702, -0.0908168032675452, 0.210146910887288,
                 -0.970500323789440, 0.0297241421098817, 0.593644928644327],
                [-0.535083480625224, 0.0926082456361234, -0.328882304592768, 0.305419503811220, -0.466485716346588,
                 -0.113205705369925, 0.286284270801412, 0.0312014696105918],
                [0.0688202050834786, 0.0645982816657266, 0.0666243591981304, -0.380223794248660, 0.127321951516083,
                 0.0126950878887590, -0.188959004539680, 0.201434874616326],
                [0.0361842131848624, 0.162949261480300, -0.253045128134895, 0.176115847545153, 0.190285937375042,
                 -0.432586681711330, -0.217881746177467, 0.381651819968799],
                [-0.250925896792077, -0.0365265374904148, -0.0701182411158502, 0.0926360814057475, -0.350454183206802,
                 0.118065668020491, -0.126686458597806, 0.209533377803771],
                [0.305987735659865, 0.0683325441621368, 0.127868190853668, -0.444050516357571, 0.0656408586268116,
                 -0.0797086932094321, 0.329752409508725, -0.0843931863224280]
            ],
            "amplitude_fifth": [
                [2.23307703136328, -0.392486385715631, -0.0776602864633596, 0.00953831227217544, -0.000304337199245751,
                 2.90730656085654e-06],
                [4.88970482662164, -1.22532119473138, -0.171949266531092, 0.0250779643768660, -0.000871788737663096,
                 9.07645329430658e-06],
                [-2.81426718484550, -3.47170658154122, 0.0914095652520060, 0.0324805465558800, -0.00182715969502068,
                 2.57163450484537e-05],
                [0.258007532252102, 0.0608159298889742, 0.0127191652109489, -0.00152367690171854, 4.79190335060405e-05,
                 -4.50488369547966e-07],
                [0.285902839651230, 1.79123508498454, -0.0200394524688913, -0.0185666485574577, 0.000972864544470418,
                 -1.32684080369225e-05],
                [-1.48836535408903, 0.428003738438755, 0.0437942691818184, -0.00767521503921851, 0.000286440153779107,
                 -3.17039806250930e-06],
                [1.47138348867474, -0.591119296030986, -0.0558291216740226, 0.0102899336230570, -0.000390431966321690,
                 4.37866145208149e-06]
            ]
        }
        self.post("ArmPlanning", data)