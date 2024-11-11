import pybullet as p
import pybullet_data
import time
import math
import numpy as np

# PyBullet 연결 초기화
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 시뮬레이션 환경 설정

plane_id = p.loadURDF("plane.urdf")  # 바닥면 추가

# 카메라 위치 설정 (옆모습 보기)
camera_distance = 2.0     # 카메라와 로봇 간 거리
camera_yaw = 0           # 카메라의 좌우 회전 (90도로 설정하면 옆모습)
camera_pitch = -20        # 카메라의 위아래 각도
camera_target_position = [0, 0, 1]  # 카메라가 바라볼 위치 (로봇 위치)

p.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, camera_target_position)

# 로봇 모델 로드 (robot_model.urdf 파일)
robot_id = p.loadURDF("robot_model.urdf", [0, 0, 0.7], useFixedBase=False)  # z축 이동을 허용하도록 로드

joints = {}
joint_names = ['abad_joint', 'hip_joint', 'knee_joint']
leg_prefixes = ['front_left', 'front_right', 'back_left', 'back_right']

# 로봇의 모든 관절 ID와 이름 확인
print("로봇의 모든 관절 정보:")
# joint1_id = None
# joint2_id = None
# abad_id = None
for i in range(p.getNumJoints(robot_id)):
    info = p.getJointInfo(robot_id, i)
    joint_name = info[1].decode('utf-8')
    print(f"Joint {i}: {joint_name}")  # 각 관절의 이름과 ID 출력

    joints[joint_name] = i

print(f'joints: {joints}')

def cal(l_1, l_2, a, b):
    theta_2 = math.acos((a**2 + b**2  - l_1*l_1 - l_2*l_2)/(2*l_1*l_2))
    k_2 = l_2 * math.cos(theta_2) + l_1
    k_1 = l_2 * math.sin(theta_2)
    theta_1 = math.asin((a * k_2 + b* k_1)/(k_1**2 + k_2 ** 2))
    return theta_1, theta_2

def cal2(z_dif, y_dif, prev_x, prev_y, l_1, l_2): #dif는 양수
    length = math.sqrt((prev_y-y_dif)**2 + z_dif**2)
    #print(f'prev: {prev_y}, length: {length}, z_dif:{z_dif}')
    theta1, theta2 = cal(l_1, l_2, prev_x, length )
    theta3 = math.atan(z_dif/(prev_y-y_dif))

    return theta1, theta2, theta3

# 기본 모터 제어 파라미터 설정
theta_1 = 45
theta_2 = 90
l_1 = 0.3
l_2 = 0.3
DEFAULT_Y = l_1 * (3 ** (1/2))-0.02
DEFAULT_X = 0
Y_DIFF = 0.05
MAX_Y = DEFAULT_Y - Y_DIFF
X_DIFF = 0.05
B1 = 0.05
B2 = 0.02
a = DEFAULT_X
b = DEFAULT_Y
step_time = 0.01 
theta_1, theta_2 = cal(l_1, l_2, a, b)
default_position_hip = -theta_1
default_position_knee = theta_2

target_position_hip = default_position_hip # 힙 관절의 초기 각도
target_position_knee = default_position_knee # 무릎 관절의 초기 각도

p.setGravity(0, 0, 0)
# for i in range(240):
#     for j in range(4):
#                 p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joints[leg_prefixes[j]+'_'+joint_names[1]],
#                                 controlMode=p.POSITION_CONTROL, targetPosition=target_position_hip)
#                 p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joints[leg_prefixes[j]+'_'+joint_names[2]],
#                                 controlMode=p.POSITION_CONTROL, targetPosition=target_position_knee)
#                 p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joints[leg_prefixes[j]+'_'+joint_names[0]],
#                             controlMode=p.POSITION_CONTROL, targetPosition=0)
#     p.stepSimulation()
#     time.sleep(2/240)
p.setGravity(0, 0, -9.8)

def box_step_forward():
    a_list = [0, -X_DIFF, -X_DIFF, 0]
    b_list = [DEFAULT_Y - Y_DIFF, DEFAULT_Y - Y_DIFF, DEFAULT_Y, DEFAULT_Y]
    joint1_id = None
    joint2_id = None
    for i in range(4):
            theta_1, theta_2 = cal(l_1, l_2, a_list[i], b_list[i])
            print('theta1:', math.degrees(theta_1))
            print("theta2:",math.degrees(theta_2))
            target_position_hip = -theta_1  # 힙 관절 각도를 전진 방향으로 설정
            target_position_knee = theta_2  # 무릎 관절 각도를 전진 방향으로 설정
            p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joint1_id,
                            controlMode=p.POSITION_CONTROL, targetPosition=target_position_hip)
            p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joint2_id,
                            controlMode=p.POSITION_CONTROL, targetPosition=target_position_knee)
            for i in range(25):
                #time.sleep(1.0/50)
                p.stepSimulation()
                time.sleep(0.1 / 25)
            time.sleep(1.0 / 2.0)

def box_step_backward():
    a_list = [0, X_DIFF, X_DIFF, 0]
    b_list = [DEFAULT_Y - Y_DIFF, DEFAULT_Y - Y_DIFF, DEFAULT_Y,  DEFAULT_Y]
    joint1_id = None
    joint2_id = None
    for i in range(4):
            theta_1, theta_2 = cal(l_1, l_2, a_list[i], b_list[i])
            print('theta1:', math.degrees(theta_1))
            print("theta2:",math.degrees(theta_2))
            target_position_hip = -theta_1  # 힙 관절 각도를 전진 방향으로 설정
            target_position_knee = theta_2  # 무릎 관절 각도를 전진 방향으로 설정
            p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joint1_id,
                            controlMode=p.POSITION_CONTROL, targetPosition=target_position_hip)
            p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joint2_id,
                            controlMode=p.POSITION_CONTROL, targetPosition=target_position_knee)
            for i in range(25):
                #time.sleep(1.0/50)
                p.stepSimulation()
                time.sleep(0.1 / 25)
            time.sleep(1.0 / 2.0)

def trot_forward(step):
    cycle_count = 20  # 한 번의 스텝을 나누는 횟수

    if step == 1:
        for i in np.linspace(0, X_DIFF, cycle_count):
            phase = i * math.pi/X_DIFF  # 0에서 pi까지의 구간을 sin으로 나눔
            x_move = i  # sin 곡선을 따라 다리 높이 설정
            y_move = B1 * math.sin(phase)  # sin 곡선을 따라 전방 이동

            theta1, theta2 = cal(l_1=l_1, l_2=l_2, a=DEFAULT_X - x_move, b=DEFAULT_Y - y_move)
            
            # Front Left (FL) and Back Right (BR)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[2]], p.POSITION_CONTROL, theta2)

            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[2]], p.POSITION_CONTROL, theta2)

            for i in range(10):
                p.stepSimulation()
            time.sleep(step_time)
    elif step % 2 == 1:
        for i in np.linspace(0, X_DIFF, cycle_count):
            phase = i * math.pi/X_DIFF  # 0에서 pi까지의 구간을 sin으로 나눔
            x_move = i  # sin 곡선을 따라 다리 높이 설정
            y_move = B1 * math.sin(phase)  # sin 곡선을 따라 전방 이동

            theta1, theta2 = cal(l_1=l_1, l_2=l_2, a=DEFAULT_X - x_move, b=DEFAULT_Y - y_move)
            
            # Front Left (FL) and Back Right (BR)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[2]], p.POSITION_CONTROL, theta2)

            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[2]], p.POSITION_CONTROL, theta2)

            x_move = X_DIFF - i  # sin 곡선을 따라 다리 높이 설정
            y_move = B2 * math.sin(phase)  # sin 곡선을 따라 전방 이동
            theta1, theta2 = cal(l_1=l_1, l_2=l_2, a=DEFAULT_X - x_move, b=DEFAULT_Y + y_move)
            p.setJointMotorControl2(robot_id, joints['front_right_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['front_right_' + joint_names[2]], p.POSITION_CONTROL, theta2)

            p.setJointMotorControl2(robot_id, joints['back_left_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['back_left_' + joint_names[2]], p.POSITION_CONTROL, theta2)
            for i in range(10):
                p.stepSimulation()
            time.sleep(step_time)
    elif step % 2 == 0:
    # Step 2: 반대편 다리 교체
        for i in np.linspace(0, X_DIFF, cycle_count):
            phase = i *math.pi/X_DIFF  # 0에서 pi까지의 구간을 sin으로 나눔
            x_move = i  # sin 곡선을 따라 다리 높이 설정
            y_move = B1 * math.sin(phase)  # sin 곡선을 따라 전방 이동

            theta1, theta2 = cal(l_1=l_1, l_2=l_2, a=DEFAULT_X-x_move, b= DEFAULT_Y- y_move)
            
            # Front Right (FR) and Back Left (BL)
            p.setJointMotorControl2(robot_id, joints['front_right_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['front_right_' + joint_names[2]], p.POSITION_CONTROL, theta2)

            p.setJointMotorControl2(robot_id, joints['back_left_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['back_left_' + joint_names[2]], p.POSITION_CONTROL, theta2)

            x_move = X_DIFF - i
            y_move = B2 * math.sin(phase)
            theta1, theta2 = cal(l_1=l_1, l_2=l_2, a=DEFAULT_X-x_move, b= DEFAULT_Y+ y_move)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[2]], p.POSITION_CONTROL, theta2)

            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[2]], p.POSITION_CONTROL, theta2)

            for i in range(10):
                p.stepSimulation()
            time.sleep(step_time)

def trot_backward(step):
    cycle_count = 20  # 한 번의 스텝을 나누는 횟수

    if step == 1:
        for i in np.linspace(0, X_DIFF, cycle_count):
            phase = i * math.pi/X_DIFF  # 0에서 pi까지의 구간을 sin으로 나눔
            x_move = i  # sin 곡선을 따라 다리 높이 설정
            y_move = B1 * math.sin(phase)  # sin 곡선을 따라 전방 이동

            theta1, theta2 = cal(l_1=l_1, l_2=l_2, a=DEFAULT_X + x_move, b=DEFAULT_Y - y_move)
            
            # Front Left (FL) and Back Right (BR)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[2]], p.POSITION_CONTROL, theta2)

            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[2]], p.POSITION_CONTROL, theta2)

            for i in range(10):
                p.stepSimulation()
            time.sleep(step_time)
    elif step % 2 == 1:
        for i in np.linspace(0, X_DIFF, cycle_count):
            phase = i * math.pi/X_DIFF  # 0에서 pi까지의 구간을 sin으로 나눔
            x_move = i  # sin 곡선을 따라 다리 높이 설정
            y_move = B1 * math.sin(phase)  # sin 곡선을 따라 전방 이동

            theta1, theta2 = cal(l_1=l_1, l_2=l_2, a=DEFAULT_X + x_move, b=DEFAULT_Y - y_move)
            
            # Front Left (FL) and Back Right (BR)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[2]], p.POSITION_CONTROL, theta2)

            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[2]], p.POSITION_CONTROL, theta2)

            x_move = X_DIFF - i  # sin 곡선을 따라 다리 높이 설정
            y_move = B2 * math.sin(phase)  # sin 곡선을 따라 전방 이동
            theta1, theta2 = cal(l_1=l_1, l_2=l_2, a=DEFAULT_X + x_move, b=DEFAULT_Y + y_move)
            p.setJointMotorControl2(robot_id, joints['front_right_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['front_right_' + joint_names[2]], p.POSITION_CONTROL, theta2)

            p.setJointMotorControl2(robot_id, joints['back_left_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['back_left_' + joint_names[2]], p.POSITION_CONTROL, theta2)
            for i in range(10):
                p.stepSimulation()
            time.sleep(step_time)
    elif step % 2 == 0:
    # Step 2: 반대편 다리 교체
        for i in np.linspace(0, X_DIFF, cycle_count):
            phase = i *math.pi/X_DIFF  # 0에서 pi까지의 구간을 sin으로 나눔
            x_move = i  # sin 곡선을 따라 다리 높이 설정
            y_move = B1 * math.sin(phase)  # sin 곡선을 따라 전방 이동

            theta1, theta2 = cal(l_1=l_1, l_2=l_2, a=DEFAULT_X + x_move, b= DEFAULT_Y- y_move)
            
            # Front Right (FR) and Back Left (BL)
            p.setJointMotorControl2(robot_id, joints['front_right_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['front_right_' + joint_names[2]], p.POSITION_CONTROL, theta2)

            p.setJointMotorControl2(robot_id, joints['back_left_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['back_left_' + joint_names[2]], p.POSITION_CONTROL, theta2)

            x_move = X_DIFF - i
            y_move = B2 * math.sin(phase)
            theta1, theta2 = cal(l_1=l_1, l_2=l_2, a=DEFAULT_X + x_move, b= DEFAULT_Y+ y_move)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[2]], p.POSITION_CONTROL, theta2)

            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[2]], p.POSITION_CONTROL, theta2)

            for i in range(10):
                p.stepSimulation()
            time.sleep(step_time)

def trot_left(step):
    cycle_count = 20  # 한 번의 스텝을 나누는 횟수

    if step == 1:
        for i in np.linspace(0, X_DIFF, cycle_count):
            phase = i * math.pi/X_DIFF  # 0에서 pi까지의 구간을 sin으로 나눔
            x_move = i  # sin 곡선을 따라 다리 높이 설정
            y_move = B1 * math.sin(phase)  # sin 곡선을 따라 전방 이동

            theta1, theta2, theta3 = cal2(z_dif=x_move, y_dif= y_move, prev_x= DEFAULT_X, prev_y=DEFAULT_Y, l_1=l_1, l_2=l_2)
            
            # Front Left (FL) and Back Right (BR)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[2]], p.POSITION_CONTROL, theta2)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[0]], p.POSITION_CONTROL, -theta3)

            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[2]], p.POSITION_CONTROL, theta2)
            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[0]], p.POSITION_CONTROL, -theta3)

            for i in range(10):
                p.stepSimulation()
            time.sleep(step_time)
    elif step % 2 == 1:
        for i in np.linspace(0, X_DIFF, cycle_count):
            phase = i * math.pi/X_DIFF  # 0에서 pi까지의 구간을 sin으로 나눔
            x_move = i  # sin 곡선을 따라 다리 높이 설정
            y_move = B1 * math.sin(phase)  # sin 곡선을 따라 전방 이동

            theta1, theta2, theta3 = cal2(z_dif=x_move, y_dif= y_move, prev_x= DEFAULT_X, prev_y=DEFAULT_Y, l_1=l_1, l_2=l_2)
            
            # Front Left (FL) and Back Right (BR)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[2]], p.POSITION_CONTROL, theta2)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[0]], p.POSITION_CONTROL, -theta3)

            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[2]], p.POSITION_CONTROL, theta2)
            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[0]], p.POSITION_CONTROL, -theta3)

            x_move = X_DIFF - i  # sin 곡선을 따라 다리 높이 설정
            y_move = B2 * math.sin(phase)  # sin 곡선을 따라 전방 이동
            theta1, theta2, theta3 = cal2(z_dif=x_move, y_dif= -y_move, prev_x= DEFAULT_X, prev_y=DEFAULT_Y, l_1=l_1, l_2=l_2)
            p.setJointMotorControl2(robot_id, joints['front_right_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['front_right_' + joint_names[2]], p.POSITION_CONTROL, theta2)
            p.setJointMotorControl2(robot_id, joints['front_right_' + joint_names[0]], p.POSITION_CONTROL, -theta3)

            p.setJointMotorControl2(robot_id, joints['back_left_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['back_left_' + joint_names[2]], p.POSITION_CONTROL, theta2)
            p.setJointMotorControl2(robot_id, joints['back_left_' + joint_names[0]], p.POSITION_CONTROL, -theta3)
            for i in range(10):
                p.stepSimulation()
            time.sleep(step_time)
    elif step % 2 == 0:
    # Step 2: 반대편 다리 교체
        for i in np.linspace(0, X_DIFF, cycle_count):
            phase = i *math.pi/X_DIFF  # 0에서 pi까지의 구간을 sin으로 나눔
            x_move = i  # sin 곡선을 따라 다리 높이 설정
            y_move = B1 * math.sin(phase)  # sin 곡선을 따라 전방 이동

            theta1, theta2, theta3 = cal2(z_dif=x_move, y_dif= y_move, prev_x= DEFAULT_X, prev_y=DEFAULT_Y, l_1=l_1, l_2=l_2)
            p.setJointMotorControl2(robot_id, joints['front_right_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['front_right_' + joint_names[2]], p.POSITION_CONTROL, theta2)
            p.setJointMotorControl2(robot_id, joints['front_right_' + joint_names[0]], p.POSITION_CONTROL, -theta3)

            p.setJointMotorControl2(robot_id, joints['back_left_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['back_left_' + joint_names[2]], p.POSITION_CONTROL, theta2)
            p.setJointMotorControl2(robot_id, joints['back_left_' + joint_names[0]], p.POSITION_CONTROL, -theta3)

            x_move = X_DIFF - i
            y_move = B2 * math.sin(phase)
            theta1, theta2, theta3 = cal2(z_dif=x_move, y_dif= -y_move, prev_x= DEFAULT_X, prev_y=DEFAULT_Y, l_1=l_1, l_2=l_2)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[2]], p.POSITION_CONTROL, theta2)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[0]], p.POSITION_CONTROL, -theta3)

            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[2]], p.POSITION_CONTROL, theta2)
            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[0]], p.POSITION_CONTROL, -theta3)

            for i in range(10):
                p.stepSimulation()
            time.sleep(step_time)
def trot_right(step):
    cycle_count = 20  # 한 번의 스텝을 나누는 횟수

    if step == 1:
        for i in np.linspace(0, X_DIFF, cycle_count):
            phase = i * math.pi/X_DIFF  # 0에서 pi까지의 구간을 sin으로 나눔
            x_move = i  # sin 곡선을 따라 다리 높이 설정
            y_move = B1 * math.sin(phase)  # sin 곡선을 따라 전방 이동

            theta1, theta2, theta3 = cal2(z_dif=-x_move, y_dif= y_move, prev_x= DEFAULT_X, prev_y=DEFAULT_Y, l_1=l_1, l_2=l_2)
            
            # Front Left (FL) and Back Right (BR)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[2]], p.POSITION_CONTROL, theta2)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[0]], p.POSITION_CONTROL, -theta3)

            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[2]], p.POSITION_CONTROL, theta2)
            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[0]], p.POSITION_CONTROL, -theta3)

            for i in range(10):
                p.stepSimulation()
            time.sleep(step_time)
    elif step % 2 == 1:
        for i in np.linspace(0, X_DIFF, cycle_count):
            phase = i * math.pi/X_DIFF  # 0에서 pi까지의 구간을 sin으로 나눔
            x_move = i  # sin 곡선을 따라 다리 높이 설정
            y_move = B1 * math.sin(phase)  # sin 곡선을 따라 전방 이동

            theta1, theta2, theta3 = cal2(z_dif=-x_move, y_dif= y_move, prev_x= DEFAULT_X, prev_y=DEFAULT_Y, l_1=l_1, l_2=l_2)
            
            # Front Left (FL) and Back Right (BR)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[2]], p.POSITION_CONTROL, theta2)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[0]], p.POSITION_CONTROL, -theta3)

            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[2]], p.POSITION_CONTROL, theta2)
            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[0]], p.POSITION_CONTROL, -theta3)

            x_move = X_DIFF - i  # sin 곡선을 따라 다리 높이 설정
            y_move = B2 * math.sin(phase)  # sin 곡선을 따라 전방 이동
            theta1, theta2, theta3 = cal2(z_dif=-x_move, y_dif= -y_move, prev_x= DEFAULT_X, prev_y=DEFAULT_Y, l_1=l_1, l_2=l_2)
            p.setJointMotorControl2(robot_id, joints['front_right_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['front_right_' + joint_names[2]], p.POSITION_CONTROL, theta2)
            p.setJointMotorControl2(robot_id, joints['front_right_' + joint_names[0]], p.POSITION_CONTROL, -theta3)

            p.setJointMotorControl2(robot_id, joints['back_left_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['back_left_' + joint_names[2]], p.POSITION_CONTROL, theta2)
            p.setJointMotorControl2(robot_id, joints['back_left_' + joint_names[0]], p.POSITION_CONTROL, -theta3)
            for i in range(10):
                p.stepSimulation()
            time.sleep(step_time)
    elif step % 2 == 0:
    # Step 2: 반대편 다리 교체
        for i in np.linspace(0, X_DIFF, cycle_count):
            phase = i *math.pi/X_DIFF  # 0에서 pi까지의 구간을 sin으로 나눔
            x_move = i  # sin 곡선을 따라 다리 높이 설정
            y_move = B1 * math.sin(phase)  # sin 곡선을 따라 전방 이동

            theta1, theta2, theta3 = cal2(z_dif=-x_move, y_dif= y_move, prev_x= DEFAULT_X, prev_y=DEFAULT_Y, l_1=l_1, l_2=l_2)
            p.setJointMotorControl2(robot_id, joints['front_right_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['front_right_' + joint_names[2]], p.POSITION_CONTROL, theta2)
            p.setJointMotorControl2(robot_id, joints['front_right_' + joint_names[0]], p.POSITION_CONTROL, -theta3)

            p.setJointMotorControl2(robot_id, joints['back_left_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['back_left_' + joint_names[2]], p.POSITION_CONTROL, theta2)
            p.setJointMotorControl2(robot_id, joints['back_left_' + joint_names[0]], p.POSITION_CONTROL, -theta3)

            x_move = X_DIFF - i
            y_move = B2 * math.sin(phase)
            theta1, theta2, theta3 = cal2(z_dif=-x_move, y_dif= -y_move, prev_x= DEFAULT_X, prev_y=DEFAULT_Y, l_1=l_1, l_2=l_2)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[2]], p.POSITION_CONTROL, theta2)
            p.setJointMotorControl2(robot_id, joints['front_left_' + joint_names[0]], p.POSITION_CONTROL, -theta3)

            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[1]], p.POSITION_CONTROL, -theta1)
            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[2]], p.POSITION_CONTROL, theta2)
            p.setJointMotorControl2(robot_id, joints['back_right_' + joint_names[0]], p.POSITION_CONTROL, -theta3)

            for i in range(10):
                p.stepSimulation()
            time.sleep(step_time)

def sin_step_left():
    z_diff = 0.15
    b1, b2 = 0.05, 0.02
    for i in np.linspace(0, z_diff, 10):        
        theta1, theta2, theta3 = cal2(z_dif=i,y_dif=b1 * math.sin(i/z_diff* math.pi),prev_x=DEFAULT_X, prev_y=DEFAULT_Y, l_1=l_1, l_2=l_2)
        target_position_hip=-theta1
        target_position_knee=theta2
        #print(f'theta1: {math.degrees(theta1)}, theta2: {math.degrees(theta2)}, theta3: {math.degrees(theta3)}')
        for j in range(4):
            p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joints[leg_prefixes[j]+'_'+joint_names[1]],
                            controlMode=p.POSITION_CONTROL, targetPosition=target_position_hip)
            p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joints[leg_prefixes[j]+'_'+joint_names[2]],
                            controlMode=p.POSITION_CONTROL, targetPosition=target_position_knee)
            p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joints[leg_prefixes[j]+'_'+joint_names[0]],
                            controlMode=p.POSITION_CONTROL, targetPosition=-theta3)
        
        for i in range(10):
            p.stepSimulation()
        time.sleep(sleep)
    for i in np.linspace(z_diff, 0, 10):        
        theta1, theta2, theta3 = cal2(z_dif=i,y_dif=-b2 * math.sin(i/z_diff* math.pi),prev_x=DEFAULT_X, prev_y=DEFAULT_Y, l_1=l_1, l_2=l_2)
        target_position_hip=-theta1
        target_position_knee=theta2
        #print(f'theta1: {math.degrees(theta1)}, theta2: {math.degrees(theta2)}, theta3: {math.degrees(theta3)}')
        for j in range(4):
            p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joints[leg_prefixes[j]+'_'+joint_names[1]],
                            controlMode=p.POSITION_CONTROL, targetPosition=target_position_hip)
            p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joints[leg_prefixes[j]+'_'+joint_names[2]],
                            controlMode=p.POSITION_CONTROL, targetPosition=target_position_knee)
            p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joints[leg_prefixes[j]+'_'+joint_names[0]],
                            controlMode=p.POSITION_CONTROL, targetPosition=-theta3)
        
        for i in range(10):
            p.stepSimulation()
        time.sleep(sleep)

def sin_step_right():
    z_diff = 0.15
    b1, b2 = 0.05, 0.02
    for i in np.linspace(0, z_diff, 10):        
        theta1, theta2, theta3 = cal2(z_dif=-i,y_dif=b1 * math.sin(i/z_diff* math.pi),prev_x=DEFAULT_X, prev_y=DEFAULT_Y, l_1=l_1, l_2=l_2)
        target_position_hip=-theta1
        target_position_knee=theta2
        #print(f'theta1: {math.degrees(theta1)}, theta2: {math.degrees(theta2)}, theta3: {math.degrees(theta3)}')
        for j in range(4):
             
            p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joints[leg_prefixes[j]+'_'+joint_names[1]],
                            controlMode=p.POSITION_CONTROL, targetPosition=target_position_hip)
            p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joints[leg_prefixes[j]+'_'+joint_names[2]],
                            controlMode=p.POSITION_CONTROL, targetPosition=target_position_knee)
            p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joints[leg_prefixes[j]+'_'+joint_names[0]],
                            controlMode=p.POSITION_CONTROL, targetPosition=-theta3)
        
        for i in range(10):
            p.stepSimulation()
        time.sleep(sleep)
    for i in np.linspace(z_diff, 0, 10):        
        theta1, theta2, theta3 = cal2(z_dif=-i,y_dif=-b2 * math.sin(i/z_diff* math.pi),prev_x=DEFAULT_X, prev_y=DEFAULT_Y, l_1=l_1, l_2=l_2)
        target_position_hip=-theta1
        target_position_knee=theta2
        #print(f'theta1: {math.degrees(theta1)}, theta2: {math.degrees(theta2)}, theta3: {math.degrees(theta3)}')
        for j in range(4):
             
            p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joints[leg_prefixes[j]+'_'+joint_names[1]],
                            controlMode=p.POSITION_CONTROL, targetPosition=target_position_hip)
            p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joints[leg_prefixes[j]+'_'+joint_names[2]],
                            controlMode=p.POSITION_CONTROL, targetPosition=target_position_knee)
            p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joints[leg_prefixes[j]+'_'+joint_names[0]],
                            controlMode=p.POSITION_CONTROL, targetPosition=-theta3)
        
        for i in range(10):
            p.stepSimulation()
        time.sleep(sleep)

sleep = 1/ 50
def sin_step_forward():
    b1, b2, a = 0.05, 0.02, 0.1
    for i in np.linspace(0, a, 10):
        
        theta_1, theta_2 = cal(l_1, l_2, DEFAULT_X-i, DEFAULT_Y - b1 * math.sin(i * math.pi / a))
        target_position_hip = -theta_1  # 힙 관절 각도를 전진 방향으로 설정
        target_position_knee = theta_2  # 무릎 관절 각도를 전진 방향으로 설정
        for j in range(4):
                p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joints[leg_prefixes[j]+'_'+joint_names[1]],
                                controlMode=p.POSITION_CONTROL, targetPosition=target_position_hip)
                p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joints[leg_prefixes[j]+'_'+joint_names[2]],
                                controlMode=p.POSITION_CONTROL, targetPosition=target_position_knee)
        for i in range(10):
            p.stepSimulation()
        time.sleep(sleep)
    for i in np.linspace(a, 0, 10):
        theta_1, theta_2 = cal(l_1, l_2, DEFAULT_X-i, DEFAULT_Y + b2 * math.sin(i * math.pi / a))
        target_position_hip = -theta_1  # 힙 관절 각도를 전진 방향으로 설정
        target_position_knee = theta_2  # 무릎 관절 각도를 전진 방향으로 설정
        for j in range(4):
                p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joints[leg_prefixes[j]+'_'+joint_names[1]],
                                controlMode=p.POSITION_CONTROL, targetPosition=target_position_hip)
                p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joints[leg_prefixes[j]+'_'+joint_names[2]],
                                controlMode=p.POSITION_CONTROL, targetPosition=target_position_knee)
        for i in range(10):
            p.stepSimulation()
        time.sleep(sleep)
def sin_step_backward():
    b1, b2, a = 0.05, 0.02, 0.1
    for i in np.linspace(0, a, 10):
        
        theta_1, theta_2 = cal(l_1, l_2, DEFAULT_X+i, DEFAULT_Y - b1 * math.sin(i * math.pi / a))
        target_position_hip = -theta_1  # 힙 관절 각도를 전진 방향으로 설정
        target_position_knee = theta_2  # 무릎 관절 각도를 전진 방향으로 설정
        for j in range(4):
            p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joints[leg_prefixes[j]+'_'+joint_names[1]],
                            controlMode=p.POSITION_CONTROL, targetPosition=target_position_hip)
            p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joints[leg_prefixes[j]+'_'+joint_names[2]],
                            controlMode=p.POSITION_CONTROL, targetPosition=target_position_knee)
        for i in range(10):
            p.stepSimulation()
        time.sleep(sleep)
    for i in np.linspace(a,0, 10):
        theta_1, theta_2 = cal(l_1, l_2, DEFAULT_X+i, DEFAULT_Y + b2 * math.sin(i * math.pi / a))
        target_position_hip = -theta_1  # 힙 관절 각도를 전진 방향으로 설정
        target_position_knee = theta_2  # 무릎 관절 각도를 전진 방향으로 설정
        for j in range(4):
            p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joints[leg_prefixes[j]+'_'+joint_names[1]],
                            controlMode=p.POSITION_CONTROL, targetPosition=target_position_hip)
            p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joints[leg_prefixes[j]+'_'+joint_names[2]],
                            controlMode=p.POSITION_CONTROL, targetPosition=target_position_knee)
        for i in range(10):
            p.stepSimulation()
        time.sleep(sleep)

target = 0
forward_step = 0
backward_step = 0
left_step = 0
right_step=0
# 시뮬레이션 루프
while True:
    keys = p.getKeyboardEvents()  # 키보드 입력 감지

    # 위 화살표: 앞으로 이동
    if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN:
        #box_step_forward()
        #sin_step_forward()
        forward_step+=1
        trot_forward(forward_step)

    # 아래 화살표: 뒤로 이동
    elif p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:
        #box_step_backward()
        #sin_step_backward()
        backward_step += 1
        trot_backward(backward_step)
        

    elif p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:
        #sin_step_left()
        left_step += 1
        trot_left(left_step)

    elif p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
        #sin_step_right()
        right_step += 1
        trot_right(right_step)
    # 'T' 키로 trot 함수 실행
    else:
        # print('theta1:', math.degrees(theta_1))
        # print("theta2:",math.degrees(theta_2))
        forward_step=0
        backward_step = 0
        left_step = 0
        right_step = 0
        target_position_hip = default_position_hip  # 힙 관절 각도 초기화
        target_position_knee = default_position_knee  # 무릎 관절 각도 초기화
        for j in range(4):
                p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joints[leg_prefixes[j]+'_'+joint_names[1]],
                                controlMode=p.POSITION_CONTROL, targetPosition=target_position_hip)
                p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joints[leg_prefixes[j]+'_'+joint_names[2]],
                                controlMode=p.POSITION_CONTROL, targetPosition=target_position_knee)
                p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joints[leg_prefixes[j]+'_'+joint_names[0]],
                            controlMode=p.POSITION_CONTROL, targetPosition=0)
                
        

    # 시뮬레이션 진행
    p.stepSimulation()
    time.sleep(1.0 / 240.0)  # 시뮬레이션 속도 조절

# 시뮬레이션 종료
p.disconnect()
