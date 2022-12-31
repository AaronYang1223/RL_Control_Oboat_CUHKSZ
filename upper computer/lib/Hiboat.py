""" 用于处理四元数及空间变换 """
import math

""" gym """
import gym
from gym import spaces
#from gym.utils import seeding TODO 目前仍不清楚seed的作用

"""numpy"""
import numpy as np
from numpy.core.defchararray import center #TODO

""" UDP """
import socket
import time

"""serial"""
import serial

"""check"""
from stable_baselines3.common.env_checker import check_env

"""NatNetClient"""
from lib.NatNetClient import NatNetClient

v = [0, 0, 0, 0, 0, 0, 0]

def data_stream():
    '''
    def receiveNewFrame(frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged):
        return
    '''

    # This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
    def receiveNewFrame( frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged ):
        pass
        # print( "Received frame", frameNumber )

    def receiveRigidBodyFrame(id, position, rotation):
        # position in meter
        # rotation in radius
        # print( "Received frame for rigid body", id )
        
        # TODO only receive the info of the rigid body with the right id
        if (id == 1001): 
            v[0] = position[0]
            v[1] = position[1]
            v[2] = position[2]
            
            v[3] = rotation[0]
            v[4] = rotation[1]
            v[5] = rotation[2]
            v[6] = rotation[3]

    # This will create a new NatNet client
    streamingClient = NatNetClient()

    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streamingClient.newFrameListener = receiveNewFrame
    streamingClient.rigidBodyListener = receiveRigidBodyFrame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    streamingClient.run()

class HiboatEnv(gym.Env):
    """
    Initialize:
        Range: A, B, C, D
        Start Point: _S
        Target Point: T
        Start Rotation: _R
        Episode length：L
        Max Error：M_Error
        Expected Error: E_Error

    Observation:
        Type: Box(6)
        Num     Observation               Min                     Max
        0       x                         A[0]                    C[0]
        1       y                         A[1]                    C[1]
        2       z                         -Inf                    Inf
        3       roll                      -math.pi rad(-180 deg)  math.pi rad(180 deg)
        4       pitch                     -math.pi rad(-180 deg)  math.pi rad(180 deg)
        5       yaw                       -math.pi rad(-180 deg)  math.pi rad(180 deg)
    Actions:
        Type: Box(3)

        ###range(-1, 1)
        Num     Action                    Min                     Max
        0       pwmA                      (-1)1400                    (1)1600
        1       pwmB                      (-1)1400                    (1)1600
        2       pwmC                      (-1)1400                    (1)1600
    Reward:
        船的位置到目标点的距离负数
    Starting State:
        Position:_S
        Rotation:_R
    Episode Termination:
        #1 船到起始点和目标点的中点的距离大于M_Error
        #2 Episode length is greater than L.
        #3 Solved Requirements: 船到目标点距离小于E_Error

    TIPS:
    #1 写成多线程 
    #2 发送命令的频率
    """

    #metadata = {"render.modes": ["human", "rgb_array"], "video.frames_per_second": 50} 与GUI有关，不考虑

    def __init__(self):
        data_stream()
        self.A = np.array([0, 0, 0])
        self.B = np.array([5, 0, 0])
        self.C = np.array([5, 5, 0])
        self.D = np.array([0, 5, 0])
        
        self._S = np.array([1, 1, 0])
        self._R = np.array([1, 1, 1])
        self.T = np.array([3, 3, 0])

        self.L = 200
        self.M_Error = 0.5
        self.E_Error = 0.1
        
        # Bounds.
        low_o = np.array(
            [
                self.A[0],
                self.A[1],
                -np.finfo(np.float32).max,
                -math.pi,
                -math.pi,
                -math.pi,
            ],
            dtype=np.float32,
        )

        high_o = np.array(
            [
                self.C[0],
                self.C[1],
                np.finfo(np.float32).max,
                math.pi,
                math.pi,
                math.pi,
            ],
            dtype=np.float32,
        )

        # low_a = np.array([1400, 1400, 1400])
        # high_a = np.array([1600, 1600, 1600])

        low_a = np.array([-1, -1, -1])
        high_a = np.array([1, 1, 1])

        self.low = 1400
        self.high = 1600

        self.observation_space = spaces.Box(low_o, high_o, dtype=np.float32)
        self.action_space = spaces.Box(low_a, high_a, dtype=np.float32)

        # self.seed() #TODO 还没整明白
        self.state = None

        # 初始化UDP通讯
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.PORT = 8000

        #TODO 这个需要不断更新
        self.v = [0, 0, 0, 0, 0, 0, 0]

    # TODO seed
    # def seed(self, seed=None):
    #     self.np_random, seed = seeding.np_random(seed)
    #     return [seed]

    """
    Private methods
    #1 quart_to_rpy
    #2 send_data
    #3 match
    """
    
    def quart_to_rpy(self, x, y, z, w):
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(2 * (w * y - x * z))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
        return roll, pitch, yaw
    
    def send_data(self, msg):
        start = time.time()  #获取当前时间
        print(time.strftime('%Y-%m-%d %H:%M:%S',time.localtime(start)))  #以指定格式显示当前时间
        print(msg)
        #TODO: 这里需要调整ip地址
        server_address = ("192.168.3.137", self.PORT)  # TODO 接收方 服务器的ip地址和端口号
        self.client_socket.sendto(msg.encode(), server_address) #将msg内容发送给指定接收方
        # time.sleep(1)

    def match(self, _action):
        order = (self.low + self.high) / 2 + ( (self.high - self.low) / 2 * _action)
        return order  

    def step(self, action):
        # err_msg = "%r (%s) invalid" % (action, type(action))
        # assert self.action_space.contains(action), err_msg ### 应该是与可视化有关，不确定

        #TODO
        """
        #1 发送命令：命令格式待定, 前置！！！
        #2 读取Hiboat位置
        #3 done判断
        """ 

        # 1 传输命令

        #order = "AAAA,BBBB,CCCC," 
        _order = [str(self.match(int(action[0]))), str(self.match(int(action[1]))), str(self.match(int(action[2])))]
        order = _order[0] + "," + _order[1] + "," + _order[2] + "," 

        # 传输命令
        self.send_data(order)

        #中间可能需要一些sleep去控制
        self.v[0] = v[0]
        self.v[1] = v[1]
        self.v[2] = v[2]
        self.v[3] = v[3]
        self.v[4] = v[4]
        self.v[5] = v[5]
        self.v[6] = v[6]

        # #1      
        # #order = "AAAABBBBCCCC" 
        # _order = [str(self.match(int(action[0]))), str(self.match(int(action[0]))), str(self.match(int(action[0])))]
        # order = _order[0] + _order[1] + _order[2] 

        # # 传输命令
        # self.send_data(order)
        
        #2 
        roll, pitch, yaw = self.quart_to_rpy(self.v[3], self.v[4], self.v[5], self.v[6]) #TODO v的数据来自datastream进程
        self.state = [self.v[0], self.v[1], self.v[2], roll, pitch, yaw]
        print(self.state)

        #3
        distance_p = math.sqrt(pow((self.v[0] - self.T[0]), 2) + pow((self.v[1] - self.T[1]), 2) + pow((self.v[2] - self.T[2]), 2))
        center = np.array([(self.T[0] + self._S[0])/2, (self.T[1] + self._S[1])/2, (self.T[2] + self._S[2])/2])
        distance_c = math.sqrt(pow((self.v[0] - center[0]), 2) + pow((self.v[1] - center[1]), 2) + pow((self.v[2] - center[2]), 2))

        # done = bool(
        #     distance_p < self.E_Error
        #     or distance_c > self.M_Error
        # )

        done = False

        # if not done:
        #     reward = -distance_p
        if distance_c > self.M_Error:
            reward = -2 * distance_p
        else:
            reward = -distance_p
        
        if(done):
            print("DONE")

        return np.array(self.state, dtype=np.float32), reward, done, {}

    #TODO 只是reset了state，并且发送了reset命令，但是没有check是否确实reset成功
    def reset(self):
        print("Start RESET")
        try:

            #TODO: 
            ser = serial.Serial('COM14', 9600)

            if(ser.isOpen()):
                print("OPEN")

            # f
            for i in range(4):
                ser.write(b"f")
                time.sleep(0.5)
            _result = ser.read_all()
            print(_result)

            time.sleep(5)

            # r
            # for i in range(4):
            ser.write(b"r")
                # time.sleep(0.5)
            _result = ser.read_all()
            print(_result)           

            ser.close()#关闭串口
            print("CLOSE")

        except Exception as e:
            print("---异常---：",e)

        print("RESET DOWN")
        self.state = np.array([self._S[0], self._S[1], self._S[2], self._R[0], self._R[0], self._R[0]])
        return np.array(self.state, dtype=np.float32)

    def render(self, mode="human"):
        pass

    def close(self):
        pass

env = HiboatEnv()
# # It will check your custom environment and output additional warnings if needed
# check_env(env)

# env.send_data("1400,1400,1400,")
while True:
    action = env.action_space.sample()
    n_state, reward, done, info = env.step(action)
    # print(env.v)
    time.sleep(1)



