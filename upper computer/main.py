from lib.NatNetClient import NatNetClient
# from lib.Hiboat import *
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
import time

import gym
import multiprocessing as mp

def data_stream(v):
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

def main(v):
    # print("STEP0")

    # # Parallel environments, TODO 这里的""中内容我不是很确定
    # env = make_vec_env("Hiboat-v0")

    # print("STEP1")
    # model = PPO("MlpPolicy", env, verbose=1)
    # model.learn(total_timesteps=25000)
    # model.save("ppo_cartpole")

    # del model # remove to demonstrate saving and loading

    # print("STEP2")
    # model = PPO.load("ppo_cartpole")

    # obs = env.reset()

    # # print("STEP0")
    # while True:
    #     time.sleep(1)
    #     # print("STEP1")
    #     action, _states = model.predict(obs)
    #     # 1 传输命令

    #     #order = "AAAA,BBBB,CCCC," 
    #     _order = [str(env.match(int(action[0]))), str(env.match(int(action[1]))), str(env.match(int(action[2])))]
    #     order = _order[0] + "," + _order[1] + "," + _order[2] + "," 

    #     # 传输命令
    #     env.send_data(order)

    #     #中间可能需要一些sleep去控制
    #     env.v[0] = v[0]
    #     env.v[1] = v[1]
    #     env.v[2] = v[2]
    #     env.v[3] = v[3]
    #     env.v[4] = v[4]
    #     env.v[5] = v[5]
    #     env.v[6] = v[6]

    # #     # print(env.v[0])
    # #     # print(env.v[1])
    # #     # print(env.v[2])
    # #     # print(env.v[3])
    # #     # print(env.v[4])
    # #     # print(env.v[5])
    # #     # print(env.v[6])
        
    #     # obs, rewards, dones, info = env.step(action)
    #     env.render()

    #================================================

    env_name = "Hiboat-v0"
    env = gym.make(env_name)          # 导入注册器中的环境

    episodes = 10
    for episode in range(1, episodes + 1):
        state = env.reset()           # gym风格的env开头都需要reset一下以获取起点的状态
        done = False
        score = 0

        while not done:
            # env.render()              # 将当前的状态化成一个frame，再将该frame渲染到小窗口上
            time.sleep(2)
            action = env.action_space.sample()     # 通过随机采样获取一个随即动作
            print(action)

            #order = "AAAA,BBBB,CCCC," 
            _order = [str(int(env.match(action[0]))), str(int(env.match(action[1]))), str(int(env.match(action[2])))]
            order = _order[0] + "," + _order[1] + "," + _order[2] + "," 

            # 传输命令
            env.send_data(order)

            #中间可能需要一些sleep去控制
            env.v[0] = v[0]
            env.v[1] = v[1]
            env.v[2] = v[2]
            env.v[3] = v[3]
            env.v[4] = v[4]
            env.v[5] = v[5]
            env.v[6] = v[6]

            n_state, reward, done, info = env.step(action)    # 将动作扔进环境中，从而实现和模拟器的交互
            score += reward
        print("Episode : {}, Score : {}".format(episode, score))

    env.close()     # 关闭窗口

    #=====================
    # while True:
    # # for i in range(2):
    #     print(v[0])
    #     print(v[1])
    #     print(v[2])
    #     print(v[3])
    #     print(v[4])
    #     print(v[5])
    #     print(v[6])
    #     print("=============")
    #     time.sleep(1)

def multiprocess():
    v = mp.Array('d',[0,0,0,0,0,0,0]) # shared array [position and rotation] ###
    p1 = mp.Process(target=data_stream, args=(v,)) # process for reading ###这两个part应该是保持一致的
    p2 = mp.Process(target=main, args=(v,)) # process for calculating and sending ###
    p1.start()
    p2.start()
    p1.join()
    p2.join()

if __name__ == '__main__':
    multiprocess()
