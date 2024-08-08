import numpy as np
import random
from datetime import datetime
import gymnasium as gym
from gymnasium import spaces
import messages_pb2 as pb
import ns3ai_gym_msg_py as py_binding
from ns3ai_utils import Experiment

import os

from ray.rllib.env.multi_agent_env import MultiAgentEnv, make_multi_agent

class Ns3Env(gym.Env):
    _created = False

    def _create_space(self, spaceDesc):
        # print("Debugging dummyEnv _create_space()")
        space = None
        if spaceDesc.type == pb.Discrete:
            discreteSpacePb = pb.DiscreteSpace()
            spaceDesc.space.Unpack(discreteSpacePb)
            space = spaces.Discrete(discreteSpacePb.n)

        elif spaceDesc.type == pb.Box:
            boxSpacePb = pb.BoxSpace()
            spaceDesc.space.Unpack(boxSpacePb)
            low = boxSpacePb.low
            high = boxSpacePb.high
            shape = tuple(boxSpacePb.shape)
            mtype = boxSpacePb.dtype

            if mtype == pb.INT:
                mtype = np.int64
            elif mtype == pb.UINT:
                mtype = np.uint
            elif mtype == pb.DOUBLE:
                mtype = np.float64
            else:
                mtype = np.float64 #np.float

            space = spaces.Box(low=low, high=high, shape=shape, dtype=mtype)

        elif spaceDesc.type == pb.Tuple:
            mySpaceList = []
            tupleSpacePb = pb.TupleSpace()
            spaceDesc.space.Unpack(tupleSpacePb)

            for pbSubSpaceDesc in tupleSpacePb.element:
                subSpace = self._create_space(pbSubSpaceDesc)
                mySpaceList.append(subSpace)

            mySpaceTuple = tuple(mySpaceList)
            space = spaces.Tuple(mySpaceTuple)

        elif spaceDesc.type == pb.Dict:
            mySpaceDict = {}
            dictSpacePb = pb.DictSpace()
            spaceDesc.space.Unpack(dictSpacePb)

            for pbSubSpaceDesc in dictSpacePb.element:
                subSpace = self._create_space(pbSubSpaceDesc)
                mySpaceDict[pbSubSpaceDesc.name] = subSpace

            space = spaces.Dict(mySpaceDict)

        return space

    def _create_data(self, dataContainerPb):
        # print("Debugging dummyEnv _create_data()")
        if dataContainerPb.type == pb.Discrete:
            discreteContainerPb = pb.DiscreteDataContainer()
            dataContainerPb.data.Unpack(discreteContainerPb)
            data = discreteContainerPb.data
            return data
        
        if dataContainerPb.type == pb.Box:
            boxContainerPb = pb.BoxDataContainer()
            dataContainerPb.data.Unpack(boxContainerPb)
            # print(boxContainerPb.shape, boxContainerPb.dtype, boxContainerPb.uintData)

            if boxContainerPb.dtype == pb.INT:
                data = boxContainerPb.intData
            elif boxContainerPb.dtype == pb.UINT:
                data = boxContainerPb.uintData
            elif boxContainerPb.dtype == pb.DOUBLE:
                data = boxContainerPb.doubleData
            else:
                data = boxContainerPb.floatData

            # TODO: reshape using shape info
            # print("Debugging dummyEnv _create_data() with data : ", data)
            data = np.array(data)
            return data

        elif dataContainerPb.type == pb.Tuple:
            tupleDataPb = pb.TupleDataContainer()
            dataContainerPb.data.Unpack(tupleDataPb)

            myDataList = []
            for pbSubData in tupleDataPb.element:
                subData = self._create_data(pbSubData)
                myDataList.append(subData)

            data = tuple(myDataList)
            return data

        elif dataContainerPb.type == pb.Dict:
            dictDataPb = pb.DictDataContainer()
            dataContainerPb.data.Unpack(dictDataPb)

            myDataDict = {}
            for pbSubData in dictDataPb.element:
                subData = self._create_data(pbSubData)
                myDataDict[pbSubData.name] = subData

            data = myDataDict
            return data

    def initialize_env(self):
        # print("Debugging dummyEnv initialize_env()")
        simInitMsg = pb.SimInitMsg()
        self.msgInterface.PyRecvBegin()
        request = self.msgInterface.GetCpp2PyStruct().get_buffer()
        simInitMsg.ParseFromString(request)
        self.msgInterface.PyRecvEnd()

        self.action_space = self._create_space(simInitMsg.actSpace)
        self.observation_space = self._create_space(simInitMsg.obsSpace)

        reply = pb.SimInitAck()
        reply.done = True
        reply.stopSimReq = False
        reply_str = reply.SerializeToString()
        assert len(reply_str) <= py_binding.msg_buffer_size

        self.msgInterface.PySendBegin()
        self.msgInterface.GetPy2CppStruct().size = len(reply_str)
        self.msgInterface.GetPy2CppStruct().get_buffer_full()[:len(reply_str)] = reply_str
        self.msgInterface.PySendEnd()
        return True

    def send_close_command(self):
        # print("Debugging dummyEnv send_close_command()")
        reply = pb.EnvActMsg()
        reply.stopSimReq = True

        replyMsg = reply.SerializeToString()
        assert len(replyMsg) <= py_binding.msg_buffer_size
        self.msgInterface.PySendBegin()
        self.msgInterface.GetPy2CppStruct().size = len(replyMsg)
        self.msgInterface.GetPy2CppStruct().get_buffer_full()[:len(replyMsg)] = replyMsg
        self.msgInterface.PySendEnd()

        self.newStateRx = False
        return True

    def rx_env_state(self):
        # print("Debugging dummyEnv rx_env_state()")
        if self.newStateRx:
            return

        envStateMsg = pb.EnvStateMsg()
        self.msgInterface.PyRecvBegin()
        request = self.msgInterface.GetCpp2PyStruct().get_buffer()
        envStateMsg.ParseFromString(request)
        self.msgInterface.PyRecvEnd()

        self.obsData = self._create_data(envStateMsg.obsData)
        self.reward = envStateMsg.reward
        self.gameOver = envStateMsg.isGameOver
        self.gameOverReason = envStateMsg.reason

        if self.gameOver:
            self.send_close_command()

        self.extraInfo = envStateMsg.info
        if not self.extraInfo:
            self.extraInfo = {}

        self.newStateRx = True

    def get_obs(self):

        # print("Debugging dummyEnv get_obs()", self.obsData)

        return self.obsData

    def get_reward(self):
        # print("Debugging dummyEnv get_reward()")
        return self.reward

    def is_game_over(self):
        # print("Debugging dummyEnv is_game_over()")
        return self.gameOver

    def get_extra_info(self):
        # print("Debugging dummyEnv get_extra_info()")
        return self.extraInfo

    def _pack_data(self, actions, spaceDesc):
        # print("Debugging dummyEnv _pack_data()")
        dataContainer = pb.DataContainer()

        spaceType = spaceDesc.__class__

        if spaceType == spaces.Discrete:
            dataContainer.type = pb.Discrete
            discreteContainerPb = pb.DiscreteDataContainer()
            discreteContainerPb.data = actions
            dataContainer.data.Pack(discreteContainerPb)

        elif spaceType == spaces.Box:
            dataContainer.type = pb.Box
            boxContainerPb = pb.BoxDataContainer()
            shape = [len(actions)]
            boxContainerPb.shape.extend(shape)

            if spaceDesc.dtype in ['int', 'int8', 'int16', 'int32', 'int64']:
                boxContainerPb.dtype = pb.INT
                boxContainerPb.intData.extend(actions)

            elif spaceDesc.dtype in ['uint', 'uint8', 'uint16', 'uint32', 'uint64']:
                boxContainerPb.dtype = pb.UINT
                boxContainerPb.uintData.extend(actions)

            elif spaceDesc.dtype in ['float', 'float32', 'float64']:
                boxContainerPb.dtype = pb.FLOAT
                boxContainerPb.floatData.extend(actions)

            elif spaceDesc.dtype in ['double']:
                boxContainerPb.dtype = pb.DOUBLE
                boxContainerPb.doubleData.extend(actions)

            else:
                boxContainerPb.dtype = pb.FLOAT
                boxContainerPb.floatData.extend(actions)

            dataContainer.data.Pack(boxContainerPb)

        elif spaceType == spaces.Tuple:
            dataContainer.type = pb.Tuple
            tupleDataPb = pb.TupleDataContainer()

            spaceList = list(self.action_space.spaces)
            subDataList = []
            for subAction, subActSpaceType in zip(actions, spaceList):
                subData = self._pack_data(subAction, subActSpaceType)
                subDataList.append(subData)

            tupleDataPb.element.extend(subDataList)
            dataContainer.data.Pack(tupleDataPb)

        elif spaceType == spaces.Dict:
            dataContainer.type = pb.Dict
            dictDataPb = pb.DictDataContainer()

            subDataList = []
            for sName, subAction in actions.items():
                subActSpaceType = self.action_space.spaces[sName]
                subData = self._pack_data(subAction, subActSpaceType)
                subData.name = sName
                subDataList.append(subData)

            dictDataPb.element.extend(subDataList)
            dataContainer.data.Pack(dictDataPb)

        return dataContainer

    def send_actions(self, actions):
        # print("Debugging dummyEnv send_actions()")
        reply = pb.EnvActMsg()

        actionMsg = self._pack_data(actions, self.action_space)
        reply.actData.CopyFrom(actionMsg)

        replyMsg = reply.SerializeToString()
        assert len(replyMsg) <= py_binding.msg_buffer_size
        self.msgInterface.PySendBegin()
        self.msgInterface.GetPy2CppStruct().size = len(replyMsg)
        self.msgInterface.GetPy2CppStruct().get_buffer_full()[:len(replyMsg)] = replyMsg
        self.msgInterface.PySendEnd()
        self.newStateRx = False
        return True

    def get_state(self):
        # print("Debugging dummyEnv get_state()")
        obs = self.get_obs()
        reward = self.get_reward()
        done = self.is_game_over()
        extraInfo = {"info": self.get_extra_info()}
        return obs, reward, done, False, extraInfo

    def __init__(self, targetName, ns3Path, ns3Settings=None, shmSize=4096, envNumber=''):
        # print("Debugging dummyEnv __init__()")
        # print("self._created : ", self._created)
        if self._created:
            raise Exception('Error: Ns3Env is singleton')
        self._created = True
        self.exp = Experiment(targetName, ns3Path, py_binding, shmSize=shmSize, envNumber=envNumber)
        self.ns3Settings = ns3Settings

        self.newStateRx = False
        self.obsData = None
        self.reward = 0
        self.gameOver = False
        self.gameOverReason = None
        self.extraInfo = None
        
        self.msgInterface = self.exp.run(setting=self.ns3Settings, show_output=True)
        self.initialize_env()
        # get first observations
        self.rx_env_state()
        self.envDirty = False

    def step(self, actions):
        print("Debugging dummyEnv step()")
        self.send_actions(actions)
        self.rx_env_state()
        self.envDirty = True
        return self.get_state()

    def reset(self, seed=None, options=None):
        print("Debugging dummyEnv reset()")
        if not self.envDirty:
            obs = self.get_obs()
            return obs, {}

        # not using self.exp.kill() here in order for semaphores to reset to initial state
        if not self.gameOver:
            self.rx_env_state()
            self.send_close_command()

        self.msgInterface = None
        self.newStateRx = False
        self.obsData = None
        self.reward = 0
        self.gameOver = False
        self.gameOverReason = None
        self.extraInfo = None

        self.msgInterface = self.exp.run(setting=self.ns3Settings, show_output=True)
        self.initialize_env()
        # get first observations
        self.rx_env_state()
        self.envDirty = False

        obs = self.get_obs()
        return obs, {}

    def render(self, mode='human'):
        return

    def get_random_action(self):
        # print("Debugging dummyEnv get_random_action()")
        act = self.action_space.sample()
        return act

    def close(self):
        print("Debugging dummyEnv close()")
        # environment is not needed anymore, so kill subprocess in a straightforward way
        self.exp.kill()
        # destroy the message interface and its shared memory segment
        del self.exp


class CentralDecisionWifiEnv (gym.Env):
    """Custom Environment that follows gym interface."""

    metadata = {"render_modes": ["human"], "render_fps": 30}

    def __init__(self, ns3Settings, sim_config) :
        print("Debugging CentralEnv __init__()")

        # self.seed = sim_config['seed'] 
        # np.random.seed(self.seed)
        # random.seed(self.seed)

        self.reward = 0
        self.done = False
        self.max_num_aps = 4
        self.num_ap = sim_config['num_ap']
        print('Number of APs : ',self.num_ap)
        self.reward_type = sim_config['reward_type']
        self.isEvaluation = sim_config['isEvaluation']
        self.new_udpLambda = sim_config['new_udpLambda']
        self.output_dir = sim_config['output_dir']
        self.alpha = sim_config['alpha']
        self.beta = sim_config['beta']
        self.min_ap_interval = sim_config['min_num_aps']
        self.max_ap_interval = sim_config['max_num_aps']
        self.sim_round_set = sim_config['simRoundSet']
        self.epsNumber = 0

        self.ns3Settings = ns3Settings

        # self.envNumber = str(np.random.choice(range(0,100)))
        self.envNumber = str(os.getpid())
        print('Environment number : ',self.envNumber)
        self.ns3Settings['envNumber'] = self.envNumber

        # Defining the action space for all APs
        action_shape = {
            'new_ampdu': gym.spaces.Box(low=0, high=66, shape=(self.max_num_aps,), dtype=np.int32),
            'new_txPower': gym.spaces.Box(low=10, high=30, shape=(self.max_num_aps,), dtype=np.int32),
            'new_mcs': gym.spaces.Box(low=0, high=11, shape=(self.max_num_aps,), dtype=np.int32),
            'new_edThreshold': gym.spaces.Box(low=-85, high=-60, shape=(self.max_num_aps,), dtype=np.int32),
            'new_minCw': gym.spaces.Box(low=0, high=63, shape=(self.max_num_aps,), dtype=np.int32),
            'new_aifsn': gym.spaces.Box(low=1, high=10, shape=(self.max_num_aps,), dtype=np.int32),
        }
        self.action_space = gym.spaces.Dict(action_shape)
        #Defining the observation space for all APs
        observation_shape = {
            'new_ampdu': gym.spaces.Box(low=0, high=66, shape=(self.max_num_aps,), dtype=np.int32),
            'new_txPower': gym.spaces.Box(low=-10, high=30, shape=(self.max_num_aps,), dtype=np.int32),
            'new_mcs': gym.spaces.Box(low=0, high=11, shape=(self.max_num_aps,), dtype=np.int32),
            'new_edThreshold': gym.spaces.Box(low=-85, high=-60, shape=(self.max_num_aps,), dtype=np.int32),
            'new_minCw': gym.spaces.Box(low=0, high=63, shape=(self.max_num_aps,), dtype=np.int32),
            'new_aifsn': gym.spaces.Box(low=0, high=10, shape=(self.max_num_aps,), dtype=np.int32),
            'ampdu': gym.spaces.Box(low=0, high=66, shape=(self.max_num_aps,), dtype=np.int32),
            'txPower': gym.spaces.Box(low=-10, high=30, shape=(self.max_num_aps,), dtype=np.int32),
            'mcs': gym.spaces.Box(low=0, high=11, shape=(self.max_num_aps,), dtype=np.int32),
            'edThreshold' : gym.spaces.Box(low=-85, high=0, shape=(self.max_num_aps,), dtype=np.int32),
            'minCw': gym.spaces.Box(low=0, high=63, shape=(self.max_num_aps,), dtype=np.int32),
            'aifsn': gym.spaces.Box(low=0, high=10, shape=(self.max_num_aps,), dtype=np.int32),
            'nodesNum': gym.spaces.Box(low=0, high=6, shape=(1,), dtype=np.int32),
            'trafficType' : gym.spaces.Box(low=0, high=1, shape=(self.max_num_aps,), dtype=np.int32),
            'udpLambda' : gym.spaces.Box(low=0, high=20000, shape=(self.max_num_aps,), dtype=np.int32),
            'packetSize' : gym.spaces.Box(low=0, high=1500, shape=(self.max_num_aps,), dtype=np.int32),
            'ueRxPowerConnected' : gym.spaces.Box(low=-200, high=40, shape=(self.max_num_aps,), dtype=np.int32),
            'gnbRxPowerInterference' : gym.spaces.Box(low=-200, high=40, shape=(self.max_num_aps*(self.max_num_aps-1),), dtype=np.int32),
            'throughput': gym.spaces.Box(low=0, high=1000000000, shape=(self.max_num_aps,), dtype=np.int64),
            'delay' : gym.spaces.Box(low=0, high=100000, shape=(self.max_num_aps,), dtype=np.int32),
            'airtime' : gym.spaces.Box(low=0, high=100000, shape=(self.max_num_aps,), dtype=np.int32),
        }
        self.observation_space = gym.spaces.Dict(observation_shape)
        
        print("Debugging CentralEnv create dummyEnv")
        self.dummyEnv = Ns3Env(targetName="ns3ai_wifimac_gym",
            ns3Path="../../../../../", ns3Settings=ns3Settings, envNumber = self.envNumber)

        self.observation = self.observation_space.sample()

        # print("Debug self.obs data type at __init__() : ", type(self.observation))

        self.info = {}
        self.th_array = []

        if self.isEvaluation == True :
            currentTime = datetime.now().strftime("%H:%M:%S")
            fileName = self.output_dir + "eval_action_" + '_' + str(self.ns3Settings['envNumber']) + '_' + str(self.ns3Settings['simRound']) + '_' + currentTime + ".txt"
            self.f = open(fileName, "a")
            self.fCounter = 0
            endMessage = "\nStarting new episode " + self.ns3Settings['envNumber'] + " with --simRound=" + str(self.ns3Settings['simRound']) + "--numAgents=" + str(self.ns3Settings['numAgents']) + " --simTime=" + str(self.ns3Settings['simTime']) +  " --simRound=" + str(self.ns3Settings['simRound']) + " --trafficType=" + str(self.ns3Settings['trafficType']) + " --packetSize=" + str(self.ns3Settings['packetSize']) + " --udpLambda=" + str(self.new_udpLambda) + "\n"
            self.f.write(endMessage)
    
    def get_reward(self, alpha = 0.3, beta = 0.3):
        print("Debugging CentralEnv get_reward()")
        
        self.reward = 0
        
        if self.reward_type == 'option1' :
            for i in range(self.max_num_aps) :
                self.reward += self.normalize_throughput(self.observation['throughput'][i])         
        if self.reward_type == 'option3' :
            sum_airTime = 0
            sum_delay = 0
            for index in range(self.max_num_aps) :
                sum_airTime += self.normalize_airtime(self.observation['airtime'][index])
                sum_delay += self.normalize_delay(self.observation['delay'][index])
                if sum_airTime <= 0 :
                    sum_airTime = 1
                if sum_delay <= 0 :
                    sum_delay = 1

            for i in range(self.max_num_aps) :
                self.reward += self.normalize_throughput(self.observation['throughput'][i])/self.normalize_udpLambda(self.observation['udpLambda'][i]) - alpha*self.normalize_delay(self.observation['airtime'][i])/sum_airTime - beta*self.normalize_delay(self.observation['delay'][i])/sum_delay

        return self.reward
    
    def get_mean_th(self):
        total_th = 0
        for i in range (self.max_num_aps) :
            total_th += self.th_array[i]
        mean_th = total_th/self.num_ap
        
        return mean_th

    def normalize_throughput(self, throughput) :
        norm_factor = 100
        return throughput/norm_factor
    
    def normalize_delay(self, delay) :
        norm_factor = 1000
        return delay/norm_factor
    
    def normalize_airtime(self, airtime) :
        norm_factor = 1000
        return airtime/norm_factor
    
    def normalize_udpLambda(self, udpLambda) :
        norm_factor = 0
        for i in range (self.max_num_aps) :
            norm_factor += self.observation['udpLambda'][i]

        if udpLambda/norm_factor <= 0 :
            return 1
        else :
            return udpLambda/norm_factor
    
    def set_new_action(self, action_dict):
        print("Debugging CentralEnv set_new_action()")

        new_ampdu1 = action_dict['new_ampdu'][0]
        new_ampdu2 = action_dict['new_ampdu'][1]
        new_ampdu3 = action_dict['new_ampdu'][2]
        new_ampdu4 = action_dict['new_ampdu'][3]
        new_txPower1 = action_dict['new_txPower'][0]
        new_txPower2 = action_dict['new_txPower'][1]
        new_txPower3 = action_dict['new_txPower'][2]
        new_txPower4 = action_dict['new_txPower'][3]
        new_mcs1 = action_dict['new_mcs'][0]
        new_mcs2 = action_dict['new_mcs'][1]
        new_mcs3 = action_dict['new_mcs'][2]
        new_mcs4 = action_dict['new_mcs'][3]
        new_edThreshold1 = action_dict['new_edThreshold'][0]
        new_edThreshold2 = action_dict['new_edThreshold'][1]
        new_edThreshold3 = action_dict['new_edThreshold'][2]
        new_edThreshold4 = action_dict['new_edThreshold'][3]
        new_minCw1 = action_dict['new_minCw'][0]
        new_minCw2 = action_dict['new_minCw'][1]
        new_minCw3 = action_dict['new_minCw'][2]
        new_minCw4 = action_dict['new_minCw'][3]
        new_aifsn1 = action_dict['new_aifsn'][0]
        new_aifsn2 = action_dict['new_aifsn'][1]
        new_aifsn3 = action_dict['new_aifsn'][2]
        new_aifsn4 = action_dict['new_aifsn'][3]

        dummyAction =([new_ampdu1,
                     new_ampdu2,
                     new_ampdu3,
                     new_ampdu4, 
                     new_txPower1, 
                     new_txPower2,
                     new_txPower3,
                     new_txPower4,
                     new_mcs1, 
                     new_mcs2,
                     new_mcs3,
                     new_mcs4,
                     new_edThreshold1, 
                     new_edThreshold2,
                     new_edThreshold3,
                     new_edThreshold4,
                     new_minCw1, 
                     new_minCw2,
                     new_minCw3,
                     new_minCw4,
                     new_aifsn1,
                     new_aifsn2,
                     new_aifsn3,
                     new_aifsn4])
        
        return dummyAction
    
    def get_obs(self, dObs, dAction):
        print("Debugging CentralEnv get_obs()")
        

        if len(dObs) == 0:
            self.observation['ampdu'] = np.array([0,0,0,0]).astype(np.int32)
            self.observation['txPower'] = np.array([0,0,0,0]).astype(np.int32)
            self.observation['mcs'] = np.array([0,0,0,0]).astype(np.int32)
            self.observation['edThreshold'] = np.array([0,0,0,0]).astype(np.int32)
            self.observation['minCw'] = np.array([0,0,0,0]).astype(np.int32)
            self.observation['aifsn'] = np.array([0,0,0,0]).astype(np.int32)
            self.observation['nodesNum'] = np.array([self.num_ap]).astype(np.int32)
            self.observation['trafficType'] = np.array([0,0,0,0]).astype(np.int32)
            self.observation['udpLambda'] = np.array([0,0,0,0]).astype(np.int32)
            self.observation['packetSize'] = np.array([0,0,0,0]).astype(np.int32)
            self.observation['ueRxPowerConnected'] = np.array([0,0,0,0]).astype(np.int32)
            self.observation['gnbRxPowerInterference'] = np.array(
                [0,0,0,0,0,0,0,0,0,0,0,0]).astype(np.int32)
            self.observation['throughput'] = np.array([0,0,0,0]).astype(np.int64)
            self.observation['delay'] = np.array([0,0,0,0]).astype(np.int32)
            self.observation['airtime'] = np.array([0,0,0,0]).astype(np.int32)

            self.th_array = np.array([0,0,0,0]).astype(np.int64)

        else :
            # print("Data type of dObs", type(dObs))
            self.observation['ampdu'] = np.array([dObs[0],dObs[1],dObs[2],dObs[3]]).astype(np.int32)
            self.observation['txPower'] = np.array([dObs[4],dObs[5],dObs[6],dObs[7]]).astype(np.int32)
            self.observation['mcs'] = np.array([dObs[8],dObs[9],dObs[10],dObs[11]]).astype(np.int32)
            self.observation['edThreshold'] = np.array([dObs[12],dObs[13],dObs[14],dObs[15]]).astype(np.int32)
            self.observation['minCw'] = np.array([dObs[16],dObs[17],dObs[18],dObs[19]]).astype(np.int32)
            self.observation['aifsn'] = np.array([dObs[20],dObs[21],dObs[22],dObs[23]]).astype(np.int32)
            self.observation['nodesNum'] = np.array([self.num_ap]).astype(np.int32)
            self.observation['trafficType'] = np.array([dObs[24],dObs[25],dObs[26],dObs[27]]).astype(np.int32)
            self.observation['udpLambda'] = np.array([dObs[28],dObs[29],dObs[30],dObs[31]]).astype(np.int32)
            self.observation['packetSize'] = np.array([dObs[32],dObs[33],dObs[34],dObs[35]]).astype(np.int32)
            self.observation['ueRxPowerConnected'] = np.array([dObs[36],dObs[41],dObs[46],dObs[51]]).astype(np.int32)
            self.observation['gnbRxPowerInterference'] = np.array(
                [dObs[37],dObs[38],dObs[39],
                 dObs[40],dObs[42],dObs[43],
                 dObs[44],dObs[45],dObs[47],
                 dObs[48],dObs[49],dObs[50],]).astype(np.int32)
            self.observation['throughput'] = np.array([dObs[52],dObs[53],dObs[54],dObs[55]]).astype(np.int64)
            self.observation['delay'] = np.array([dObs[56],dObs[57],dObs[58],dObs[59]]).astype(np.int32)
            self.observation['airtime'] = np.array([dObs[60],dObs[61],dObs[62],dObs[63]]).astype(np.int32)
            

            self.th_array = np.array([dObs[52],dObs[53],dObs[54],dObs[55]]).astype(np.int64)

        # print("Data type of dAction", type(dAction))
        self.observation['new_ampdu'] = np.array([dAction[0], dAction[1], dAction[2], dAction[3]]).astype(np.int32)
        self.observation['new_txPower'] = np.array([dAction[4], dAction[5], dAction[6], dAction[7]]).astype(np.int32)
        self.observation['new_mcs'] = np.array([dAction[8], dAction[9], dAction[10], dAction[11]]).astype(np.int32)
        self.observation['new_edThreshold'] = np.array([dAction[12], dAction[13], dAction[14], dAction[15]]).astype(np.int32)
        self.observation['new_minCw'] = np.array([dAction[16], dAction[17], dAction[18], dAction[19]]).astype(np.int32)
        self.observation['new_aifsn'] = np.array([dAction[20], dAction[21], dAction[22], dAction[23]]).astype(np.int32)

        return self.observation

    def render(self):
        pass

    def reset(self, seed=None, options=None):
        print("Debugging CentralEnv reset()")

        if self.done == True :
            self.ns3Settings['simRound'] = np.random.randint(1,1000)
            if self.isEvaluation == True :
                self.ns3Settings['simRound'] = self.sim_round_set[self.epsNumber]
                self.epsNumber += 1
            if self.ns3Settings['customEpisode'] == False :
                self.ns3Settings['numAgents'] = np.random.randint(low=self.min_ap_interval,high=self.max_ap_interval) 
                self.num_ap = self.ns3Settings['numAgents']
                self.ns3Settings['trafficType'] = np.random.choice(['UDP_CBR', 'BURST'])
                self.ns3Settings['packetSize'] = 1500
                self.ns3Settings['fragmentSize'] = 1500
                self.new_udpLambda =  np.random.randint(low=1, high=500, size=4)*40
                self.ns3Settings['udpLambda1'] = self.new_udpLambda[0]
                self.ns3Settings['udpLambda2'] = self.new_udpLambda[1]
                self.ns3Settings['udpLambda3'] = self.new_udpLambda[2]
                self.ns3Settings['udpLambda4'] = self.new_udpLambda[3]
            self.dummyEnv.ns3Settings = self.ns3Settings
            print('Starting next episode with simRound : ',self.dummyEnv.ns3Settings['simRound'],' ap_nums : ', self.dummyEnv.ns3Settings['numAgents'], ', trafficType : ', self.dummyEnv.ns3Settings['trafficType'], ', packetSize : ', self.dummyEnv.ns3Settings['packetSize'], ', fragmentSize : ', self.dummyEnv.ns3Settings['fragmentSize'],', udpLambda : ', self.new_udpLambda)
            if self.isEvaluation == True :
                self.f.close()
                self.fCounter = 0
                currentTime = datetime.now().strftime("%H:%M:%S")
                fileName = self.output_dir + "eval_action_" + '_' + str(self.ns3Settings['envNumber']) + '_' + str(self.ns3Settings['simRound']) + '_' + currentTime + ".txt"
                self.f = open(fileName, "a")
                endMessage = "\nStarting new episode " + self.ns3Settings['envNumber'] + " with --simRound=" + str(self.ns3Settings['simRound']) + "--numAgents=" + str(self.ns3Settings['numAgents']) + " --simTime=" + str(self.ns3Settings['simTime']) +  " --simRound=" + str(self.ns3Settings['simRound']) + " --trafficType=" + str(self.ns3Settings['trafficType']) + " --packetSize=" + str(self.ns3Settings['packetSize']) + " --udpLambda=" + str(self.new_udpLambda) + "\n"
                self.f.write(endMessage)
                # evalTitle = "agent_num,new_ampdu,new_txPower,new_mcs,new_edThreshold,new_backoffType,new_minCw,new_slotTime,new_aifsn"+"\n"
                evalTitle = "agent_num,new_ampdu,new_txPower,new_mcs,new_edThreshold,new_minCw,new_aifsn"+"\n"
                self.f.write(evalTitle)
            self.dummyEnv.reset()

        self.reward = 0

        self.observation = self.observation_space.sample()
        print("Debug self.observation data type at reset() : ", self.observation)

        return self.observation,{} 

    def step(self, action):

        print("Debugging CentralEnv step()")

        # Get action from RLlib agent
        # print("CentralEnv action :",action)

        dAction = self.set_new_action(action)
        # print("dAction :",action)
        dObs, dReward, dDone, _, dInfo = self.dummyEnv.step(dAction)
        # print("Debugging CentralEnv observation dObs : ",dObs)

        # Get state results from dummyEnv
        self.observation = self.get_obs(dObs, dAction)
        # print("Debug self.observation data type at step() : ", self.observation)

        self.reward = self.get_reward(alpha=self.alpha, beta = self.beta)
        print("Debugging CentralEnv reward : ", self.reward)

        self.done = dDone
        # print("Debugging CentralEnv done : ", self.done)

        self.mean_th = self.get_mean_th()

        self.info = {
            'mean_th' : self.mean_th,
            'reward' : self.reward,
        }

        if self.isEvaluation == True :
            self.fCounter += 1
            eval_report_action = "Action step " + str(self.fCounter) + " with reward : " + str(self.reward) + "\n"
            self.f.write(eval_report_action)
            for i in range(self.max_num_aps) :
                # eval_report = str(i)+","+str(action['new_ampdu'][i])+","+str(action['new_txPower'][i])+","+str(action['new_mcs'][i])+","+str(action['new_edThreshold'][i])+","+str(action['new_backoffType'][i])+","+str(action['new_minCw'][i])+","+str(action['new_slotTime'][i])+","+str(action['new_aifsn'][i])+"\n"
                eval_report = str(i)+","+str(action['new_ampdu'][i])+","+str(action['new_txPower'][i])+","+str(action['new_mcs'][i])+","+str(action['new_edThreshold'][i])+","+str(action['new_minCw'][i])+","+str(action['new_aifsn'][i])+"\n"
                self.f.write(eval_report)

        return self.observation, self.reward, self.done, self.done, self.info

    def close(self):
        print("Debugging CentralEnv close()")
        # environment is not needed anymore, so kill subprocess in a straightforward way
        self.dummyEnv.exp.kill()
        # destroy the message interface and its shared memory segment
        del self.dummyEnv.exp
        if self.isEvaluation == True :
            self.f.close()

class MultiAgentDecisionWifiEnv (MultiAgentEnv):
    """Custom Environment that follows gym interface."""

    metadata = {"render_modes": ["human"], "render_fps": 30}

    def __init__(self, ns3Settings, sim_config):
        super().__init__()

        # self.seed = sim_config['seed']
        # np.random.seed(self.seed)
        # random.seed(self.seed)

        self.reward = {}
        self.done = {}
        self.max_num_aps = 4
        self._agent_ids = set(range(self.max_num_aps))
        print('Agent IDs : ',self._agent_ids)
        self.num_ap = sim_config['num_ap']
        print('Number of APs : ',self.num_ap)
        self.reward_type = sim_config['reward_type']
        self.isEvaluation = sim_config['isEvaluation']
        self.new_udpLambda = sim_config['new_udpLambda']
        self.output_dir = sim_config['output_dir']
        self.alpha = sim_config['alpha']
        self.beta = sim_config['beta']
        self.min_ap_interval = sim_config['min_num_aps']
        self.max_ap_interval = sim_config['max_num_aps']
        self.sim_round_set = sim_config['simRoundSet']
        self.epsNumber = 0 
        self.random_order = random.sample(range(4), 4)
        self.info = {}
        self.th_array = []
        self.terminateds = set()
        self.truncateds = set()

        self.ns3Settings = ns3Settings
        # self.envNumber = str(np.random.choice(range(0,100)))
        self.envNumber = str(os.getpid())
        print('Environment number : ',self.envNumber)
        self.ns3Settings['envNumber'] = self.envNumber

        # Defining the action space for all APs
        action_shape = {
            'new_ampdu': gym.spaces.Box(low=0, high=66, shape=(1,), dtype=np.int32),
            'new_txPower': gym.spaces.Box(low=10, high=30, shape=(1,), dtype=np.int32),
            'new_mcs': gym.spaces.Box(low=0, high=11, shape=(1,), dtype=np.int32),
            'new_edThreshold': gym.spaces.Box(low=-85, high=-60, shape=(1,), dtype=np.int32),
            'new_minCw': gym.spaces.Box(low=0, high=63, shape=(1,), dtype=np.int32),
            'new_aifsn': gym.spaces.Box(low=1, high=10, shape=(1,), dtype=np.int32),
        }
        self._action_space_in_preferred_format = True
        single_action_space = gym.spaces.Dict(action_shape)
        self.action_space = gym.spaces.Dict(
            {i : single_action_space for i in range (self.max_num_aps)} 
        )

        #Defining the observation space for all APs
        observation_shape = {
            'new_ampdu': gym.spaces.Box(low=0, high=66, shape=(1,), dtype=np.int32),
            'new_txPower': gym.spaces.Box(low=-10, high=30, shape=(1,), dtype=np.int32),
            'new_mcs': gym.spaces.Box(low=0, high=11, shape=(1,), dtype=np.int32),
            'new_edThreshold': gym.spaces.Box(low=-85, high=-60, shape=(1,), dtype=np.int32),
            'new_minCw': gym.spaces.Box(low=0, high=63, shape=(1,), dtype=np.int32),
            'new_aifsn': gym.spaces.Box(low=0, high=10, shape=(1,), dtype=np.int32),
            'ampdu': gym.spaces.Box(low=0, high=66, shape=(1,), dtype=np.int32),
            'txPower': gym.spaces.Box(low=-10, high=30, shape=(1,), dtype=np.int32),
            'mcs': gym.spaces.Box(low=0, high=28, shape=(1,), dtype=np.int32),
            'edThreshold' : gym.spaces.Box(low=-85, high=0, shape=(1,), dtype=np.int32),
            'minCw': gym.spaces.Box(low=0, high=63, shape=(1,), dtype=np.int32),
            'aifsn': gym.spaces.Box(low=0, high=10, shape=(1,), dtype=np.int32),
            'nodesNum': gym.spaces.Box(low=0, high=6, shape=(1,), dtype=np.int32),
            'trafficType' : gym.spaces.Box(low=0, high=1, shape=(1,), dtype=np.int32),
            'udpLambda' : gym.spaces.Box(low=0, high=20000, shape=(self.max_num_aps,), dtype=np.int32),
            'packetSize' : gym.spaces.Box(low=0, high=1500, shape=(1,), dtype=np.int32),
            'ueRxPowerConnected' : gym.spaces.Box(low=-200, high=40, shape=(1,), dtype=np.int32),
            'gnbRxPowerInterference' : gym.spaces.Box(low=-200, high=40, shape=(self.max_num_aps-1,), dtype=np.int32),
            'throughput': gym.spaces.Box(low=0, high=1000000000, shape=(self.max_num_aps,), dtype=np.int64),
            'delay' : gym.spaces.Box(low=0, high=100000, shape=(self.max_num_aps,), dtype=np.int32),
            'airtime' : gym.spaces.Box(low=0, high=100000, shape=(self.max_num_aps,), dtype=np.int32),
        }
        self._obs_space_in_preferred_format = True
        single_observation_space = gym.spaces.Dict(observation_shape)
        self.observation_space = gym.spaces.Dict(
            {i : single_observation_space for i in range (self.max_num_aps)} 
        )

        self.dummyEnv = Ns3Env(targetName="ns3ai_wifimac_gym",
            ns3Path="../../../../../", ns3Settings=ns3Settings, envNumber = self.envNumber)

        self.observation = {i : self.observation_space[i].sample() for i in range (self.max_num_aps)} 
        # self.reset_obs_space()
        # print("Debug self.obs data type at __init__() : ", type(self.observation))

        if self.isEvaluation == True :
            self.fCounter = 0
            currentTime = datetime.now().strftime("%H:%M:%S")
            fileName = self.output_dir + "eval_action_" + '_' + str(self.ns3Settings['envNumber']) + '_' + str(self.ns3Settings['simRound']) + '_' + currentTime + ".txt"
            self.f = open(fileName, "a")
            endMessage = "\nStarting new episode " + self.ns3Settings['envNumber'] + " with --simRound=" + str(self.ns3Settings['simRound']) + "--numAgents=" + str(self.ns3Settings['numAgents']) + " --simTime=" + str(self.ns3Settings['simTime']) +  " --simRound=" + str(self.ns3Settings['simRound']) + " --trafficType=" + str(self.ns3Settings['trafficType']) + " --packetSize=" + str(self.ns3Settings['packetSize']) + " --udpLambda=" + str(self.new_udpLambda) + "\n"
            self.f.write(endMessage)
            evalTitle = "agent_num,new_ampdu,new_txPower,new_mcs,new_edThreshold,new_backoffType,new_minCw,new_slotTime,new_aifsn"+"\n"
            self.f.write(evalTitle)

    def set_new_action(self, action_dict):
        # print("Debugging MultiAgentDecisionEnv set_new_action()", action_dict)
        
        new_ampdu1 = action_dict[0]['new_ampdu'][0]
        new_ampdu2 = action_dict[1]['new_ampdu'][0]
        new_ampdu3 = action_dict[2]['new_ampdu'][0]
        new_ampdu4 = action_dict[3]['new_ampdu'][0]
        new_txPower1 = action_dict[0]['new_txPower'][0]
        new_txPower2 = action_dict[1]['new_txPower'][0]
        new_txPower3 = action_dict[2]['new_txPower'][0]
        new_txPower4 = action_dict[3]['new_txPower'][0]
        new_mcs1 = action_dict[0]['new_mcs'][0]
        new_mcs2 = action_dict[1]['new_mcs'][0]
        new_mcs3 = action_dict[2]['new_mcs'][0]
        new_mcs4 = action_dict[3]['new_mcs'][0]
        new_edThreshold1 = action_dict[0]['new_edThreshold'][0]
        new_edThreshold2 = action_dict[1]['new_edThreshold'][0]
        new_edThreshold3 = action_dict[2]['new_edThreshold'][0]
        new_edThreshold4 = action_dict[3]['new_edThreshold'][0]
        new_minCw1 = action_dict[0]['new_minCw'][0]
        new_minCw2 = action_dict[1]['new_minCw'][0]
        new_minCw3 = action_dict[2]['new_minCw'][0]
        new_minCw4 = action_dict[3]['new_minCw'][0]
        new_aifsn1 = action_dict[0]['new_aifsn'][0]
        new_aifsn2 = action_dict[1]['new_aifsn'][0]
        new_aifsn3 = action_dict[2]['new_aifsn'][0]
        new_aifsn4 = action_dict[3]['new_aifsn'][0]

        dummyAction =([new_ampdu1,
                     new_ampdu2,
                     new_ampdu3,
                     new_ampdu4, 
                     new_txPower1, 
                     new_txPower2,
                     new_txPower3,
                     new_txPower4,
                     new_mcs1, 
                     new_mcs2,
                     new_mcs3,
                     new_mcs4,
                     new_edThreshold1, 
                     new_edThreshold2,
                     new_edThreshold3,
                     new_edThreshold4,
                     new_minCw1, 
                     new_minCw2,
                     new_minCw3,
                     new_minCw4,
                     new_aifsn1,
                     new_aifsn2,
                     new_aifsn3,
                     new_aifsn4])
        
        return dummyAction

    def get_obs(self, dObs, dAction):
        # print("Debugging MultiAgentDecisionEnv get_obs()")
        for i in range(self.max_num_aps) :
            if len(dObs) == 0:
                self.observation[i]['ampdu'] = np.array([0]).astype(np.int32)
                self.observation[i]['txPower'] = np.array([0]).astype(np.int32)
                self.observation[i]['mcs'] = np.array([0]).astype(np.int32)
                self.observation[i]['edThreshold'] = np.array([0]).astype(np.int32)
                self.observation[i]['minCw'] = np.array([0]).astype(np.int32)
                self.observation[i]['aifsn'] = np.array([0]).astype(np.int32)
                self.observation[i]['nodesNum'] = np.array([self.num_ap]).astype(np.int32)
                self.observation[i]['trafficType'] = np.array([0]).astype(np.int32)
                self.observation[i]['udpLambda'] = np.array([0,0,0,0]).astype(np.int32)
                self.observation[i]['packetSize'] = np.array([0]).astype(np.int32)
                self.observation[i]['ueRxPowerConnected'] = np.array([0]).astype(np.int32)
                self.observation[i]['gnbRxPowerInterference'] = np.array([-200,-200,-200]).astype(np.int32)
                self.observation[i]['throughput'] = np.array([0,0,0,0]).astype(np.int64)
                self.observation[i]['delay'] = np.array([0,0,0,0]).astype(np.int32)
                self.observation[i]['airtime'] = np.array([0,0,0,0]).astype(np.int32)

                self.th_array = np.array([0,0,0,0]).astype(np.int64)

            else :
                self.observation[i]['ampdu'] = np.array([dObs[i]]).astype(np.int32)
                self.observation[i]['txPower'] = np.array([dObs[self.max_num_aps+i]]).astype(np.int32)
                self.observation[i]['mcs'] = np.array([dObs[2*self.max_num_aps+i]]).astype(np.int32)
                self.observation[i]['edThreshold'] = np.array([dObs[3*self.max_num_aps+i]]).astype(np.int32)
                self.observation[i]['minCw'] = np.array([dObs[4*self.max_num_aps+i]]).astype(np.int32)
                self.observation[i]['aifsn'] = np.array([dObs[5*self.max_num_aps+i]]).astype(np.int32)               
                self.observation[i]['nodesNum'] = np.array([self.num_ap]).astype(np.int32)
                self.observation[i]['trafficType'] = np.array([dObs[6*self.max_num_aps+i]]).astype(np.int32)
                self.observation[i]['udpLambda'] = np.array([dObs[28],dObs[29],dObs[30],dObs[31]]).astype(np.int32)
                self.observation[i]['packetSize'] = np.array([dObs[8*self.max_num_aps+i]]).astype(np.int32)
                self.observation[i]['ueRxPowerConnected'] = np.array([dObs[9*self.max_num_aps+i*5]]).astype(np.int32)
                self.observation[i]['throughput'] = np.array([dObs[52],dObs[53],dObs[54],dObs[55]]).astype(np.int64)
                self.observation[i]['delay'] = np.array([dObs[56],dObs[57],dObs[58],dObs[59]]).astype(np.int32)
                self.observation[i]['airtime'] = np.array([dObs[60],dObs[61],dObs[62],dObs[63]]).astype(np.int32)

                self.th_array = np.array([dObs[52],dObs[53],dObs[54],dObs[55]]).astype(np.int64)

            self.observation[i]['new_ampdu'] = np.array([dAction[i]]).astype(np.int32)
            self.observation[i]['new_txPower'] = np.array([dAction[self.max_num_aps+i]]).astype(np.int32)
            self.observation[i]['new_mcs'] = np.array([dAction[2*self.max_num_aps+i]]).astype(np.int32)
            self.observation[i]['new_edThreshold'] = np.array([dAction[3*self.max_num_aps+i]]).astype(np.int32)
            self.observation[i]['new_minCw'] = np.array([dAction[4*self.max_num_aps+i]]).astype(np.int32)
            self.observation[i]['new_aifsn'] = np.array([dAction[5*self.max_num_aps+i]]).astype(np.int32)

        if len(dObs) != 0:
            self.observation[0]['gnbRxPowerInterference'] = np.array([dObs[37],dObs[38],dObs[39]]).astype(np.int32) 
            self.observation[1]['gnbRxPowerInterference'] = np.array([dObs[40],dObs[42],dObs[43]]).astype(np.int32) 
            self.observation[2]['gnbRxPowerInterference'] = np.array([dObs[44],dObs[45],dObs[47]]).astype(np.int32) 
            self.observation[3]['gnbRxPowerInterference'] = np.array([dObs[48],dObs[49],dObs[50]]).astype(np.int32) 

        # For partial observability
        for i in range(self.max_num_aps) :
            detect = self.observation[i]['gnbRxPowerInterference'] >= self.observation[i]['edThreshold'][0]
            self.observation[i]['nodesNum'][0] = np.count_nonzero(detect) + 1
            detect_index = np.delete(range(self.max_num_aps),i)
            for j in range(self.max_num_aps-1) :
                if detect[j] == False :
                    self.observation[i]['udpLambda'][detect_index[j]] = 0
                    self.observation[i]['throughput'][detect_index[j]] = 0
                    self.observation[i]['delay'][detect_index[j]] = 0
                    self.observation[i]['airtime'][detect_index[j]] = 0
        
        # For constantly changing ap_num
        if self.isEvaluation != True and self.num_ap < self.max_num_aps:
            temp = {}
            for i in range (self.max_num_aps) :
                temp[i] = self.observation[self.random_order[i]].copy()
            
            for i in range (self.max_num_aps) :
                temp[i]['udpLambda'] = self.rearrange_np_array(temp[i]['udpLambda'],self.random_order)
                temp[i]['throughput'] = self.rearrange_np_array(temp[i]['throughput'],self.random_order)
                temp[i]['delay'] = self.rearrange_np_array(temp[i]['delay'],self.random_order)
                temp[i]['airtime'] = self.rearrange_np_array(temp[i]['airtime'],self.random_order)
                random_gnb = self.random_order.copy()
                random_gnb.remove(i)
                for j in range (self.max_num_aps-1) :
                    if random_gnb[j] > i :
                        random_gnb[j] -= 1
                temp[i]['gnbRxPowerInterference'] = self.rearrange_np_array(temp[i]['gnbRxPowerInterference'],random_gnb)

            self.observation = temp

    def rearrange_np_array(self, arr, sequence):
        # Convert the sequence list to a NumPy array for indexing
        sequence_np = np.array(sequence)
        # Rearrange the array based on the sequence
        rearranged_arr = arr[sequence_np]
        return rearranged_arr

    def normalize_throughput(self, throughput):
        norm_factor = 100
        return throughput/norm_factor
    
    def normalize_delay(self, delay):
        norm_factor = 1000
        return delay/norm_factor
    
    def normalize_airtime(self, airtime):
        norm_factor = 1000
        return airtime/norm_factor
    
    def normalize_udpLambda(self, udpLambda, ap_index) :
        norm_factor = 1
        for i in range (self.max_num_aps) :
            norm_factor += self.observation[ap_index]['udpLambda'][i]
        if udpLambda/norm_factor <= 0 :
            return 1
        else :
            return udpLambda/norm_factor
    
    def get_reward (self, alpha=0.3, beta=0.3):
        # print("Debugging MultiAgentDecisionEnv get_reward()")
        self.reward = {}
        if self.reward_type == 'option1' :
            for i in range(self.max_num_aps) :
                counter = 0
                self.reward[i] = 0.0
                detect_reward = 0.0

                self_throughput = self.normalize_throughput(self.observation[i]['throughput'][i])
                throughput = np.delete(self.observation[i]['throughput'],i)
                detect = self.observation[i]['gnbRxPowerInterference'] >= self.observation[i]['edThreshold'][0]
                for j in range(self.max_num_aps-1) :
                    if detect[j] == True:
                        detect_reward += self.normalize_throughput(throughput[j])
                        counter += 1
                if counter <= 0 :
                    self.reward[i] = self_throughput
                else :
                    self.reward[i] = self_throughput +  alpha*detect_reward/counter
        if self.reward_type == 'option2' :
            for i in range(self.max_num_aps) :
                counter = 0
                self.reward[i] = 0.0
                sum_airTime = 0

                for index in range(self.max_num_aps) :
                    sum_airTime += self.normalize_airtime(self.observation['airtime'][index])
                    if sum_airTime <= 0 :
                        sum_airTime = 1

                self.reward[i] = self.normalize_throughput(self.observation[i]['throughput'][i]) - alpha*self.normalize_airtime(self.observation[i]['airtime'][i])/sum_airTime - beta*self.normalize_delay(self.observation[i]['delay'][i])
        if self.reward_type == 'option3' :
            for i in range(self.max_num_aps) :
                counter = 0
                self.reward[i] = 0.0
                sum_airTime = 0
                sum_delay = 0

                for index in range(self.max_num_aps) :
                    sum_airTime += self.normalize_airtime(self.observation[i]['airtime'][index])
                    sum_delay += self.normalize_delay(self.observation[i]['delay'][index])
                    if sum_airTime <= 0 :
                        sum_airTime = 1
                    if sum_delay <= 0 :
                        sum_delay = 1
                self.reward[i] = self.normalize_throughput(self.observation[i]['throughput'][i])/self.normalize_udpLambda(self.observation[i]['udpLambda'][i],i) - alpha*self.normalize_airtime(self.observation[i]['airtime'][i])/sum_airTime - beta*self.normalize_delay(self.observation[i]['delay'][i])/sum_delay
        
        # print("with reward : ", self.reward)
        
        return self.reward               

    def get_mean_th(self):
        total_th = 0
        for i in range (self.max_num_aps) :
            total_th += self.th_array[i]
        mean_th = total_th/self.num_ap
        
        return mean_th
    
    def get_done(self, dDone):
        # print("Debugging MultiAgentDecisionEnv get_done()")
        for i in range(self.max_num_aps) :
            self.done[i] = dDone
        return self.done
    
    def render(self):
        pass

    def step(self, action):
        print("Debugging MultiAgentDecisionEnv step()")
        prev_obs = self.observation

        dAction = self.set_new_action(action)

        dObs, dReward, dDone, _, dInfo = self.dummyEnv.step(dAction)

        self.get_obs(dObs, dAction)
        # print("Debug self.observation data type at step() : ", self.observation)

        self.get_reward(alpha= self.alpha,beta=self.beta)

        self.done = dDone

        self.mean_th = self.get_mean_th()

        truncated, info, terminated = {}, {}, {} 
        for i in range(self.max_num_aps) :
            truncated[i] = False
            terminated[i] = dDone
            info[i] = {
                'mean_th' : self.mean_th,
                'reward' : self.reward[i],
            }

        terminated["__all__"] = dDone
        truncated["__all__"] = dDone

        print("Debugging MultiAgentTestEnv step() with reward : ", self.reward)
        # print("Debugging MultiAgentTestEnv step() with truncated : ", truncated)
        # print("Debugging MultiAgentTestEnv step() with terminated : ", terminated)

        total_reward = 0
        for i in range(self.max_num_aps) :
            total_reward += self.reward[i]

        if self.isEvaluation == True :
            self.fCounter += 1
            eval_report_action = "Action step " + str(self.fCounter) + " with reward : " + str(total_reward) + "\n"
            self.f.write(eval_report_action)
            for i in range(self.max_num_aps) :
                eval_report = str(i)+","+str(action[i]['new_ampdu'][0])+","+str(action[i]['new_txPower'][0])+","+str(action[i]['new_mcs'][0])+","+str(action[i]['new_edThreshold'][0])+","+str(action[i]['new_backoffType'][0])+","+str(action[i]['new_minCw'][0])+","+str(action[i]['new_slotTime'][0])+","+str(action[i]['new_aifsn'][0])+"\n"
                self.f.write(eval_report)
                
        return self.observation, self.reward, terminated, truncated, info
    
    def reset(self, *, seed=None, options=None):
        print("Debugging MultiAgentDecisionEnv reset()")

        if self.done == True :
            self.random_order = random.sample(range(4), 4)
            self.ns3Settings['simRound'] = np.random.randint(1,1000) 
            if self.isEvaluation == True :
                self.ns3Settings['simRound'] = self.sim_round_set[self.epsNumber]
                self.epsNumber += 1
            if self.ns3Settings['customEpisode'] == False :
                self.ns3Settings['numAgents'] = np.random.randint(low=self.min_ap_interval,high=self.max_ap_interval) 
                self.num_ap = self.ns3Settings['numAgents']
                self.ns3Settings['trafficType'] = np.random.choice(['UDP_CBR', 'BURST'])
                self.ns3Settings['packetSize'] = 1500
                self.ns3Settings['fragmentSize'] = 1500
                self.new_udpLambda =  np.random.randint(low=1, high=500, size=4)*40
                self.ns3Settings['udpLambda1'] = self.new_udpLambda[0]
                self.ns3Settings['udpLambda2'] = self.new_udpLambda[1]
                self.ns3Settings['udpLambda3'] = self.new_udpLambda[2]
                self.ns3Settings['udpLambda4'] = self.new_udpLambda[3]
            self.dummyEnv.ns3Settings = self.ns3Settings
            print('Starting next episode with simRound : ',self.dummyEnv.ns3Settings['simRound'],' ap_nums : ', self.dummyEnv.ns3Settings['numAgents'], ', trafficType : ', self.dummyEnv.ns3Settings['trafficType'], ', packetSize : ', self.dummyEnv.ns3Settings['packetSize'], ', fragmentSize : ', self.dummyEnv.ns3Settings['fragmentSize'],', udpLambda : ', self.new_udpLambda)
            if self.isEvaluation == True :
                self.f.close()
                self.fCounter = 0
                currentTime = datetime.now().strftime("%H:%M:%S")
                fileName = self.output_dir + "eval_action_" + '_' + str(self.ns3Settings['envNumber']) + '_' + str(self.ns3Settings['simRound']) + '_' + currentTime + ".txt"
                self.f = open(fileName, "a")
                endMessage = "\nStarting new episode " + self.ns3Settings['envNumber'] + " with --simRound=" + str(self.ns3Settings['simRound']) + "--numAgents=" + str(self.ns3Settings['numAgents']) + " --simTime=" + str(self.ns3Settings['simTime']) +  " --simRound=" + str(self.ns3Settings['simRound']) + " --trafficType=" + str(self.ns3Settings['trafficType']) + " --packetSize=" + str(self.ns3Settings['packetSize']) + " --udpLambda=" + str(self.new_udpLambda) + "\n"
                self.f.write(endMessage)
                evalTitle = "agent_num,new_ampdu,new_txPower,new_mcs,new_edThreshold,new_backoffType,new_minCw,new_slotTime,new_aifsn"+"\n"
                self.f.write(evalTitle)
            self.dummyEnv.reset()

        self.terminateds = set()
        self.truncateds = set()
        self.observation = {i: self.observation_space[i].sample() for i in range (self.max_num_aps)}
        # self.reset_obs_space()
        # print("Debug self.observation data type at reset() : ", self.observation)

        return self.observation,{} 
    
    def close(self):
        print("Debugging MultiAgentEnv close()")
        # environment is not needed anymore, so kill subprocess in a straightforward way
        self.dummyEnv.exp.kill()
        # destroy the message interface and its shared memory segment
        del self.dummyEnv.exp
        if self.isEvaluation == True :
            self.f.close()
    
