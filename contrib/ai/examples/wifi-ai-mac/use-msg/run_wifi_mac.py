# -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-
# Copyright (c) 2020 Huazhong University of Science and Technology, Dian Group
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation;
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#
# Author: Pengyu Liu <eic_lpy@hust.edu.cn>
#         Hao Yin <haoyin@uw.edu>

import ray
from ray import air
from ray import tune
from ray.util import inspect_serializability
from ray.rllib.env.env_context import EnvContext
from ray.rllib.algorithms import ppo 
from ray.rllib.utils.framework import try_import_tf, try_import_torch
from ray.rllib.utils.test_utils import check_learning_achieved
from ray.tune.logger import pretty_print
from ray.tune.registry import get_trainable_cls
from ray.rllib.algorithms.ppo import PPOConfig
from ray.rllib.algorithms.ppo import PPO
# from ray.rllib.agents.ppo import PPOTrainer
from ray.tune.registry import register_env

import gymnasium as gym
from dataclasses import dataclass
import sys
import random
import ns3ai_wifimac_msg_py as py_binding
from ns3ai_utils import Experiment

import os
import argparse

import numpy as np
import matplotlib.pyplot as plt
import traceback
os.environ['RAY_PICKLE_VERBOSE_DEBUG'] = '1'
RAY_DEDUP_LOGS=0

class CentralDevisionEnv (gym.Env):
    """Custom Environment that follows gym interface."""

    metadata = {"render_modes": ["human"], "render_fps": 30}

    def __init__(self, ns3Settings):
        super().__init__()

        self.exp = Experiment("ns3ai_wifimac_msg", "../../../../../", py_binding, handleFinish=True)
        self.msgInterface = self.exp.run(setting=ns3Settings, show_output=True)
        
        self.reward = 0
        self.done = False
        self.num_ap = 2

        # Defining the action space for all APs
        self.action_space = gym.spaces.Box(low=0, high=10, shape=(self.num_ap,), dtype=np.int32)

        #Defining the observation space for all APs
        self.observation_space = gym.spaces.Box(low=0, high=10000, shape=(3*self.num_ap,), dtype=np.int32)

        self.obs = self.observation_space.sample()
        print("Debug self.obs data type at __init__() : ", type(self.obs))

    def get_reward(self, dObs):
        print("Debugging CentralEnv get_reward()")
        
        if len(dObs) == 0:
            self.reward = 0
        else :
            self.reward = (self.obs[4]+self.obs[5])/2
        
        print("with reward : ", self.reward)

        return self.reward
    
    def set_new_action(self, action_dict):

        new_mcot1 = action_dict[0]
        new_mcot2 = action_dict[1]

        self.msgInterface.PySendBegin()
        self.msgInterface.GetPy2CppStruct().new_mcot1 = new_mcot1
        self.msgInterface.GetPy2CppStruct().new_mcot2 = new_mcot2
        self.msgInterface.PySendEnd()

        dummyAction =([
                     new_mcot1,
                     new_mcot2])
        
        return dummyAction
    
    def get_obs(self, dAction):
        print("Debugging CentralEnv get_obs()")
        
        self.msgInterface.PyRecvBegin()
        if self.msgInterface.PyGetFinished():
            print("Simulation ended")
            self.done = True
        
        mcot1 = self.msgInterface.GetCpp2PyStruct().mcot1
        mcot2 = self.msgInterface.GetCpp2PyStruct().mcot2
        throughput1 = self.msgInterface.GetCpp2PyStruct().throughput1
        throughput2 = self.msgInterface.GetCpp2PyStruct().throughput2
        self.msgInterface.PyRecvEnd()

        self.obs[0] = mcot1
        self.obs[1] = mcot2
        self.obs[2] = dAction[0]
        self.obs[3] = dAction[1]
        self.obs[4] = throughput1
        self.obs[5] = throughput2

        return self.obs

    def render(self):

        pass

    def reset(self, seed=None, options=None):
        self.obs = self.observation_space.sample()
        random.seed(seed)

        return self.obs,{} 

    def step(self, action):
        print("Debugging CentralEnv step()")

        # Get action from RLlib agent
        print("CentralEnv action :",action)
        dAction = self.set_new_action(action)
        print("dAction :",action)

        # Get state results from dummyEnv
        observation = self.get_obs(dAction)
        print("Debug self.obs data type at __init__() : ", type(self.obs))

        reward = self.get_reward(observation)

        print ("dDone : ", self.done)

        info = {}
        
        return observation, reward, self.done, False, info
    
    def close(self):
        # environment is not needed anymore, so kill subprocess in a straightforward way
        del self.exp

'''
class CentralDevisionEnv (gym.Env):
    """Custom Environment that follows gym interface."""

    metadata = {"render_modes": ["human"], "render_fps": 30}

    def __init__(self, ns3Settings):
        super().__init__()

        self.exp = Experiment("ns3ai_wifimac_msg", "../../../../../", py_binding, handleFinish=True)
        self.msgInterface = self.exp.run(setting=ns3Settings, show_output=True)
        
        self.reward = 0
        self.done = False
        self.num_ap = 2

        # Defining the action space for all APs
        action_shape = {
            'new_minCw': gym.spaces.Box(low=0, high=63, shape=(self.num_ap,), dtype=np.int32),
            'new_mcot': gym.spaces.Box(low=0, high=10, shape=(self.num_ap,), dtype=np.int32),
            'new_trafficType': gym.spaces.Box(low=0, high=1, shape=(self.num_ap,), dtype=np.int32),
            'new_udpLambda' : gym.spaces.Box(low=0, high=500, shape=(self.num_ap,), dtype=np.int32),
        }
        self.action_space = gym.spaces.Dict(action_shape)
        #Defining the observation space for all APs
        observation_shape = {
            'simTime' : gym.spaces.Box(low=0, high=20000, shape=(1,), dtype=np.int64),
            'nodeId' : gym.spaces.Box(low=0, high=self.num_ap, shape=(self.num_ap,), dtype=np.int32),
            'minCw': gym.spaces.Box(low=0, high=63, shape=(self.num_ap,), dtype=np.int32),
            'mcot': gym.spaces.Box(low=0, high=10, shape=(self.num_ap,), dtype=np.int32),
            'trafficType': gym.spaces.Box(low=0, high=1, shape=(self.num_ap,), dtype=np.int32),
            'udpLambda' : gym.spaces.Box(low=0, high=500, shape=(self.num_ap,), dtype=np.int32),
            'new_minCw': gym.spaces.Box(low=0, high=63, shape=(self.num_ap,), dtype=np.int32),
            'new_mcot': gym.spaces.Box(low=0, high=10, shape=(self.num_ap,), dtype=np.int32),
            'new_trafficType': gym.spaces.Box(low=0, high=1, shape=(self.num_ap,), dtype=np.int32),
            'new_udpLambda' : gym.spaces.Box(low=0, high=10000, shape=(self.num_ap,), dtype=np.int32),
            'throughput': gym.spaces.Box(low=0, high=1000000, shape=(self.num_ap,), dtype=np.int64),
        }
        self.observation_space = gym.spaces.Dict(observation_shape)

        self.obs = self.observation_space.sample()
        print("Debug self.obs data type at __init__() : ", type(self.obs))

    def get_reward(self, dObs):
        print("Debugging CentralEnv get_reward()")
        
        if len(dObs) == 0:
            self.reward = 0
        else :
            self.reward = (self.obs['throughput'][0]+self.obs['throughput'][1])/2
        
        print("with reward : ", self.reward)

        return self.reward
    
    def set_new_action(self, action_dict):

        new_minCw1 = action_dict['new_minCw'][0]
        new_minCw2 = action_dict['new_minCw'][1]
        new_mcot1 = action_dict['new_mcot'][0]
        new_mcot2 = action_dict['new_mcot'][1]
        new_trafficType1 = action_dict['new_trafficType'][0]
        new_trafficType2 = action_dict['new_trafficType'][1]
        new_udpLambda1 = action_dict['new_udpLambda'][0]
        new_udpLambda2 = action_dict['new_udpLambda'][1]

        self.msgInterface.PySendBegin()
        self.msgInterface.GetPy2CppStruct().new_minCw1 = new_minCw1
        self.msgInterface.GetPy2CppStruct().new_minCw2 = new_minCw2
        self.msgInterface.GetPy2CppStruct().new_mcot1 = new_mcot1
        self.msgInterface.GetPy2CppStruct().new_mcot2 = new_mcot2
        self.msgInterface.GetPy2CppStruct().new_trafficType1 = new_trafficType1
        self.msgInterface.GetPy2CppStruct().new_trafficType2 = new_trafficType2
        self.msgInterface.GetPy2CppStruct().new_udpLambda1 = new_udpLambda1
        self.msgInterface.GetPy2CppStruct().new_udpLambda2 = new_udpLambda2
        self.msgInterface.PySendEnd()

        dummyAction =([new_minCw1,
                     new_minCw2,
                     new_mcot1,
                     new_mcot2, 
                     new_trafficType1, 
                     new_trafficType2, 
                     new_udpLambda1,
                     new_udpLambda2])
        
        return dummyAction
    
    def get_obs(self, dAction):
        print("Debugging CentralEnv get_obs()")
        
        self.msgInterface.PyRecvBegin()
        if self.msgInterface.PyGetFinished():
            print("Simulation ended")
            self.done = True
        
        simTime = self.msgInterface.GetCpp2PyStruct().simTime
        minCw1 = self.msgInterface.GetCpp2PyStruct().minCw1
        minCw2 = self.msgInterface.GetCpp2PyStruct().minCw2
        mcot1 = self.msgInterface.GetCpp2PyStruct().mcot1
        mcot2 = self.msgInterface.GetCpp2PyStruct().mcot2
        trafficType1 = self.msgInterface.GetCpp2PyStruct().trafficType1
        trafficType2 = self.msgInterface.GetCpp2PyStruct().trafficType2
        udpLambda1 = self.msgInterface.GetCpp2PyStruct().udpLambda1
        udpLambda2 = self.msgInterface.GetCpp2PyStruct().udpLambda2
        throughput1 = self.msgInterface.GetCpp2PyStruct().throughput1
        throughput2 = self.msgInterface.GetCpp2PyStruct().throughput2
        self.msgInterface.PyRecvEnd()

        self.obs['simTime'] = np.array([simTime]).astype(np.int64)
        self.obs['minCw'] = np.array([minCw1, minCw2]).astype(np.int32)
        self.obs['mcot'] = np.array([mcot1, mcot2]).astype(np.int32)
        self.obs['trafficType'] = np.array([trafficType1, trafficType2]).astype(np.int8)
        self.obs['udpLambda'] = np.array([udpLambda1, udpLambda2]).astype(np.int32)
        self.obs['throughput'] = np.array([throughput1, throughput2]).astype(np.int64)

        print("Data type of dAction", type(dAction))
        self.obs['new_minCw'] = np.array([dAction[0], dAction[1]]).astype(np.int32)
        self.obs['new_mcot'] = np.array([dAction[2], dAction[3]]).astype(np.int32)
        self.obs['new_trafficType'] = np.array([dAction[4], dAction[5]]).astype(np.int32)
        self.obs['new_udpLambda'] = np.array([dAction[6], dAction[7]]).astype(np.int32)

        return self.obs

    def render(self):

        pass

    def reset(self, seed=None, options=None):
        self.obs = self.observation_space.sample()
        random.seed(seed)

        return self.obs,{} 

    def step(self, action):
        print("Debugging CentralEnv step()")
        # Get action from RLlib agent
        print("CentralEnv action :",action)
        dAction = self.set_new_action(action)
        print("dAction :",action)

        # Get state results from dummyEnv
        observation = self.get_obs(dAction)
        print("Debug self.obs data type at __init__() : ", type(self.obs))

        reward = self.get_reward(observation)

        print ("dDone : ", self.done)

        info = {}
        
        return observation, reward, self.done, False, info
    
    def close(self):
        # environment is not needed anymore, so kill subprocess in a straightforward way
        del self.exp
'''

parser = argparse.ArgumentParser()
parser.add_argument('--seed', type=int,
                    help='set seed for reproducibility')
parser.add_argument('--sim_seed', type=int,
                    help='set simulation run number')
parser.add_argument('--duration', type=float,
                    help='set simulation duration (seconds)')
parser.add_argument('--show_log', action='store_true',
                    help='whether show observation and action')
parser.add_argument('--result', action='store_true',
                    help='whether output figures')
parser.add_argument('--output_dir', type=str,
                    default='./result', help='output figures path')
parser.add_argument('--use_rl', action='store_true',
                    help='whether use rl algorithm')

args = parser.parse_args()
my_seed = 3
if args.seed:
    my_seed = args.seed
print("Python side random seed {}".format(my_seed))
np.random.seed(my_seed)

my_sim_seed = 3
if args.sim_seed:
    my_sim_seed = args.sim_seed

my_duration = 1000
if args.duration:
    my_duration = args.duration

ray.init()

ns3Settings = {
    'simTime': my_duration
}

def env_creator(env_config):
    return CentralDevisionEnv(ns3Settings)

try:
    register_env("my_env", env_creator)
    
    config = PPOConfig()

    algo = config.build(env="my_env")

    while True:
        result = algo.train()
        print(pretty_print(result))

except Exception as e:
    exc_type, exc_value, exc_traceback = sys.exc_info()
    print("Exception occurred: {}".format(e))
    print("Traceback:")
    traceback.print_tb(exc_traceback)
    exit(1)

finally:                                  
    print("Finally exiting...")