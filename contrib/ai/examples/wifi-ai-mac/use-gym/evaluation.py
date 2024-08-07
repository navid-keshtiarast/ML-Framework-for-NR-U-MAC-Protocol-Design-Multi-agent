import os
import argparse
os.environ['RAY_PICKLE_VERBOSE_DEBUG'] = '1'
os.environ["RAY_DEDUP_LOGS"] = "0"

import ray
from ray import air
from ray import tune
from ray.util import inspect_serializability
from ray.rllib.env import BaseEnv
from ray.rllib.env.env_context import EnvContext
from ray.rllib.algorithms import ppo 
from ray.rllib.utils.framework import try_import_tf, try_import_torch
from ray.rllib.utils.test_utils import check_learning_achieved
from ray.tune.logger import pretty_print
from ray.tune.registry import get_trainable_cls
from ray.rllib.algorithms.ppo import PPOConfig
from ray.rllib.algorithms.ppo import PPO
from ray.tune.registry import register_env
from ray.rllib.evaluation import Episode, RolloutWorker

from ray.rllib.algorithms.algorithm import Algorithm
from ray.rllib.algorithms.callbacks import DefaultCallbacks
from ray.rllib.policy.policy import Policy

from typing import Dict, Tuple
import gymnasium as gym
from dataclasses import dataclass
import sys
import random

from environments import CentralDecisionWifiEnv, MultiAgentDecisionWifiEnv

import numpy as np
import matplotlib.pyplot as plt
import traceback

parser = argparse.ArgumentParser()
parser.add_argument('--seed',
                    type=int, help='set seed for reproducibility')
parser.add_argument('--sim_seed', 
                    type=int, help='set simulation run number')
parser.add_argument('--custom_episode',
                     action='store_true', help="Use custom traffic parameter to start each episode.")
parser.add_argument('--ap_nums', 
                    type=int, help='set number of APs')
parser.add_argument('--min_num_aps', 
                    type=int, default=1, help='set minimum number of APs')
parser.add_argument('--max_num_aps', 
                    type=int, default=4, help='set maximum number of APs')                      
parser.add_argument('--alpha', 
                    type=int, help='alpha argument for airtime in reward')
parser.add_argument('--beta', 
                    type=int, help='beta argument for delay in reward')
parser.add_argument('--duration',
                    type=float, help='set simulation duration (seconds)')
parser.add_argument('--udpLambda', 
                    type=int, help='set custom udpLambda for every episode')
parser.add_argument('--udpLambda1', 
                    type=int, help='set custom udpLambda for every episode')
parser.add_argument('--udpLambda2', 
                    type=int, help='set custom udpLambda for every episode')
parser.add_argument('--udpLambda3', 
                    type=int, help='set custom udpLambda for every episode')
parser.add_argument('--udpLambda4', 
                    type=int, help='set custom udpLambda for every episode')                                                                                
parser.add_argument('--packetSize', 
                    type=int, help='set custom packetSize for every episode')
parser.add_argument('--trafficType', 
                    type=str, choices={'UDP_CBR', 'BURST'}, default='UDP_CBR', help='set custom traffic type for every episode')
parser.add_argument('--no_tune', 
                    action='store_true', help='Init Ray in local mode for easier debugging.',)
parser.add_argument("--stop_iters",
                    type=int, default=1, help="Number of iterations to train."
                    )
parser.add_argument("--stop_timesteps",
                    type=int, default=5000, help="Number of timesteps to train."
                    )
parser.add_argument("--reward_type",
                    type=str, choices={'option1', 'option2', 'option3'}, default='option3', help="Variable to use as reward."
                    )
parser.add_argument('--lstm',
                     action='store_true', help="Whether or not to use an LSTM cell")
parser.add_argument('--eps_length',
                     type=int, default=140, help="Number of time steps per episode")
parser.add_argument('--agent',
                     type=str, choices={'central', 'multi'}, default='multi', help="Whether to use a central agent, or multi agents")
parser.add_argument('--separate_agent_nns',
                     action='store_true', help="Only relevant for multi-agent RL. Use separate NNs for each agent instead of sharing.")
parser.add_argument("--framework",
                     choices=["tf", "tf2", "torch"], default="torch",help="The DL framework specifier.")
parser.add_argument("--local-mode",
                     action="store_true", help="Init Ray in local mode for easier debugging.")
parser.add_argument('--outputDir',
                     type=str, help="Output directory")
parser.add_argument('--rllibDir',
                     type=str, help="ML model directory")

args = parser.parse_args()

my_seed = 1
if args.seed:
    my_seed = args.seed
print("Python side random seed {}".format(my_seed))
np.random.seed(my_seed)

my_sim_seed = np.random.randint(1,1000)
if args.sim_seed:
    my_sim_seed = args.sim_seed
print("Python sim_round ", my_sim_seed)

my_sim_seed_set = np.random.randint(1,1000,size=20)
print("Python sim_round_set ", my_sim_seed_set)

my_duration = 10
if args.duration:
    my_duration = args.duration

my_ap_nums = 1#np.random.randint(low=1,high=7) 
if args.ap_nums:
    my_min_num_aps = args.min_num_aps
if args.ap_nums:
    my_max_num_aps = args.max_num_aps
if args.ap_nums:
    my_ap_nums = args.ap_nums

my_alpha = 0.3
if args.alpha:
    my_alpha = args.alpha

my_beta = 0.0
if args.beta:
    my_beta = args.beta

max_num_aps = 4

my_trafficType = 'UDP_CBR'#np.random.choice(['UDP_CBR', 'BURST'])
if args.trafficType:
    my_trafficType = args.trafficType

my_packetSize = 1500
my_fragmentSize = 1500
my_udpLambda =  np.random.randint(low=1, high=500, size=4)*40

# For customEpisode
my_udpLambda[0] =  2290
if args.udpLambda1:
    my_udpLambda[0] = args.udpLambda1
my_udpLambda[1] =  1710
if args.udpLambda2:
    my_udpLambda[1] = args.udpLambda2
my_udpLambda[2] =  2340
if args.udpLambda3:
    my_udpLambda[2] = args.udpLambda3
my_udpLambda[3] =  1250
if args.udpLambda4:
    my_udpLambda[3] = args.udpLambda4

my_env_number = ''

my_custom_episode = True
# if args.custom_episode :
#     if args.ap_nums :
#         my_ap_nums = args.ap_nums
#     if args.trafficType :
#         my_trafficType = args.trafficType
#     if args.packetSize :
#         my_packetSize = args.packetSize
#     if args.udpLambda :
#         my_udpLambda = []
#         for i in range (max_num_aps) :
#             my_udpLambda[i] = args.udpLambda

my_reward_type = 'option3'
if args.reward_type :
    my_reward_type = args.reward_type

my_agent = 'multi'
if args.agent :
    my_agent = args.agent

my_seperate_agent_nns = False
if args.separate_agent_nns :
    my_seperate_agent_nns = args.separate_agent_nns

my_output_dir = ""
if args.outputDir :
    my_output_dir = args.outputDir + '/rllib/' + str(my_ap_nums) + '/'

ray.shutdown()

ray.init()

ns3Settings = {
    'envNumber' : my_env_number,
    'simTime': my_duration,
    'simRound' : my_sim_seed,
    'customEpisode' : my_custom_episode,
    'numAgents' : my_ap_nums,
    'trafficType' : my_trafficType,
    'packetSize' : my_packetSize,
    'fragmentSize' : my_fragmentSize,
    'udpLambda1' : my_udpLambda[0],
    'udpLambda2' : my_udpLambda[1],
    'udpLambda3' : my_udpLambda[2],
    'udpLambda4' : my_udpLambda[3],
    'isEvaluation' : True,
    'outputDirectory' : args.outputDir + '/ns3/' + str(my_ap_nums) + '/',
    }

sim_config = {
    'seed' : my_seed,
    'num_ap' : my_ap_nums,
    'reward_type' : my_reward_type,
    'isEvaluation' : True,
    'new_udpLambda' : my_udpLambda,
    'output_dir' : my_output_dir,
    'alpha' : my_alpha,
    'beta' : my_beta,
    'min_num_aps' : my_min_num_aps,
    'max_num_aps' : my_max_num_aps,
    'simRoundSet' : my_sim_seed_set,
}

def env_creator(env_config):
    if my_agent == 'central' :
        print("Running centralized environment")
        return CentralDecisionWifiEnv(ns3Settings, sim_config)
    elif my_agent == 'multi' :
        print("Running multi-agent environment")
        return MultiAgentDecisionWifiEnv(ns3Settings, sim_config)


register_env("my_env", env_creator)
            
# Step 1 : Load checkpoint

rllib_dir = ""
analysis = tune.ExperimentAnalysis(experiment_checkpoint_path=rllib_dir)
checkpoint_dir = analysis.get_best_checkpoint(analysis.trials[0], "episode_reward_mean", "max")
print('Best checkpoint is ', checkpoint_dir)

# Step 2 : Loading policies
if my_agent == 'central' :
    # For centralized execution
    policy_checkpoint = checkpoint_dir.path + '/policies/default_policy'

    eval_policy = Policy.from_checkpoint(policy_checkpoint)
    eval_policy_weights = eval_policy.get_weights()

    class RestoreWeightsCallback(DefaultCallbacks):
        def on_algorithm_init(self, *, algorithm: "Algorithm", **kwargs) -> None:
            algorithm.set_weights({"default_policy": eval_policy_weights})
    
    config = (PPOConfig()
    .training(gamma=0.9, lr=1e-3, train_batch_size = 5000)#, sgd_minibatch_size = 200)
    .evaluation(evaluation_num_workers=1, evaluation_interval=1,evaluation_duration=10, evaluation_duration_unit="episodes", evaluation_parallel_to_training=False)
    .environment(env="my_env")
    .rollouts(num_rollout_workers=0, num_envs_per_worker=1, remote_worker_envs= False, ignore_worker_failures=True)
    .resources(num_gpus=0, num_cpus_per_worker=2, num_gpus_per_worker=0)
    .debugging(log_level='DEBUG')
    .framework(args.framework)
    .fault_tolerance(recreate_failed_workers=True, restart_failed_sub_environments=True)
    .callbacks(RestoreWeightsCallback)
    )

elif my_agent == 'multi' :
    if my_seperate_agent_nns :
        policy_checkpoint = []
        for i in range (max_num_aps) :
            policy_checkpoint.append(checkpoint_dir.path +'/policies/'+str(i))          
        
        eval_policy = []
        eval_policy_weights = []
        for i in range (max_num_aps) :
            eval_policy.append(Policy.from_checkpoint(policy_checkpoint[i]))
            eval_policy_weights.append(eval_policy[i].get_weights())
        
        class RestoreWeightsCallback(DefaultCallbacks):
            def on_algorithm_init(self, *, algorithm: "Algorithm", **kwargs) -> None:
                algorithm.set_weights(
                    {"0": eval_policy_weights[0],
                     "1": eval_policy_weights[1],
                     "2": eval_policy_weights[2],
                     "3": eval_policy_weights[3],}
                    )
        
        ap_ids = [str(i) for i in range(max_num_aps)]
        config = (PPOConfig()
        .training(gamma=0.9, lr=1e-3, train_batch_size = 5000)#, sgd_minibatch_size = 200)
        .evaluation(evaluation_num_workers=1, evaluation_interval=1,evaluation_duration=10, evaluation_duration_unit="episodes", evaluation_parallel_to_training=False)
        .environment(env="my_env")
        .debugging(log_level='DEBUG')
        .framework(args.framework)
        .resources(num_gpus=0, num_cpus_per_worker=2, num_gpus_per_worker=0)
        .rollouts(num_rollout_workers=0, num_envs_per_worker=1, remote_worker_envs= False, ignore_worker_failures=True)
        .multi_agent(
            policies = {'0', '1', '2', '3'},
            policy_mapping_fn = (lambda agent_id, episode, worker, **kw: str(agent_id)),
            )
        .fault_tolerance(recreate_failed_workers=True, restart_failed_sub_environments=True)
        .callbacks(RestoreWeightsCallback)
        )

    else :
        # For centralized execution
        policy_checkpoint = checkpoint_dir.path + '/policies/ap'

        eval_policy = Policy.from_checkpoint(policy_checkpoint)
        eval_policy_weights = eval_policy.get_weights()

        class RestoreWeightsCallback(DefaultCallbacks):
            def on_algorithm_init(self, *, algorithm: "Algorithm", **kwargs) -> None:
                algorithm.set_weights({"ap": eval_policy_weights})

        config = (PPOConfig()
        .training(gamma=0.9, lr=1e-3, train_batch_size = 5000)#, sgd_minibatch_size = 200)
        .evaluation(evaluation_num_workers=1, evaluation_interval=1,evaluation_duration=10, evaluation_duration_unit="episodes", evaluation_parallel_to_training=False)
        .environment(env="my_env")
        .debugging(log_level='DEBUG')
        .framework(args.framework)
        .resources(num_gpus=0, num_cpus_per_worker=1, num_gpus_per_worker=0)
        .rollouts(num_rollout_workers=0, num_envs_per_worker=1, remote_worker_envs= False, ignore_worker_failures=True)
        .multi_agent(
            policies = {'ap'},
            policy_mapping_fn = (lambda agent_id, episode, worker, **kw: f"ap"),
            )
        .fault_tolerance(recreate_failed_workers=True, restart_failed_sub_environments=True)
        .callbacks(RestoreWeightsCallback)
        )

stop = {
        "training_iteration": args.stop_iters,
        "timesteps_total": args.stop_timesteps,
    }
    
try:
    if args.no_tune :
        algo = config.build(env="my_env")

        print("Running manual training loop without Ray Tune.")
        for _ in range(args.stop_iters) :
            result = algo.train()
            print(pretty_print(result))
            if (result["timesteps_total"] >= args.stop_timesteps) :
                break
        
        algo.stop()
    
    else :
        print("Training automatically with Ray Tune")
        # tuner = tune.Tuner(
        #         "PPO", 
        #         param_space = config.to_dict(),
        #         run_config = air.RunConfig(stop=stop),
        #         )
        # results = tuner.fit()
        results = tune.run(
                "PPO", 
                config=config.to_dict(),
                stop=stop,
                checkpoint_freq=50,
                log_to_file=True
                )

except Exception as e:
    exc_type, exc_value, exc_traceback = sys.exc_info()
    print("Exception occurred: {}".format(e))
    print("Traceback:")
    traceback.print_tb(exc_traceback)
    exit(1)

finally:                                  
    print("Finally exiting...")
    
    ray.shutdown()
        
