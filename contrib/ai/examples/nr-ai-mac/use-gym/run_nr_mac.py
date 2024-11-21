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

from environments import CentralDecisionEnv, MultiAgentDecisionEnv

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
                    type=int, default=6, help='set maximum number of APs')                    
parser.add_argument('--alpha', 
                    type=int, help='alpha argument for airtime in reward')
parser.add_argument('--beta', 
                    type=int, help='beta argument for delay in reward')
parser.add_argument('--duration',
                    type=float, help='set simulation duration (seconds)')
parser.add_argument('--udpLambda', 
                    type=int, help='set custom udpLambda for every episode')
parser.add_argument('--packetSize', 
                    type=int, help='set custom packetSize for every episode')
parser.add_argument('--trafficType', 
                    type=str, choices={'UDP_CBR', 'BURST'}, default='UDP_CBR', help='set custom traffic type for every episode')
parser.add_argument('--no_tune', 
                    action='store_true', help='Init Ray in local mode for easier debugging.',)
parser.add_argument("--stop_iters",
                    type=int, default=15000, help="Number of iterations to train."
                    )
parser.add_argument("--stop_timesteps",
                    type=int, default=15000000, help="Number of timesteps to train."
                    )
parser.add_argument("--reward_type",
                    type=str, choices={'option1', 'option2', 'option3'}, default='option3', help="Variable to use as reward."
                    )
parser.add_argument('--lstm',
                     action='store_true', help="Whether or not to use an LSTM cell")
parser.add_argument('--max_seq_len',
                     type=int, default=10, help="The length of memory in an LSTM cell")
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
parser.add_argument("--isEvaluation",
                     action="store_true", help="This mode is used for evaluation")
parser.add_argument('--entropy_coeff',
                     type=float, default=0.0, help="The rate of exploration entropy coefficient")


args = parser.parse_args()

my_seed = 1
if args.seed:
    my_seed = args.seed
print("Python side random seed {}".format(my_seed))
np.random.seed(my_seed)

my_sim_seed = np.random.randint(1,1000)
if args.sim_seed:
    my_sim_seed = args.sim_seed

my_sim_seed_set = np.random.randint(1,1000,size=20)

my_duration = 50
if args.duration:
    my_duration = args.duration

my_ap_nums = np.random.randint(low=1,high=7) 
if args.min_num_aps :
        my_min_num_aps = args.min_num_aps
if args.max_num_aps :
        my_max_num_aps = args.max_num_aps
if args.min_num_aps or args.max_num_aps :
	my_ap_nums = np.random.randint(low=my_min_num_aps,high=my_max_num_aps+1)

my_alpha = 0.3
if args.alpha:
    my_alpha = args.alpha

my_beta = 0.0
if args.beta:
    my_beta = args.beta
    
max_num_aps = 6

my_trafficType = np.random.choice(['UDP_CBR', 'BURST'])
my_packetSize = 1500
my_fragmentSize = 1500
my_udpLambda =  np.random.randint(low=1, high=300, size=6)*10

my_env_number = ''

my_custom_episode = False
if args.custom_episode :
    if args.ap_nums :
        my_ap_nums = args.ap_nums
    if args.trafficType :
        my_trafficType = args.trafficType
    if args.packetSize :
        my_packetSize = args.packetSize
    if args.udpLambda :
        my_udpLambda = []
        for i in range (max_num_aps) :
            my_udpLambda[i] = args.udpLambda

my_reward_type = 'option3'
if args.reward_type :
    my_reward_type = args.reward_type

my_agent = 'multi'
if args.agent :
    my_agent = args.agent

my_seperate_agent_nns = False
if args.separate_agent_nns :
    my_seperate_agent_nns = args.separate_agent_nns

my_entropy_coeff = 0.0
if args.entropy_coeff :
    my_entropy_coeff = args.entropy_coeff

my_lstm = False
if args.lstm :
    my_lstm = args.lstm

my_max_seq_len = 10
if args.max_seq_len :
    my_max_seq_len = args.max_seq_len

my_output_dir = ""

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
    'udpLambda5' : my_udpLambda[4],
    'udpLambda6' : my_udpLambda[5],
    'isEvaluation' : False,
    }

sim_config = {
    'seed' : my_seed,
    'num_ap' : my_ap_nums,
    'reward_type' : my_reward_type,
    'isEvaluation' : False,
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
        return CentralDecisionEnv(ns3Settings, sim_config)
    elif my_agent == 'multi' :
        print("Running multi-agent environment")
        return MultiAgentDecisionEnv(ns3Settings, sim_config)

try:
    register_env("my_env", env_creator)

    if my_agent == 'central' :
        class MyCallbacks(DefaultCallbacks):
            def on_episode_start(self, worker: RolloutWorker, base_env: BaseEnv,
                                policies: Dict[str, Policy],
                                episode: Episode, **kwargs):
                episode.user_data["mean_th"] = []
                episode.user_data["reward"] = []

            def on_episode_step(self, worker: RolloutWorker, base_env: BaseEnv,
                                episode: Episode, **kwargs):
                mean_th = episode._last_infos['agent0']['mean_th']
                reward = episode._last_infos['agent0']['reward']
                episode.user_data["mean_th"].append(mean_th)
                episode.user_data["reward"].append(reward)

            def on_episode_end(self, worker: RolloutWorker, base_env: BaseEnv,
                            policies: Dict[str, Policy], episode: Episode,
                            **kwargs):
                mean_ep_th = np.mean(episode.user_data["mean_th"])
                mean_ep_reward = np.mean(episode.user_data["reward"])
                episode.custom_metrics["mean_ep_reward"] = mean_ep_reward
                episode.custom_metrics["mean_ep_th"] = mean_ep_th

            def on_train_result(self, *, algorithm, result: dict, **kwargs):
                if bool(result['env_runners']["custom_metrics"]) : 
                    ep_th = result['env_runners']["custom_metrics"]["mean_ep_th"]
                    ep_reward = result['env_runners']["custom_metrics"]["mean_ep_reward"]

                    mean_ep_reward = np.mean(ep_reward)
                    mean_ep_th = np.mean(ep_th)
                    result["custom_metrics"]["mean_ep_th"] = mean_ep_th
                    result["custom_metrics"]["mean_ep_reward"] = mean_ep_reward

        print("Running centralized environment")
        config = (PPOConfig()
        .training(gamma=0.9, lr=1e-3, train_batch_size = 1000, entropy_coeff = my_entropy_coeff, 
                model={'use_lstm' : my_lstm, 
                       'max_seq_len' : my_max_seq_len,
                      }
                )
        .environment(env="my_env")
        .debugging(log_level='DEBUG')
        .framework(args.framework)
        .resources(num_gpus=2)
        .env_runners(num_env_runners=25, num_envs_per_env_runner=1, num_cpus_per_env_runner=1, num_gpus_per_env_runner=0, remote_worker_envs=False)
        .fault_tolerance(ignore_env_runner_failures=True, recreate_failed_env_runners=True, restart_failed_sub_environments=True)
        .callbacks(MyCallbacks)
        .reporting(keep_per_episode_custom_metrics=True)
        )
    elif my_agent == 'multi' :
        if my_seperate_agent_nns :
            class MyCallbacks(DefaultCallbacks):
                def on_episode_start(self, worker: RolloutWorker, base_env: BaseEnv,
                                    policies: Dict[str, Policy],
                                    episode: Episode, **kwargs):
                    episode.user_data["mean_th"] = []
                    episode.user_data["reward_0"] = []
                    episode.user_data["reward_1"] = []
                    episode.user_data["reward_2"] = []
                    episode.user_data["reward_3"] = []
                    episode.user_data["reward_4"] = []
                    episode.user_data["reward_5"] = []

                def on_episode_step(self, worker: RolloutWorker, base_env: BaseEnv,
                                    episode: Episode, **kwargs):
                    mean_th = episode._last_infos[0]['mean_th']
                    reward_0 = episode._last_infos[0]['reward']
                    reward_1 = episode._last_infos[1]['reward']
                    reward_2 = episode._last_infos[2]['reward']
                    reward_3 = episode._last_infos[3]['reward']
                    reward_4 = episode._last_infos[4]['reward']
                    reward_5 = episode._last_infos[5]['reward']
                    episode.user_data["mean_th"].append(mean_th)
                    episode.user_data["reward_0"].append(reward_0)
                    episode.user_data["reward_1"].append(reward_1)
                    episode.user_data["reward_2"].append(reward_2)
                    episode.user_data["reward_3"].append(reward_3)
                    episode.user_data["reward_4"].append(reward_4)
                    episode.user_data["reward_5"].append(reward_5)

                def on_episode_end(self, worker: RolloutWorker, base_env: BaseEnv,
                                policies: Dict[str, Policy], episode: Episode,
                                **kwargs):
                    mean_ep_th = np.mean(episode.user_data["mean_th"])
                    policy_reward = []
                    policy_reward.append(np.mean(episode.user_data["reward_0"]))
                    policy_reward.append(np.mean(episode.user_data["reward_1"]))
                    policy_reward.append(np.mean(episode.user_data["reward_2"]))
                    policy_reward.append(np.mean(episode.user_data["reward_3"]))
                    policy_reward.append(np.mean(episode.user_data["reward_4"]))
                    policy_reward.append(np.mean(episode.user_data["reward_5"]))
                    mean_ep_reward = 0
                    
                    for i in range(max_num_aps) :
                        mean_ep_reward += policy_reward[i]

                    episode.custom_metrics["policy_reward1"] = policy_reward[0]
                    episode.custom_metrics["policy_reward2"] = policy_reward[1]
                    episode.custom_metrics["policy_reward3"] = policy_reward[2]
                    episode.custom_metrics["policy_reward4"] = policy_reward[3]
                    episode.custom_metrics["policy_reward5"] = policy_reward[4]
                    episode.custom_metrics["policy_reward6"] = policy_reward[5]
                    episode.custom_metrics["mean_ep_reward"] = mean_ep_reward
                    episode.custom_metrics["mean_ep_th"] = mean_ep_th

                def on_train_result(self, *, algorithm, result: dict, **kwargs):
                    if bool(result['env_runners']["custom_metrics"]) : 
                        ep_th = result['env_runners']["custom_metrics"]["mean_ep_th"]
                        ep_reward = result['env_runners']["custom_metrics"]["mean_ep_reward"]

                        mean_ep_reward = np.mean(ep_reward)
                        mean_ep_th = np.mean(ep_th)
                        result["custom_metrics"]["mean_ep_th"] = mean_ep_th
                        result["custom_metrics"]["mean_ep_reward"] = mean_ep_reward
                
            print("Running multi-agent environment with distributed execution")
            ap_ids = [str(i) for i in range(max_num_aps)]
            config = (PPOConfig()
            .training(gamma=0.9, lr=1e-3, train_batch_size = 1000, entropy_coeff = my_entropy_coeff, 
                model={'use_lstm' : my_lstm, 
                       'max_seq_len' : my_max_seq_len,
                      }
                )
            .environment(env="my_env")
            .debugging(log_level='DEBUG')
            .framework(args.framework)
            .resources(num_gpus=2)
            .env_runners(num_env_runners=25, num_envs_per_env_runner=1, num_cpus_per_env_runner=1, num_gpus_per_env_runner=0, remote_worker_envs=False)
            .fault_tolerance(ignore_env_runner_failures=True, recreate_failed_env_runners=True, restart_failed_sub_environments=True)
            .multi_agent(
                policies = {'0', '1', '2', '3', '4', '5'},
                policy_mapping_fn = (lambda agent_id, episode, worker, **kw: str(agent_id)),
                )
            .callbacks(MyCallbacks)
            .reporting(keep_per_episode_custom_metrics=True)
            )
        else :
            class MyCallbacks(DefaultCallbacks):
                def on_episode_start(self, worker: RolloutWorker, base_env: BaseEnv,
                                    policies: Dict[str, Policy],
                                    episode: Episode, **kwargs):
                    episode.user_data["mean_th"] = []
                    episode.user_data["reward_0"] = []
                    episode.user_data["reward_1"] = []
                    episode.user_data["reward_2"] = []
                    episode.user_data["reward_3"] = []
                    episode.user_data["reward_4"] = []
                    episode.user_data["reward_5"] = []

                def on_episode_step(self, worker: RolloutWorker, base_env: BaseEnv,
                                    episode: Episode, **kwargs):
                    mean_th = episode._last_infos[0]['mean_th']
                    reward_0 = episode._last_infos[0]['reward']
                    reward_1 = episode._last_infos[1]['reward']
                    reward_2 = episode._last_infos[2]['reward']
                    reward_3 = episode._last_infos[3]['reward']
                    reward_4 = episode._last_infos[4]['reward']
                    reward_5 = episode._last_infos[5]['reward']
                    episode.user_data["mean_th"].append(mean_th)
                    episode.user_data["reward_0"].append(reward_0)
                    episode.user_data["reward_1"].append(reward_1)
                    episode.user_data["reward_2"].append(reward_2)
                    episode.user_data["reward_3"].append(reward_3)
                    episode.user_data["reward_4"].append(reward_4)
                    episode.user_data["reward_5"].append(reward_5)

                def on_episode_end(self, worker: RolloutWorker, base_env: BaseEnv,
                                policies: Dict[str, Policy], episode: Episode,
                                **kwargs):
                    mean_ep_th = np.mean(episode.user_data["mean_th"])
                    policy_reward = []
                    policy_reward.append(np.mean(episode.user_data["reward_0"]))
                    policy_reward.append(np.mean(episode.user_data["reward_1"]))
                    policy_reward.append(np.mean(episode.user_data["reward_2"]))
                    policy_reward.append(np.mean(episode.user_data["reward_3"]))
                    policy_reward.append(np.mean(episode.user_data["reward_4"]))
                    policy_reward.append(np.mean(episode.user_data["reward_5"]))
                    mean_ep_reward = 0
                    
                    for i in range(max_num_aps) :
                        mean_ep_reward += policy_reward[i]

                    episode.custom_metrics["policy_reward1"] = policy_reward[0]
                    episode.custom_metrics["policy_reward2"] = policy_reward[1]
                    episode.custom_metrics["policy_reward3"] = policy_reward[2]
                    episode.custom_metrics["policy_reward4"] = policy_reward[3]
                    episode.custom_metrics["policy_reward5"] = policy_reward[4]
                    episode.custom_metrics["policy_reward6"] = policy_reward[5]
                    episode.custom_metrics["mean_ep_reward"] = mean_ep_reward
                    episode.custom_metrics["mean_ep_th"] = mean_ep_th

                def on_train_result(self, *, algorithm, result: dict, **kwargs):
                    if bool(result['env_runners']["custom_metrics"]) : 
                        ep_th = result['env_runners']["custom_metrics"]["mean_ep_th"]
                        ep_reward = result['env_runners']["custom_metrics"]["mean_ep_reward"]

                        mean_ep_reward = np.mean(ep_reward)
                        mean_ep_th = np.mean(ep_th)
                        result["custom_metrics"]["mean_ep_th"] = mean_ep_th
                        result["custom_metrics"]["mean_ep_reward"] = mean_ep_reward
                        
            print("Running multi-agent environment with centralized execution")
            config = (PPOConfig()
            .training(gamma=0.9, lr=1e-3, train_batch_size = 1000, entropy_coeff = my_entropy_coeff, 
                model={'use_lstm' : my_lstm, 
                       'max_seq_len' : my_max_seq_len,
                      }
                )
            .environment(env="my_env")
            .debugging(log_level='DEBUG')
            .framework(args.framework)
            .resources(num_gpus=2)
            .env_runners(num_env_runners=25, num_envs_per_env_runner=1, num_cpus_per_env_runner=1, num_gpus_per_env_runner=0, remote_worker_envs=False)
            .fault_tolerance(ignore_env_runner_failures=True, recreate_failed_env_runners=True, restart_failed_sub_environments=True)
            .multi_agent(
                policies = {'ap'},
                policy_mapping_fn = (lambda agent_id, episode, worker, **kw: f'ap'),
                )
            .callbacks(MyCallbacks)
            .reporting(keep_per_episode_custom_metrics=True)
            )
    
    stop = {
        "training_iteration": args.stop_iters,
        "timesteps_total": args.stop_timesteps,
    }

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
