# WIFI-MAC-AI

### Introduction

The following description will explain the workaround of the ns3-ai NR-MAC-AI program : 

1. nr-mac-env.cc

This C++ side of the nvironment provides the skeleton to create a Gymnaisum-compatioble environment at ns3. This in done by inheriting 'OpenGymEnv', as done in class NrMacEnv : public OpenGymEnv.

To change the paraneters for each timestep, some functions need to be implemented : 

a. NrMacTimeStepEnv::ScheduleNextStateRead()

This function updates all information (observation, states and action) and send it to the Python interface through the Notify() function. The `Notify()` function, defined in base class, is the core of C++-Python interaction. It registers essential callbacks,
collects state and send it to Python, receives the action, and executes it.

b. Ptr<OpenGymSpace> NrMacEnv::GetActionSpace() 

Called when initialing Gym interface. It defines the action space of environment.

Ptr<OpenGymSpace>
NrMacEnv::GetActionSpace()
{
  //Give the structure of the action space to python side
  uint32_t parameterNum = 2;
  std::vector<uint32_t> shape = {
        parameterNum,
  };
  std::string dtype1 = TypeNameGet<uint32_t>();
  std::string dtype2 = TypeNameGet<double>();
  Ptr<OpenGymBoxSpace> minCwBox = CreateObject<OpenGymBoxSpace>(0, 63, shape, dtype1);
  Ptr<OpenGymBoxSpace> mcotBox = CreateObject<OpenGymBoxSpace>(0, 10, shape, dtype1);
  Ptr<OpenGymBoxSpace> trafficTypeBox = CreateObject<OpenGymBoxSpace>(0, 1, shape, dtype1);
  Ptr<OpenGymBoxSpace> packetSizeBox = CreateObject<OpenGymBoxSpace>(500, 1500, shape, dtype1);
  Ptr<OpenGymBoxSpace> udpLambdaBox = CreateObject<OpenGymBoxSpace>(0, 10000, shape, dtype1);
  
  Ptr<OpenGymDictSpace> actionDict = CreateObject<OpenGymDictSpace>();
  actionDict->Add("new_minCw", minCwBox);
  actionDict->Add("new_mcot", mcotBox);
  actionDict->Add("new_traffictype", trafficTypeBox);
  actionDict->Add("new_packetSize", packetSizeBox);
  actionDict->Add("new_udpLambda", udpLambdaBox);

  return actionDict;
}

In this implementation, GetActionSpace() defines a dictionary consist of 5 boxes, each box has a length of 2. The boxes save the value the "new_.." parameter for each GNBs.

c. Ptr<OpenGymSpace> NrMacTimeStepEnv::GetObservationSpace() 

Similar to `GetActionSpace`, it defines the observation space of environment.

d. NrMacEnv::GetGameOver()

In Gym interface an environment have two ways to stop: game over or simulation end. Since, we prefer the environment end, when the simulation ends, the return value is always 'false'

e. Ptr<OpenGymDataContainer> NrMacTimeStepEnv::GetObservation()

Function to collect observation (state) from environment. In this implementation, a dictionary of minCw, mcot, trafficType, etc. for both gNbs are collected.

f. float NrMacEnv::GetReward()

Function that define the reward (`float` type). In this implementation, the reward is the average throughput of all gNbs, configured in the NrMacTimeStepEnv::GiveThroughput(double throughput) function.

g. bool NrMacEnv::ExecuteActions(Ptr<OpenGymDataContainer> action)

Function that executes the action according to the information in container. In this function, for each timestep we update the local parameters such as m_new_minCw1, m_new_minCw2 to be executed by nr-ai-mac.cc.

2. nr-ai-mac.cc

The full GymInterface is linked in the program through 

Ptr<OpenGymInterface> g_openGymInterface. The gymEnv is later created though the function CreateEnv(). 

Each timestep (g_timestep) in the simulation, the simulation exchanges information with the environment through a sequence of scheduled callback functions :

Simulator::Schedule (g_timeStep, &GiveThroughputAlt, &flowHelper, monitor, numNruPairs);
Simulator::Schedule (g_timeStep, &PrintProgressAlt);
Simulator::Schedule (g_timeStep, &ChangeMacTypeAlt);
Simulator::Schedule (g_timeStep, &ChangeTrafficType, serverAppsNru, clientAppsNru);
Simulator::Schedule (g_timeStep, &UpdateTrafficType, newGnbNodes);

At the end of the simulation, the interface is destroyed using this command 

g_openGymInterface->NotifySimulationEnd();

3. agents.py

To-be implemented

4. run_nr_mac.py

a. Start by importing essential modules 

import ns3ai_gym_env
import gymnasium as gym

b. Define an agent which interacts with the environment

c. Create the environment and do intial setup

ns3Settings = {
    'simTime': my_duration,
    'simSeed': my_sim_seed}
env = gym.make("ns3ai_gym_env/Ns3-v0", targetName="ns3ai_nrmac_gym",
               ns3Path="../../../../../", ns3Settings=ns3Settings)
ob_space = env.observation_space
ac_space = env.action_space

d. Create a training protocol, which interacts with the ns3 side, through calling the env.step(action)

e. Close the environment

env.close()

### Running the program

This program can be run using the following interfaces:

1. Copy the ai file from /scratch to /contrib 
2. [Setup ns3-ai](../../docs/install.md) Do not clone from git !!!!!
3. Build C++ executable & Python bindings

```shell
cd YOUR_NS3_DIRECTORY
./ns3 build ns3ai_nrmac_gym
```

4. Run Python script

```bash
cd contrib/ai/examples/nr-ai-mac/use-gym
python3 run_nr_mac.py
```

### Possible Errors

1. Commented #include <complex.h> in src/spectrum/matrix-based-channel-model.h, src/three-gpp-channel-model.h, and src/three-gpp-spectrum-propagation-loss-model.h

### TO-DOs

1. Define an agent to interact with the environment
2. Setup training using RLlib
3. Scaling up for multi-agent requires changin the built-in registered environment "ns3ai_gym_env/Ns3-v0" in ai/model/gym-interface/py/ns3ai_gym_env
