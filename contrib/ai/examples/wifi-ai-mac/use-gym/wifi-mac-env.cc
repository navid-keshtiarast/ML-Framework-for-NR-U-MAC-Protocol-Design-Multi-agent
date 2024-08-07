/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2018 Technische Universität Berlin
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Piotr Gawlowicz <gawlowicz@tkn.tu-berlin.de>
 * Modify: Pengyu Liu <eic_lpy@hust.edu.cn> 
 *         Hao Yin <haoyin@uw.edu>
 */

#include <numeric>
#include "wifi-mac-env.h"

namespace ns3
{
NS_LOG_COMPONENT_DEFINE("ns3::WifiMacEnv");

WifiMacEnv::WifiMacEnv()
{
  NS_LOG_FUNCTION(this);
  SetOpenGymInterface(OpenGymInterface::Get());
}

WifiMacEnv::~WifiMacEnv()
{
  NS_LOG_FUNCTION(this);
}

Ptr<OpenGymSpace>
WifiMacEnv::GetActionSpace()
{
  // std::cout << "Debug GetActionSpace()" << std::endl;
  
  // Give the structure of the action space to python side
  // new_mcot1 - new_mcot6
  // new_txPower1 - new_txPower6
  // new_mcs1 - new_mcs6
  // new_edThreshold1 - new_edThreshold6
  // new_backoffType1 -ew_backoffType6
  // new_minCw1 - new_minCw6
  // new_slotTime1 - new_slotTime6

  uint32_t parameterNum = 24;
  float low = -10000.0;
  float high = 1000000.0;
  std::vector<uint32_t> shape = {
        parameterNum,
    };
  std::string dtype = TypeNameGet<int32_t>();

  Ptr<OpenGymBoxSpace> actionBox = CreateObject<OpenGymBoxSpace>(low, high, shape, dtype);

  NS_LOG_INFO("MyGetActionSpace: " << actionBox);
  // std::cout << "MyGetActionSpace: " << actionBox << std::endl;

  return actionBox;
}

bool
WifiMacEnv::GetGameOver()
{
  // std::cout << "Debug GetGameOver()" << std::endl;

  return false;
}

float
WifiMacEnv::GetReward()
{
  // std::cout << "Debug GetReward()" << std::endl;

  return 0.0;
}

std::string
WifiMacEnv::GetExtraInfo()
{
  // std::cout << "Debug GetExtraInfo()" << std::endl;
  
  return "";
}

bool
WifiMacEnv::ExecuteActions(Ptr<OpenGymDataContainer> action)
{
  // std::cout << "Debug ExecuteActions()" << std::endl;
  //Updates the new action from python side
  Ptr<OpenGymBoxContainer<int32_t>> actionBox = DynamicCast<OpenGymBoxContainer<int32_t>>(action);
  
  // Give the structure of the action space to python side
  // new_mcot1 - new_mcot6
  // new_txPower1 - new_txPower6
  // new_mcs1 - new_mcs6
  // new_edThreshold1 - new_edThreshold6
  // new_backoffType1 -ew_backoffType6
  // new_minCw1 - new_minCw6
  // new_slotTime1 - new_slotTime6

  for (uint32_t i = 0; i < 4; i++)
  {
    m_new_ampdu[i] = actionBox ->GetValue (i);
    m_new_txPower[i] = actionBox ->GetValue (1*4+i);
    m_new_mcs[i] = actionBox ->GetValue (2*4+i);
    m_new_edThreshold[i] = actionBox ->GetValue (3*4+i);
    m_new_minCw[i] = actionBox ->GetValue (4*4+i);
    m_new_aifsn[i] = actionBox ->GetValue (5*4+i);
  }  
  
  NS_LOG_INFO("MyExecuteActions: " << action);
  // std::cout << "MyExecuteActions: " << action << std::endl;

  return true;
}

WifiMacTimeStepEnv::WifiMacTimeStepEnv (uint32_t num_ap, uint32_t trafficType, std::vector<uint32_t> udpLambda, uint32_t packetSize)
  : WifiMacEnv()
{
  NS_LOG_FUNCTION(this);
  m_envReward = 0.0;

  m_new_ampdu.resize(4);
  m_new_txPower.resize(4);
  m_new_mcs.resize(4);
  m_new_edThreshold.resize(4);
  m_new_backoffType.resize(4);
  m_new_minCw.resize(4);
  m_new_slotTime.resize(4);
  m_new_aifsn.resize(4);
  m_new_trafficType.resize(4);
  m_new_packetSize.resize(4);
  m_new_udpLambda.resize(4);

  m_ampdu.resize(4);
  m_txPower.resize(4);
  m_mcs.resize(4);
  m_edThreshold.resize(4);
  m_backoffType.resize(4);
  m_minCw.resize(4);
  m_slotTime.resize(4);
  m_aifsn.resize(4);
  
  m_trafficType.resize(4);
  m_packetSize.resize(4);
  m_udpLambda.resize(4);
  
  m_rxPower.resize(4, std::vector<double>(4));
  m_throughput.resize(4);
  m_airTime.resize(4);
  m_delay.resize(4);

  for (uint32_t i = 0; i < 4; i++)
  {
    for (uint32_t j = 0; j < 4; j++)
    {
      m_rxPower[i][j] = -200;
    }
  }

  for (uint32_t i = 0; i < num_ap; i++)
  {
    m_edThreshold[i] = -62;
    m_trafficType[i] = trafficType;
    m_packetSize[i] = packetSize;
    m_udpLambda[i] = udpLambda[i];
    m_aifsn[i] = 2;
    m_slotTime[i] = 9;

    m_new_edThreshold[i] = -62;
    m_new_udpLambda[i] = m_udpLambda[i];
    m_new_packetSize[i] = m_packetSize[i];
    m_new_aifsn[i] = 2;
    m_new_slotTime[i] = 9;
  }
}

void WifiMacTimeStepEnv::ScheduleNextStateRead()
{
  // std::cout << "Debug ScheduleNextStateRead()" << std::endl;;
  NS_LOG_FUNCTION(this);
  Simulator::Schedule(m_timeStep, &WifiMacTimeStepEnv::ScheduleNextStateRead, this);
  Notify();
}

WifiMacTimeStepEnv::~WifiMacTimeStepEnv()
{
  NS_LOG_FUNCTION(this);
}

Ptr<OpenGymSpace>
WifiMacTimeStepEnv::GetObservationSpace()
{
  // std::cout << "Debug GetObservationSpace()" << std::endl;
  
  // Gives the structure of the observation space to python side, which represents following variables
  // mcot1 - mcot6
  // txPower1 - txPower6
  // mcs1 - mcs6
  // edThreshold1 - edThreshold6
  // backoffType1 - backoffType6
  // minCw1 - minCw6
  // slotTime1 - slotTime6
  // trafficType1 - trafficType6
  // packetSize1 - packetSize6
  // udpLambda1 - udpLambda6 
  // rxPowerAp1Sta1 - rxPowerAp1Sta6
  // rxPowerAp2Sta1 - rxPowerAp2Sta6
  // rxPowerAp3Sta1 - rxPowerAp3Sta6
  // rxPowerAp4Sta1 - rxPowerAp4Sta6
  // rxPowerAp5Sta1 - rxPowerAp5Sta6
  // rxPowerAp6Sta1 - rxPowerAp6Sta6
  // throughput1 - throughput6
  // airTime1 - airTime6
  // delay1 - delay6
  // deferTime1- deferTime6

  uint32_t parameterNum = 64;
  float low = -10000.0;
  float high = 1000000.0;
  std::vector<uint32_t> shape = {
      parameterNum,
  };
  std::string dtype = TypeNameGet<int32_t>();

  Ptr<OpenGymBoxSpace> observationBox = CreateObject<OpenGymBoxSpace>(low, high, shape, dtype);
  
  NS_LOG_INFO("MyGetObservationSpace: " << observationBox);
  // std::cout << "MyGetObservationSpace: " << observationBox << std::endl;

  return observationBox;
}

Ptr<OpenGymDataContainer>
WifiMacTimeStepEnv::GetObservation()
{
  // std::cout << "Debug GetObservation()" << std::endl;
  //Get the current observation in the ns3-environment

  uint32_t parameterNum = 64;
  std::vector<uint32_t> shape = {
      parameterNum,
  };

  Ptr<OpenGymBoxContainer<int32_t>> observationBox = CreateObject<OpenGymBoxContainer<int32_t>>(shape);

  for (uint32_t i = 0; i < 4; i++)
  {
    observationBox->AddValue(m_ampdu[i]);
  }
  for (uint32_t i = 0; i < 4; i++)
  {
    observationBox->AddValue(m_txPower[i]);
  }
  for (uint32_t i = 0; i < 4; i++)
  {
    observationBox->AddValue(m_mcs[i]);
  }
  for (uint32_t i = 0; i < 4; i++)
  {
    observationBox->AddValue(m_edThreshold[i]);
  }
  for (uint32_t i = 0; i < 4; i++)
  {
    observationBox->AddValue(m_minCw[i]);
  }
  for (uint32_t i = 0; i < 4; i++)
  {
    observationBox->AddValue(m_aifsn[i]);
  }
  for (uint32_t i = 0; i < 4; i++)
  {
    observationBox->AddValue(m_trafficType[i]);
  }
  for (uint32_t i = 0; i < 4; i++)
  {
    observationBox->AddValue(m_udpLambda[i]);
  }
  for (uint32_t i = 0; i < 4; i++)
  {
    observationBox->AddValue(m_packetSize[i]);
  }
  for (uint32_t i = 0; i < 4; i++)
  {
    for (uint32_t j = 0; j < 4; j++)
    {
      observationBox->AddValue(m_rxPower[i][j]);
    }
  }
  for (uint32_t i = 0; i < 4; i++)
  {
    observationBox->AddValue(m_throughput[i]);
  }
  for (uint32_t i = 0; i < 4; i++)
  {
    observationBox->AddValue(m_delay[i]);
  }
  for (uint32_t i = 0; i < 4; i++)
  {
    observationBox->AddValue(m_airTime[i]);
  }

  // Print data
  NS_LOG_INFO("MyGetObservation: " << observationBox);
  // std::cout << "MyGetObservation: " << observationBox << std::endl;

  return observationBox;
}

void
WifiMacTimeStepEnv::GiveThroughput(double throughput, uint16_t ap_num)
{
  // std::cout << "Debug ns3env GiveThroughput()" << std::endl;
  // std::cout << "  with throughput : " << throughput << " and ap_num : "  << ap_num << std::endl;
  //Update the throughput from after each timestep
  m_throughput[ap_num] = throughput;
}

void 
WifiMacTimeStepEnv::GiveDelay (double delay, uint16_t ap_num)
{
  // std::cout << "Debug ns3env GiveDelay()" << std::endl;
  // std::cout << "  with delay : " << delay << " and ap_num : "  << ap_num << std::endl;
  //Update the throughput from after each timestep
  m_delay[ap_num] = delay;
}

void 
WifiMacTimeStepEnv::GiveAirTime (double airtime, uint16_t ap_num)
{
  // std::cout << "Debug ns3env GiveAirTime()" << std::endl;
  // std::cout << "  with airtime : " << airtime << " and ap_num : "  << ap_num << std::endl;
  //Update the throughput from after each timestep
  m_airTime[ap_num] = airtime;
}

void 
WifiMacTimeStepEnv::GiveRxPower (double rxPower, uint16_t ap_num, uint16_t sta_num)
{
  // std::cout << "Debug ns3env GiveRxPower()" << std::endl;
  // std::cout << "  with rxPower : " << rxPower << " and ap_num : "  << ap_num << " and sta_num : " << sta_num << std::endl;
  //Update the throughput from after each timestep
  m_rxPower[ap_num][sta_num] = rxPower;
}


uint32_t
WifiMacTimeStepEnv::ChangeTrafficType(uint32_t trafficType, uint16_t ap_num)
{
  //Change the trafficType of the gNbs at each timestep (Burst or UDP)
  // std::cout << "Debug ChangeTrafficType()" << std::endl;
  m_trafficType[ap_num] = trafficType;
  return m_new_trafficType[ap_num];
}

uint32_t
WifiMacTimeStepEnv::ChangeMinCw(uint32_t minCw, uint16_t ap_num)
{
  //Change the minCw of the gNbs at each timestep
  // std::cout << "Debug ChangeMinCw()" << std::endl;
  // std::cout << "  with minCw : " << minCw << " and ap_num : " << ap_num << std::endl;
  m_minCw[ap_num] = minCw;
  return m_new_minCw[ap_num]; 
}

double 
WifiMacTimeStepEnv::ChangeTxPower (double txPower, uint16_t ap_num)
{
  // std::cout << "Debug ChangeTxPower()" << std::endl;
  // std::cout << "  with txPower : " << txPower << " and ap_num : " << ap_num << std::endl;
  m_txPower[ap_num] = txPower;
  return m_new_txPower[ap_num];
}

uint8_t 
WifiMacTimeStepEnv::ChangeMcs (uint8_t mcs, uint16_t ap_num)
{
  // std::cout << "Debug ChangeMcs()" << std::endl;
  // std::cout << "  with mcs : " << mcs << " and ap_num : " << ap_num << std::endl;
  m_mcs[ap_num] = mcs;
  return m_new_mcs[ap_num];
}

double 
WifiMacTimeStepEnv::ChangeEDThreshold (double edThreshold, uint16_t ap_num)
{
  // std::cout << "Debug ChangeEDThreshold()" << std::endl;
  // std::cout << "  with edThreshold : " << edThreshold << " and ap_num : " << ap_num << std::endl;
  m_edThreshold[ap_num] = edThreshold;
  return m_new_edThreshold[ap_num];
}

uint32_t 
WifiMacTimeStepEnv::ChangeBackoffType (uint32_t backoffType, uint16_t ap_num)
{
  // std::cout << "Debug ChangeBackoffType()" << std::endl;
  // std::cout << "  with backoffType : " << backoffType << " and ap_num : " << ap_num << std::endl;
  m_backoffType[ap_num] = backoffType;
  return m_new_backoffType[ap_num];
}

double 
WifiMacTimeStepEnv::ChangeSlotTime (double slotTime, uint16_t ap_num)
{
  // std::cout << "Debug ChangeSlotTime()" << std::endl;
  // std::cout << "  with slotTime : " << slotTime << " and ap_num : " << ap_num << std::endl;
  m_slotTime[ap_num] = slotTime;
  return m_new_slotTime[ap_num];
}

double 
WifiMacTimeStepEnv::ChangeDeferTime (double deferTime, uint16_t ap_num)
{
  // std::cout << "Debug ChangeDeferTime()" << std::endl;
  // std::cout << "  with deferTime : " << deferTime << " and ap_num : " << ap_num << std::endl;
  m_deferTime[ap_num] = deferTime;
  return m_new_deferTime[ap_num];
}

uint32_t
WifiMacTimeStepEnv::ChangeMcot(uint32_t mcot, uint16_t ap_num)
{
  // std::cout << "Debug ChangeMcot()" << std::endl;
  //Change the Mcot of the gNbs at each timestep
  m_mcot[ap_num] = mcot;
  return m_new_mcot[ap_num];
}

uint32_t 
WifiMacTimeStepEnv::ChangeAifsn (uint32_t aifsn, uint16_t ap_num)
{
  // std::cout << "Debug ChangeDeferTime()" << std::endl;
  // std::cout << "  with deferTime : " << deferTime << " and ap_num : " << ap_num << std::endl;
  m_aifsn[ap_num] = aifsn;
  return m_new_aifsn[ap_num];
}

uint32_t
WifiMacTimeStepEnv::ChangeAmpdu(uint32_t ampdu, uint16_t ap_num)
{
  // std::cout << "Debug ChangeMcot()" << std::endl;
  //Change the Mcot of the gNbs at each timestep
  m_ampdu[ap_num] = ampdu;
  return m_new_ampdu[ap_num];
}

uint32_t
WifiMacTimeStepEnv::ChangePacketSize(uint32_t packetSize, uint16_t ap_num)
{
  // std::cout << "Debug ChangePacketSize()" << std::endl;
  //Change the packetSize of the gNbs at each timestep
  m_packetSize[ap_num] = packetSize;
  return m_new_packetSize[ap_num];
}

uint32_t
WifiMacTimeStepEnv::ChangeUdpLambda(uint32_t udpLambda, uint16_t ap_num)
{
  // std::cout << "Debug ChangeUdpLambda()" << std::endl;
  //Change the udpLambda of the gNbs at each timestep for trafficType UDP
  m_udpLambda[ap_num] = udpLambda;
  return m_new_udpLambda[ap_num];
}

} // namespace ns3
