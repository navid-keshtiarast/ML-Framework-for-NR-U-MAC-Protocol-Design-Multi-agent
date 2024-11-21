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
#include "nr-mac-env.h"

namespace ns3
{
NS_LOG_COMPONENT_DEFINE("ns3::NrMacEnv");

NrMacEnv::NrMacEnv()
{
  NS_LOG_FUNCTION(this);
  SetOpenGymInterface(OpenGymInterface::Get());
}

NrMacEnv::~NrMacEnv()
{
  NS_LOG_FUNCTION(this);
}

Ptr<OpenGymSpace>
NrMacEnv::GetActionSpace()
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

  uint32_t parameterNum = 48;
  float low = -10000.0;
  float high = 1000000.0;
  std::vector<uint32_t> shape = {
        parameterNum,
    };
  std::string dtype = TypeNameGet<int32_t>();

  Ptr<OpenGymBoxSpace> actionBox = CreateObject<OpenGymBoxSpace>(low, high, shape, dtype);

  NS_LOG_INFO("MyGetActionSpace: " << actionBox);

  return actionBox;
}

bool
NrMacEnv::GetGameOver()
{
  // std::cout << "Debug GetGameOver()" << std::endl;

  return false;
}

float
NrMacEnv::GetReward()
{
  // std::cout << "Debug GetReward()" << std::endl;

  return 0.0;
}

std::string
NrMacEnv::GetExtraInfo()
{
  // std::cout << "Debug GetExtraInfo()" << std::endl;
  
  return "";
}

bool
NrMacEnv::ExecuteActions(Ptr<OpenGymDataContainer> action)
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

  for (uint32_t i = 0; i < 6; i++)
  {
    m_new_mcot[i] = actionBox ->GetValue (i);
    // m_new_ampdu[i] = actionBox ->GetValue (i);
    m_new_txPower[i] = actionBox ->GetValue (1*6+i);
    m_new_mcs[i] = actionBox ->GetValue (2*6+i);
    m_new_edThreshold[i] = actionBox ->GetValue (3*6+i);
    m_new_backoffType[i] = actionBox ->GetValue (4*6+i);
    m_new_minCw[i] = actionBox ->GetValue (5*6+i);
    m_new_slotTime[i] = actionBox ->GetValue (6*6+i);
    m_new_deferTime[i] = actionBox ->GetValue (7*6+i);
    // m_new_aifsn[i] = actionBox ->GetValue (7*6+i);
  }  
  
  NS_LOG_INFO("MyExecuteActions: " << action);
  // std::cout << "MyExecuteActions: " << action << std::endl;

  return true;
}

NrMacTimeStepEnv::NrMacTimeStepEnv (uint32_t num_ap, uint32_t trafficType, std::vector<uint32_t> udpLambda, uint32_t packetSize)
  : NrMacEnv()
{
  NS_LOG_FUNCTION(this);
  m_envReward = 0.0;

  m_new_mcot.resize(6);
  // m_new_ampdu.resize(6);
  m_new_txPower.resize(6);
  m_new_mcs.resize(6);
  m_new_edThreshold.resize(6);
  m_new_backoffType.resize(6);
  m_new_minCw.resize(6);
  m_new_slotTime.resize(6);
  m_new_deferTime.resize(6);
  // m_new_aifsn.resize(6);
  m_new_trafficType.resize(6);
  m_new_packetSize.resize(6);
  m_new_udpLambda.resize(6);

  m_mcot.resize(6);
  // m_ampdu.resize(6);
  m_txPower.resize(6);
  m_mcs.resize(6);
  m_edThreshold.resize(6);
  m_backoffType.resize(6);
  m_minCw.resize(6);
  m_slotTime.resize(6);
  m_deferTime.resize(6);
  // m_aifsn.resize(6);
  
  m_trafficType.resize(6);
  m_packetSize.resize(6);
  m_udpLambda.resize(6);
  
  m_rxPower.resize(6, std::vector<double>(6));
  m_throughput.resize(6);
  m_airTime.resize(6);
  m_delay.resize(6);

  for (uint32_t i = 0; i < 6; i++)
  {
    for (uint32_t j = 0; j < 6; j++)
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

    m_new_edThreshold[i] = -62;
    m_new_udpLambda[i] = m_udpLambda[i];
    m_new_packetSize[i] = m_packetSize[i];

    m_slotTime[i] = 9;
    m_deferTime[i] = 16;
    m_mcot[i] = 5;
    m_backoffType[i] = 3;
    m_minCw[i] = 15;
    m_txPower[i] = 23;
    
    m_new_slotTime[i] = 9;
    m_new_deferTime[i] = 16;
    m_new_mcot[i] = 5;
    m_new_backoffType[i] = 3;
    m_new_minCw[i] = 15;
    m_new_txPower[i] = 23;
  }
}

void NrMacTimeStepEnv::ScheduleNextStateRead()
{
  // std::cout << "Debug ScheduleNextStateRead()" << std::endl;;
  NS_LOG_FUNCTION(this);
  Simulator::Schedule(m_timeStep, &NrMacTimeStepEnv::ScheduleNextStateRead, this);
  Notify();
}

NrMacTimeStepEnv::~NrMacTimeStepEnv()
{
  NS_LOG_FUNCTION(this);
}

Ptr<OpenGymSpace>
NrMacTimeStepEnv::GetObservationSpace()
{
  // std::cout << "Debug GetObservationSpace()" << std::endl;

  uint32_t parameterNum = 120;
  float low = -10000.0;
  float high = 1000000.0;
  std::vector<uint32_t> shape = {
      parameterNum,
  };
  std::string dtype = TypeNameGet<int32_t>();

  Ptr<OpenGymBoxSpace> observationBox = CreateObject<OpenGymBoxSpace>(low, high, shape, dtype);
  
  NS_LOG_INFO("MyGetObservationSpace: " << observationBox);

  return observationBox;
}

Ptr<OpenGymDataContainer>
NrMacTimeStepEnv::GetObservation()
{
  // std::cout << "Debug GetObservation()" << std::endl;
  //Get the current observation in the ns3-environment

  uint32_t parameterNum = 120;
  std::vector<uint32_t> shape = {
      parameterNum,
  };

  Ptr<OpenGymBoxContainer<int32_t>> observationBox = CreateObject<OpenGymBoxContainer<int32_t>>(shape);

  for (uint32_t i = 0; i < 6; i++)
  {
    observationBox->AddValue(m_mcot[i]);
  }
  // for (uint32_t i = 0; i < 6; i++)
  // {
  //   observationBox->AddValue(m_ampdu[i]);
  // }
  for (uint32_t i = 0; i < 6; i++)
  {
    observationBox->AddValue(m_txPower[i]);
  }
  for (uint32_t i = 0; i < 6; i++)
  {
    observationBox->AddValue(m_mcs[i]);
  }
  for (uint32_t i = 0; i < 6; i++)
  {
    observationBox->AddValue(m_edThreshold[i]);
  }
  for (uint32_t i = 0; i < 6; i++)
  {
    observationBox->AddValue(m_backoffType[i]);
  }
  for (uint32_t i = 0; i < 6; i++)
  {
    observationBox->AddValue(m_minCw[i]);
  }
  for (uint32_t i = 0; i < 6; i++)
  {
    observationBox->AddValue(m_slotTime[i]);
  }
  for (uint32_t i = 0; i < 6; i++)
  {
    observationBox->AddValue(m_trafficType[i]);
  }
  for (uint32_t i = 0; i < 6; i++)
  {
    observationBox->AddValue(m_udpLambda[i]);
  }
  for (uint32_t i = 0; i < 6; i++)
  {
    observationBox->AddValue(m_packetSize[i]);
  }
  for (uint32_t i = 0; i < 6; i++)
  {
    for (uint32_t j = 0; j < 6; j++)
    {
      observationBox->AddValue(m_rxPower[i][j]);
    }
  }
  for (uint32_t i = 0; i < 6; i++)
  {
    observationBox->AddValue(m_throughput[i]);
  }
  for (uint32_t i = 0; i < 6; i++)
  {
    observationBox->AddValue(m_delay[i]);
  }
  for (uint32_t i = 0; i < 6; i++)
  {
    observationBox->AddValue(m_airTime[i]);
  }
  for (uint32_t i = 0; i < 6; i++)
  {
    observationBox->AddValue(m_deferTime[i]);
  }
  // for (uint32_t i = 0; i < 6; i++)
  // {
  //   observationBox->AddValue(m_aifsn[i]);
  // }
  

  // Print data
  NS_LOG_INFO("MyGetObservation: " << observationBox);
  // std::cout << "MyGetObservation: " << observationBox << std::endl;

  return observationBox;
}

void
NrMacTimeStepEnv::GiveThroughput(double throughput, uint16_t ap_num)
{
  // std::cout << "Debug ns3env GiveThroughput()" << std::endl;
  // std::cout << "  with throughput : " << throughput << " and ap_num : "  << ap_num << std::endl;
  //Update the throughput from after each timestep
  m_throughput[ap_num] = throughput;
}

void 
NrMacTimeStepEnv::GiveDelay (double delay, uint16_t ap_num)
{
  // std::cout << "Debug ns3env GiveDelay()" << std::endl;
  // std::cout << "  with delay : " << delay << " and ap_num : "  << ap_num << std::endl;
  //Update the throughput from after each timestep
  m_delay[ap_num] = delay;
}

void 
NrMacTimeStepEnv::GiveAirTime (double airtime, uint16_t ap_num)
{
  // std::cout << "Debug ns3env GiveAirTime()" << std::endl;
  // std::cout << "  with airtime : " << airtime << " and ap_num : "  << ap_num << std::endl;
  //Update the throughput from after each timestep
  m_airTime[ap_num] = airtime;
}

void 
NrMacTimeStepEnv::GiveRxPower (double rxPower, uint16_t ap_num, uint16_t sta_num)
{
  // std::cout << "Debug ns3env GiveRxPower()" << std::endl;
  // std::cout << "  with rxPower : " << rxPower << " and ap_num : "  << ap_num << " and sta_num : " << sta_num << std::endl;
  //Update the throughput from after each timestep
  m_rxPower[ap_num][sta_num] = rxPower;
}


uint32_t
NrMacTimeStepEnv::ChangeTrafficType(uint32_t trafficType, uint16_t ap_num)
{
  //Change the trafficType of the gNbs at each timestep (Burst or UDP)
  // std::cout << "Debug ChangeTrafficType()" << std::endl;
  m_trafficType[ap_num] = trafficType;
  return m_new_trafficType[ap_num];
}

uint32_t
NrMacTimeStepEnv::ChangeMinCw(uint32_t minCw, uint16_t ap_num)
{
  //Change the minCw of the gNbs at each timestep
  // std::cout << "Debug ChangeMinCw()" << std::endl;
  // std::cout << "  with minCw : " << minCw << " and ap_num : " << ap_num << std::endl;
  m_minCw[ap_num] = minCw;
  return m_new_minCw[ap_num]; 
}

double 
NrMacTimeStepEnv::ChangeTxPower (double txPower, uint16_t ap_num)
{
  // std::cout << "Debug ChangeTxPower()" << std::endl;
  // std::cout << "  with txPower : " << txPower << " and ap_num : " << ap_num << std::endl;
  m_txPower[ap_num] = txPower;
  return m_new_txPower[ap_num];
}

uint8_t 
NrMacTimeStepEnv::ChangeMcs (uint8_t mcs, uint16_t ap_num)
{
  // std::cout << "Debug ChangeMcs()" << std::endl;
  // std::cout << "  with mcs : " << mcs << " and ap_num : " << ap_num << std::endl;
  m_mcs[ap_num] = mcs;
  return m_new_mcs[ap_num];
}

double 
NrMacTimeStepEnv::ChangeEDThreshold (double edThreshold, uint16_t ap_num)
{
  // std::cout << "Debug ChangeEDThreshold()" << std::endl;
  // std::cout << "  with edThreshold : " << edThreshold << " and ap_num : " << ap_num << std::endl;
  m_edThreshold[ap_num] = edThreshold;
  return m_new_edThreshold[ap_num];
}

uint32_t 
NrMacTimeStepEnv::ChangeBackoffType (uint32_t backoffType, uint16_t ap_num)
{
  // std::cout << "Debug ChangeBackoffType()" << std::endl;
  // std::cout << "  with backoffType : " << backoffType << " and ap_num : " << ap_num << std::endl;
  m_backoffType[ap_num] = backoffType;
  return m_new_backoffType[ap_num];
}

double 
NrMacTimeStepEnv::ChangeSlotTime (double slotTime, uint16_t ap_num)
{
  // std::cout << "Debug ChangeSlotTime()" << std::endl;
  // std::cout << "  with slotTime : " << slotTime << " and ap_num : " << ap_num << std::endl;
  m_slotTime[ap_num] = slotTime;
  return m_new_slotTime[ap_num];
}

double 
NrMacTimeStepEnv::ChangeDeferTime (double deferTime, uint16_t ap_num)
{
  // std::cout << "Debug ChangeDeferTime()" << std::endl;
  // std::cout << "  with deferTime : " << deferTime << " and ap_num : " << ap_num << std::endl;
  m_deferTime[ap_num] = deferTime;
  return m_new_deferTime[ap_num];
}

uint32_t
NrMacTimeStepEnv::ChangeMcot(uint32_t mcot, uint16_t ap_num)
{
  // std::cout << "Debug ChangeMcot()" << std::endl;
  //Change the Mcot of the gNbs at each timestep
  m_mcot[ap_num] = mcot;
  return m_new_mcot[ap_num];
}

uint32_t 
NrMacTimeStepEnv::ChangeAifsn (uint32_t aifsn, uint16_t ap_num)
{
  // std::cout << "Debug ChangeDeferTime()" << std::endl;
  // std::cout << "  with deferTime : " << deferTime << " and ap_num : " << ap_num << std::endl;
  m_aifsn[ap_num] = aifsn;
  return m_new_aifsn[ap_num];
}

uint32_t
NrMacTimeStepEnv::ChangeAmpdu(uint32_t ampdu, uint16_t ap_num)
{
  // std::cout << "Debug ChangeMcot()" << std::endl;
  //Change the Mcot of the gNbs at each timestep
  m_ampdu[ap_num] = ampdu;
  return m_new_ampdu[ap_num];
}

uint32_t
NrMacTimeStepEnv::ChangePacketSize(uint32_t packetSize, uint16_t ap_num)
{
  // std::cout << "Debug ChangePacketSize()" << std::endl;
  //Change the packetSize of the gNbs at each timestep
  m_packetSize[ap_num] = packetSize;
  return m_new_packetSize[ap_num];
}

uint32_t
NrMacTimeStepEnv::ChangeUdpLambda(uint32_t udpLambda, uint16_t ap_num)
{
  // std::cout << "Debug ChangeUdpLambda()" << std::endl;
  //Change the udpLambda of the gNbs at each timestep for trafficType UDP
  m_udpLambda[ap_num] = udpLambda;
  return m_new_udpLambda[ap_num];
}

} // namespace ns3
