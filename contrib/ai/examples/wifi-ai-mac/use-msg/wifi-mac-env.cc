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

#include "wifi-mac-env.h"
#include <numeric>
#include <iostream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("wifi-mac-env-msg");

NS_OBJECT_ENSURE_REGISTERED(WifiMacTimeStepEnv);

WifiMacTimeStepEnv::WifiMacTimeStepEnv()
{
  auto interface = Ns3AiMsgInterface::Get();
  interface->SetIsMemoryCreator(false);
  interface->SetUseVector(false);
  interface->SetHandleFinish(true);
}

WifiMacTimeStepEnv::~WifiMacTimeStepEnv()
{

}

void WifiMacTimeStepEnv::ScheduleNextStateRead()
{
  std::cout << "Debug ns3env ScheduleNextStateRead()" << std::endl;
  Simulator::Schedule(m_timeStep, &WifiMacTimeStepEnv::ScheduleNextStateRead, this);

  Ns3AiMsgInterfaceImpl<WifiMacEnv, WifiMacAct>* msgInterface =
        Ns3AiMsgInterface::Get()->GetInterface<WifiMacEnv, WifiMacAct>();

  msgInterface->CppSendBegin();
  auto env = msgInterface->GetCpp2PyStruct();
  env->envType = 1;
  env->simTime = Simulator::Now().GetSeconds();

  env->throughput1 = m_throughput1;
  env->throughput1 = m_throughput2;
  env->minCw1 = m_minCw1;
  env->minCw2 = m_minCw2;
  env->mcot1 = m_mcot1;
  env->mcot2 = m_mcot2;
  env->trafficType1 = m_trafficType1;
  env->trafficType2 = m_trafficType2;
  env->packetSize1 = m_packetSize1;
  env->packetSize2 = m_packetSize2;
  env->udpLambda1 = m_udpLambda1;
  env->udpLambda2 = m_udpLambda2;

  msgInterface->CppSendEnd();
  
  msgInterface->CppRecvBegin();
  auto act = msgInterface->GetPy2CppStruct();

  m_new_minCw1 = act->new_minCw1;
  m_new_minCw2 = act->new_minCw2;
  m_new_mcot1 = act->new_mcot1;
  m_new_mcot2 = act->new_mcot2;
  m_new_trafficType1 = act->new_trafficType1;
  m_new_trafficType2 = act->new_trafficType2;
  m_new_packetSize1 = act->new_packetSize1;
  m_new_packetSize2 = act->new_packetSize2;
  m_new_udpLambda1 = act->new_udpLambda1;
  m_new_udpLambda2 = act->new_udpLambda2;
  
  msgInterface->CppRecvEnd();
}

void
WifiMacTimeStepEnv::GiveThroughput(double throughput, uint16_t ap_num)
{
  std::cout << "Debug ns3env GiveThroughput()" << std::endl;
  std::cout << "  with throughput : " << throughput << " and ap_num : "  << ap_num << std::endl;
  //Update the throughput from after each timestep
  if (ap_num == 0)
  {
    m_throughput1 = throughput;
  }
  else
  {
    m_throughput2 = throughput;
  }
}

uint32_t
WifiMacTimeStepEnv::ChangeMacType(uint32_t macType)
{
  std::cout << "Debug ChangeMacType()" << std::endl;
  //Change the macType of the gNbs at each timestep
  m_macType = macType;

  // if (!m_started)
  // {
  //   m_started = true;
  //   // Notify();
  //   ScheduleNextStateRead();
  // }

  return m_new_macType;
}

uint32_t
WifiMacTimeStepEnv::ChangeTrafficType(uint32_t trafficType, uint16_t ap_num)
{
  //Change the trafficType of the gNbs at each timestep (Burst or UDP)
  std::cout << "Debug ChangeTrafficType()" << std::endl;
  if (ap_num == 0)
  {
    m_trafficType1 = trafficType;
  }
  else
  {
    m_trafficType2 = trafficType;
  }

  // if (!m_started)
  // {
  //   m_started = true;
  //   // Notify();
  //   ScheduleNextStateRead();
  // }
  
  if (ap_num == 0)
  {
    return m_new_trafficType1;
  }
  else
  {
    return m_new_trafficType2;
  }
}

uint32_t
WifiMacTimeStepEnv::ChangeMinCw(uint32_t minCw, uint16_t ap_num)
{
  //Change the minCw of the gNbs at each timestep
  std::cout << "Debug ChangeMinCw()" << std::endl;
  std::cout << "  with minCw : " << minCw << " and ap_num : " << ap_num << std::endl;
  if (ap_num == 0)
  {
    m_minCw1 = minCw;
  }
  else
  {
    m_minCw2 = minCw;
  }
  
  // if (!m_started)
  // {
  //   m_started = true;
  //   // Notify();
  //   ScheduleNextStateRead();
  // }

  if (ap_num == 0)
  {
    return m_new_minCw1;
  }
  else
  {
    return m_new_minCw2;
  }
}

uint32_t
WifiMacTimeStepEnv::ChangeMcot(uint32_t mcot, uint16_t ap_num)
{
  std::cout << "Debug ChangeMcot()" << std::endl;
  //Change the Mcot of the gNbs at each timestep
  if (ap_num == 0)
  {
    m_mcot1 = mcot;
  }
  else
  {
    m_mcot2 = mcot;
  }

  // if (!m_started)
  // {
  //   m_started = true;
  //   // Notify();
  //   ScheduleNextStateRead();
  // }

  if (ap_num == 0)
  {
    return m_new_mcot1;
  }
  else
  {
    return m_new_mcot2;
  }
}

uint32_t
WifiMacTimeStepEnv::ChangePacketSize(uint32_t packetSize, uint16_t ap_num)
{
  std::cout << "Debug ChangePacketSize()" << std::endl;
  //Change the packetSize of the gNbs at each timestep
  if (ap_num == 0)
  {
    m_packetSize1 = packetSize;
  }
  else
  {
    m_packetSize2 = packetSize;
  }

  // if (!m_started)
  // {
  //   m_started = true;
  //   // Notify();
  //   ScheduleNextStateRead();
  // }

  if (ap_num == 0)
  {
    return m_new_packetSize1;
  }
  else
  {
    return m_new_packetSize2;
  }
}

uint32_t
WifiMacTimeStepEnv::ChangeUdpLambda(uint32_t udpLambda, uint16_t ap_num)
{
  std::cout << "Debug ChangeUdpLambda()" << std::endl;
  //Change the udpLambda of the gNbs at each timestep for trafficType UDP
  if (ap_num == 0)
  {
    m_udpLambda1 = udpLambda;
  }
  else
  {
    m_udpLambda2 = udpLambda;
  }

  // if (!m_started)
  // {
  //   m_started = true;
  //   // Notify();
  //   ScheduleNextStateRead();
  // }

  if (ap_num == 0)
  {
    return m_new_udpLambda1;
  }
  else
  {
    return m_new_udpLambda2;
  }
}

} // namespace ns3
