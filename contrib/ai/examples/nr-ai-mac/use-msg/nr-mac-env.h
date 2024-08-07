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

#pragma once
#include <ns3/core-module.h>

/*
#include "ns3/log.h"
#include "ns3/config-store.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/nr-u-module.h"
#include "ns3/nr-module.h"
#include "ns3/propagation-module.h"
#include "ns3/wifi-module.h"
#include "ns3/config-store-module.h"
#include "ns3/antenna-module.h"
*/

#include <ns3/ai-module.h>
#include <vector>

namespace ns3 
{
struct NrMacEnv
{
  uint8_t envType;
  int64_t simTime;
  uint32_t minCw1;
  uint32_t minCw2;
  uint32_t mcot1;
  uint32_t mcot2;
  uint32_t trafficType1;
  uint32_t trafficType2;
  uint32_t packetSize1;
  uint32_t packetSize2; 
  uint32_t udpLambda1; 
  uint32_t udpLambda2; 
  double throughput1;
  double throughput2;
};

struct NrMacAct
{
  uint32_t new_minCw1;
  uint32_t new_minCw2;
  uint32_t new_mcot1;
  uint32_t new_mcot2;
  uint32_t new_trafficType1;
  uint32_t new_trafficType2;
  uint32_t new_packetSize1;
  uint32_t new_packetSize2;
  uint32_t new_udpLambda1;
  uint32_t new_udpLambda2;
};

class NrMacTimeStepEnv : public Object
{
  public:
    NrMacTimeStepEnv();
    ~NrMacTimeStepEnv() override;

    void GiveThroughput (double throughput, uint16_t ap_num);
    uint32_t ChangeMacType (uint32_t macType);
    uint32_t ChangeTrafficType (uint32_t trafficType, uint16_t ap_num);
    uint32_t ChangeMinCw (uint32_t minCw, uint16_t ap_num);
    uint32_t ChangeMcot (uint32_t mcot, uint16_t ap_num);
    uint32_t ChangePacketSize (uint32_t packetSize, uint16_t ap_num);
    uint32_t ChangeUdpLambda (uint32_t udpLambda, uint16_t ap_num);
    void ScheduleNextStateRead ();
  
  private:
    uint32_t m_nodeId;
    bool m_started{false};
    Time m_timeStep{MilliSeconds(500)};

    //state
    double m_throughput1;
    double m_throughput2;
    uint32_t m_macType;
    uint32_t m_minCw1{0};
    uint32_t m_minCw2{0};
    uint32_t m_mcot1{5};
    uint32_t m_mcot2{5};
    uint32_t m_trafficType1{0};
    uint32_t m_trafficType2{0};
    uint32_t m_packetSize1{1000};
    uint32_t m_packetSize2{1000}; 
    uint32_t m_udpLambda1{5};
    uint32_t m_udpLambda2{5}; 

    //action
    uint32_t m_new_macType;
    uint32_t m_new_minCw1{0};
    uint32_t m_new_minCw2{0};
    uint32_t m_new_mcot1{5};
    uint32_t m_new_mcot2{5};
    uint32_t m_new_trafficType1{0};
    uint32_t m_new_trafficType2{0};
    uint32_t m_new_packetSize1{1000};
    uint32_t m_new_packetSize2{1000};
    uint32_t m_new_udpLambda1{5};
    uint32_t m_new_udpLambda2{5};
};

} // namespace ns3
