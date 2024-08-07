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

namespace ns3 {

class NrMacEnv : public OpenGymEnv
{
public:
  NrMacEnv ();
  ~NrMacEnv () override;

  virtual void GiveThroughput (double throughput, uint16_t ap_num) = 0;
  virtual void GiveDelay (double delay, uint16_t ap_num) = 0;
  virtual void GiveAirTime (double airtime, uint16_t ap_num) = 0;
  virtual void GiveRxPower (double rxPower, uint16_t ap_num, uint16_t sta_num) = 0;

  virtual double ChangeTxPower (double txPower, uint16_t ap_num) = 0;
  virtual uint32_t ChangeMinCw (uint32_t minCw, uint16_t ap_num) = 0;
  virtual uint32_t ChangeMcot (uint32_t mcot, uint16_t ap_num) = 0;
  virtual uint32_t ChangeAmpdu (uint32_t ampdu, uint16_t ap_num) = 0;
  virtual uint8_t ChangeMcs (uint8_t mcs, uint16_t ap_num) = 0;
  virtual double ChangeEDThreshold (double edThreshold, uint16_t ap_num) = 0;
  virtual uint32_t ChangeBackoffType (uint32_t backoffType, uint16_t ap_num) = 0;
  virtual double ChangeSlotTime (double slotTime, uint16_t ap_num) = 0;
  virtual double ChangeDeferTime (double slotTime, uint16_t ap_num) = 0;
  virtual uint32_t ChangeAifsn (uint32_t aifsn, uint16_t ap_num) = 0;
  virtual uint32_t ChangePacketSize (uint32_t packetSize, uint16_t ap_num) = 0;
  virtual uint32_t ChangeUdpLambda (uint32_t udpLambda, uint16_t ap_num) = 0;
  virtual uint32_t ChangeTrafficType (uint32_t trafficType, uint16_t ap_num) = 0;
  

  // OpenGym interface
  Ptr<OpenGymSpace> GetActionSpace() override;
  bool GetGameOver() override;
  float GetReward() override;
  std::string GetExtraInfo() override;
  bool ExecuteActions(Ptr<OpenGymDataContainer> action) override;

  virtual void ScheduleNextStateRead () = 0;

  Ptr<OpenGymSpace> GetObservationSpace() override = 0;
  Ptr<OpenGymDataContainer> GetObservation() override = 0;

protected:
  //reward
  float m_envReward;

  //actions;
  std::vector<uint32_t> m_new_mcot;
  std::vector<uint32_t> m_new_ampdu;
  std::vector<double> m_new_txPower;
  std::vector<uint8_t> m_new_mcs;
  std::vector<double> m_new_edThreshold;
  std::vector<uint32_t> m_new_backoffType;
  std::vector<uint32_t> m_new_minCw;
  std::vector<double> m_new_slotTime;
  std::vector<double> m_new_deferTime;
  std::vector<uint32_t> m_new_aifsn;
  std::vector<uint32_t> m_new_trafficType;
  std::vector<uint32_t> m_new_packetSize;
  std::vector<uint32_t> m_new_udpLambda;
};

class NrMacTimeStepEnv : public NrMacEnv
{
public:
  // NrMacTimeStepEnv ();
  NrMacTimeStepEnv (uint32_t num_ap, uint32_t trafficType, std::vector<uint32_t> udpLambda, uint32_t packetSize);
  ~NrMacTimeStepEnv() override;

  virtual void GiveThroughput (double throughput, uint16_t ap_num);
  virtual void GiveDelay (double delay, uint16_t ap_num);
  virtual void GiveAirTime (double airtime, uint16_t ap_num);
  virtual void GiveRxPower (double rxPower, uint16_t ap_num, uint16_t sta_num);

  virtual double ChangeTxPower (double txPower, uint16_t ap_num);
  virtual uint32_t ChangeMinCw (uint32_t minCw, uint16_t ap_num);
  virtual uint32_t ChangeMcot (uint32_t mcot, uint16_t ap_num);
  virtual uint32_t ChangeAmpdu (uint32_t ampdu, uint16_t ap_num);
  virtual uint8_t ChangeMcs (uint8_t mcs, uint16_t ap_num);
  virtual double ChangeEDThreshold (double edThreshold, uint16_t ap_num);
  virtual uint32_t ChangeBackoffType (uint32_t backoffType, uint16_t ap_num);
  virtual double ChangeSlotTime (double slotTime, uint16_t ap_num);
  virtual double ChangeDeferTime (double slotTime, uint16_t ap_num);
  virtual uint32_t ChangeAifsn (uint32_t aifsn, uint16_t ap_num);
  virtual uint32_t ChangePacketSize (uint32_t packetSize, uint16_t ap_num);
  virtual uint32_t ChangeUdpLambda (uint32_t udpLambda, uint16_t ap_num);
  virtual uint32_t ChangeTrafficType (uint32_t trafficType, uint16_t ap_num);

  // OpenGym interface
  Ptr<OpenGymSpace> GetObservationSpace() override;
  Ptr<OpenGymDataContainer> GetObservation() override;

private:
  virtual void ScheduleNextStateRead () override;
  bool m_started{false};
  Time m_timeStep{MilliSeconds(100)};
  
  // state
  std::vector<uint32_t> m_mcot;
  std::vector<uint32_t> m_ampdu;
  std::vector<double> m_txPower;
  std::vector<uint8_t> m_mcs;
  std::vector<double> m_edThreshold;
  std::vector<uint32_t> m_backoffType;
  std::vector<uint32_t> m_minCw;
  std::vector<double> m_slotTime;
  std::vector<double> m_deferTime;
  std::vector<uint32_t> m_aifsn;
  
  std::vector<uint32_t> m_trafficType;
  std::vector<uint32_t> m_packetSize;
  std::vector<uint32_t> m_udpLambda;
  
  std::vector<std::vector<double>> m_rxPower;
  std::vector<double> m_throughput;
  std::vector<double> m_airTime;
  std::vector<double> m_delay;
};

} // namespace ns3
