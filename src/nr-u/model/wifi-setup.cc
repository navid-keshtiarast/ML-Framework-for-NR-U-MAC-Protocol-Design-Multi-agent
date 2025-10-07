/* Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; */
/*
 *   Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *   Copyright (c) 2021 Orange Labs
 *   Copyright (c) 2024 RWTH Aachen University
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License version 2 as
 *   published by the Free Software Foundation;
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *   Modify: Oliver Renaldi <oliver.renaldi@rwth-aachen.de> 
 */

#include "wifi-setup.h"
#include <ns3/wifi-module.h>
#include <ns3/multi-model-spectrum-channel.h>
#include <ns3/three-gpp-propagation-loss-model.h>
#include <ns3/three-gpp-spectrum-propagation-loss-model.h>
#include <ns3/beamforming-vector.h>
#include <ns3/internet-stack-helper.h>
#include <ns3/ipv4-static-routing-helper.h>
#include <ns3/point-to-point-helper.h>
#include <ns3/ipv4-address-helper.h>
#include <ns3/log.h>
#include <ns3/names.h>
#include <ns3/pointer.h>
#include <ns3/string.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("WifiSetup");

void
DataReceived (Ptr<WifiMac> wifiMac, Ptr<const Packet> p)
{
  NS_LOG_DEBUG ("\nDevice " << wifiMac->GetAddress () << " received DATA: " << *p);
}

void
StationAssociated (Ptr<StaWifiMac> staWifiMac, Mac48Address address)
{
  NS_LOG_DEBUG ("\nSTA " << staWifiMac->GetAddress () << " associated with AP: " << address);
}

void
BeaconArrival (Ptr<StaWifiMac> staWifiMac, Time time)
{
  NS_LOG_DEBUG ("\nSTA " << staWifiMac->GetAddress () << " reports beacon arrival at:" <<
                time.As (Time::S));
}

void
BeaconQueued (Ptr<ApWifiMac> apWifiMac, Time time)
{
  NS_LOG_DEBUG ("\nAP " << apWifiMac->GetAddress () << " queued beacon at:" <<
                time.As (Time::S));
}


WifiSetup::WifiSetup (const NodeContainer &apNodes, const NodeContainer &staNodes,
                      const Ptr<SpectrumChannel> &channel,
                      const Ptr<ThreeGppPropagationLossModel> &propagation,
                      const Ptr<ThreeGppSpectrumPropagationLossModel> & spectrumPropagation,
                      double freq, double bw,
                      double apTxPower, double staTxPower,
                      double ccaThresholdAp, double ccaThresholdSta,
                      enum WifiStandard standard, const std::string &ssid, int frameaggr,
                      uint32_t wifiMpdu, uint32_t wifiMsdu, 
                      uint32_t wifiMinCw, uint32_t wifiAifsn, uint32_t wifiBackoffType,
                      uint32_t wifiSlotTime, 
                      uint32_t wifiMcs)
  : L2Setup (apNodes, staNodes)
{
  NS_LOG_FUNCTION (this);
  // NS_ABORT_MSG_IF (channel == nullptr || propagation == nullptr || spectrumPropagation == nullptr,
  //                  "The SpectrumChannel, PropagationLossModel and SpectrumPropagationLossModel should be configured");
  std::cout << "Configuring Wi-Fi with " << apNodes.GetN ()
            << " APs, and " << staNodes.GetN () << " STAs" << std::endl;

  for (auto it = apNodes.Begin (); it != apNodes.End (); ++it)
    {
      std::stringstream ss;
      ss << "AP-WIFI-" << ssid << "-" << (*it)->GetId ();
      Names::Add (ss.str (), *it);
    }

  for (auto it = staNodes.Begin (); it != staNodes.End (); ++it)
    {
      std::stringstream ss;
      ss << "STA-WIFI-" << ssid << "-" << (*it)->GetId ();
      Names::Add (ss.str (), *it);
    }

  double MPDU=0;
  double MSDU=0;
  if (frameaggr==0)
    {
      MPDU=0;
      MSDU=0;
    }
  else if (frameaggr==1)
    {
      MPDU=6500631;//;262143;//;6500631;
      MSDU=0;
    }
  else if (frameaggr==2)
    {
      MPDU=0;
      MSDU=11398;//7935;//11398;
    }
  
  MPDU = wifiMpdu*1000;
  MSDU = wifiMsdu*1000;

  m_wifiHelper.SetStandard (standard);

  if (wifiMcs == -1)
  {
    m_wifiHelper.SetRemoteStationManager ("ns3::MinstrelHtWifiManager");
  }
  else
  {
    std::ostringstream ossDataMode;
    ossDataMode << "HeMcs" << wifiMcs;
    StringValue ctrlRate = StringValue(ossDataMode.str());
    m_wifiHelper.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                              "DataMode",
                                              StringValue(ossDataMode.str()),
                                              "ControlMode",
                                              ctrlRate);
  }

  m_wifiMacHelper.SetType ("ns3::ApWifiMac",
                   "Ssid", SsidValue (Ssid (ssid)),
                   "BE_MaxAmpduSize", UintegerValue (MPDU), //Enable A-MPDU with the highest maximum size allowed by the standard
                   "BE_MaxAmsduSize", UintegerValue (MSDU));   //Enable A-MSDU with the highest maximum size (in Bytes) allowed by the standard
                   //"BeaconInterval", TimeValue (MicroSeconds (102400)));

  m_spectrumWifiPhyHelper.SetChannel (channel);
  m_spectrumWifiPhyHelper.Set ("TxPowerStart", DoubleValue (apTxPower));
  m_spectrumWifiPhyHelper.Set ("TxPowerEnd", DoubleValue (apTxPower));
  m_spectrumWifiPhyHelper.Set ("TxPowerLevels", UintegerValue (1));
  /* Sensitivity model includes implementation loss and noise figure */
  m_spectrumWifiPhyHelper.Set ("CcaEdThreshold", DoubleValue (ccaThresholdAp));
  m_spectrumWifiPhyHelper.Set ("ChannelWidth", UintegerValue (static_cast<uint16_t> (bw / 1e6)));
  //spectrumWifiPhy.Set ("ChannelNumber", UintegerValue (7));
  m_spectrumWifiPhyHelper.Set ("Frequency", UintegerValue (static_cast<int> (freq / 1e6)));
  /* Set the PHY layer error model (switch to NistErrorRateModel if needed) */
  m_spectrumWifiPhyHelper.SetErrorRateModel ("ns3::TableBasedErrorRateModel");
  //spectrumWifiPhy.Set("Antennas",UintegerValue (1));

  for (auto it = GetGnbNodes ().Begin (); it != GetGnbNodes ().End (); ++it)
    {
      m_gnbDev.Add (m_wifiHelper.Install (m_spectrumWifiPhyHelper, m_wifiMacHelper, (*it)));
    }

  // Install NetDevice on STAs
  m_wifiMacHelper.SetType ("ns3::StaWifiMac",
                   "Ssid", SsidValue (Ssid (ssid)),
                   "ActiveProbing", BooleanValue (false),
                   //"AssocRequestTimeout", TimeValue (MilliSeconds (20)), //because the default 0.5s will lead to systematic collisions with periodic NR-U DL_CTRL frames
                   "BE_MaxAmpduSize", UintegerValue (MPDU),   //Enable A-MPDU with the highest maximum size allowed by the standard
                   "BE_MaxAmsduSize", UintegerValue (MSDU));   //Enable A-MSDU with the highest maximum size (in Bytes) allowed by the standard

  m_spectrumWifiPhyHelper.Set ("TxPowerStart", DoubleValue (staTxPower));
  m_spectrumWifiPhyHelper.Set ("TxPowerEnd", DoubleValue (staTxPower));
  m_spectrumWifiPhyHelper.Set ("CcaEdThreshold", DoubleValue (ccaThresholdSta));

  for (auto it = GetUeNodes ().Begin (); it != GetUeNodes ().End (); ++it)
    {
      m_ueDev.Add (m_wifiHelper.Install (m_spectrumWifiPhyHelper, m_wifiMacHelper, *it));
    }

  m_spectrumWifiPhyHelper.SetPcapDataLinkType(SpectrumWifiPhyHelper::DLT_IEEE802_11_RADIO);
  // m_spectrumWifiPhyHelper.EnablePcap ("wifiPcap", m_gnbDev);
  // m_spectrumWifiPhyHelper.EnablePcap ("wifiPcap", m_ueDev);

  // Configure NetDevices
  for (uint32_t i = 0; i < m_gnbDev.GetN (); ++i)
    {
      auto netDevice =  DynamicCast<WifiNetDevice> (m_gnbDev.Get (i));
      auto apWifiMac = StaticCast<ApWifiMac> (netDevice->GetMac ());
      auto apWifiPhy = DynamicCast<SpectrumWifiPhy> (netDevice->GetPhy ());

      apWifiMac->TraceConnectWithoutContext ("BeaconQueued", MakeBoundCallback (&BeaconQueued, apWifiMac));
      apWifiMac->TraceConnectWithoutContext ("MacPromiscRx", MakeBoundCallback (&DataReceived, apWifiMac));

      char node_id[12];
      snprintf (node_id, sizeof(node_id), "%d", netDevice->GetNode ()->GetId ());
      const std::string nodeId (node_id);
      apWifiPhy->TraceConnect ("PhyRxDrop", nodeId,
                               MakeCallback (&WifiSetup::PhyRxDrop, this));
      apWifiPhy->TraceConnect ("PhyTxDrop", nodeId,
                               MakeCallback (&WifiSetup::PhyTxDrop, this));
      apWifiPhy->TraceConnect ("MonitorSnifferRx", nodeId,
                               MakeCallback (&WifiSetup::RxDataTrace, this));

      PointerValue statePtr;
      apWifiPhy->GetAttribute ("State", statePtr);
      Ptr <WifiPhyStateHelper> state = DynamicCast <WifiPhyStateHelper> (statePtr.Get<WifiPhyStateHelper> ());
      state->TraceConnect ("State", nodeId,
                           MakeCallback (&WifiSetup::TxOccupancy, this));

      PointerValue ptr;
      apWifiMac->GetAttribute ("BE_Txop", ptr);
      m_beTxop = ptr.Get<QosTxop> ();
      m_beTxop->TraceConnect ("TxopTrace", nodeId,
                             MakeCallback (&WifiSetup::ChannelRequestToAccessDuration, this));
      m_beTxop->SetMinCw(wifiMinCw);
      m_beTxop->SetAifsn(wifiAifsn);
      m_beTxop->SetBackoffType(wifiBackoffType);

      apWifiPhy->SetSlot(MicroSeconds(wifiSlotTime));
    }

  for (uint32_t i = 0; i < m_ueDev.GetN (); ++i)
    {
      auto netDevice =  DynamicCast<WifiNetDevice> (m_ueDev.Get (i));
      auto staWifiMac = StaticCast<StaWifiMac> (netDevice->GetMac ());
      auto staWifiPhy = DynamicCast<SpectrumWifiPhy> (netDevice->GetPhy ());

      staWifiMac->TraceConnectWithoutContext ("BeaconArrival", MakeBoundCallback (&BeaconArrival, staWifiMac));
      staWifiMac->TraceConnectWithoutContext ("DataReceived", MakeBoundCallback (&DataReceived, staWifiMac));

      char node_id[12];
      snprintf (node_id, sizeof(node_id), "%d", netDevice->GetNode ()->GetId ());
      const std::string nodeId (node_id);
      staWifiPhy->TraceConnect ("PhyRxDrop", nodeId,
                                MakeCallback (&WifiSetup::PhyRxDrop, this));
      staWifiPhy->TraceConnect ("PhyTxDrop", nodeId,
                                MakeCallback (&WifiSetup::PhyTxDrop, this));
      staWifiPhy->TraceConnect ("MonitorSnifferRx", nodeId,
                                MakeCallback (&WifiSetup::RxDataTrace, this));

      PointerValue statePtr;
      staWifiPhy->GetAttribute ("State", statePtr);
      Ptr <WifiPhyStateHelper> state = DynamicCast <WifiPhyStateHelper> (statePtr.Get<WifiPhyStateHelper> ());
      state->TraceConnect ("State", nodeId,
                           MakeCallback (&WifiSetup::TxOccupancy, this));
    }

  // Dummy antenna array in Omni mode
  UintegerValue numCols, numRows;
  Ptr<UniformPlanarArray> antennaArray = CreateObject<UniformPlanarArray> ();
  antennaArray->GetAttribute ("NumColumns", numCols);
  antennaArray->GetAttribute ("NumRows", numRows);
  //antennaArray->SetBeamformingVector (CreateQuasiOmniBfv (numCols.Get (), numRows.Get ()));
  // // registering AP devices to 3GPP model
  // for (uint32_t i = 0; i < m_gnbDev.GetN (); ++i)
  //   {
  //     Ptr<WifiNetDevice> netDevice =  DynamicCast<WifiNetDevice> (m_gnbDev.Get (i));
  //     spectrumPropagation->AddDevice (netDevice, antennaArray);
  //   }

  // // registering STAs devices to 3GPP model
  // for (uint32_t i = 0; i < m_ueDev.GetN (); ++i)
  //   {
  //     Ptr<WifiNetDevice> netDevice =  DynamicCast<WifiNetDevice> (m_ueDev.Get (i));
  //     spectrumPropagation->AddDevice (netDevice, antennaArray);
  //   }

}

WifiSetup::~WifiSetup ()
{
  NS_LOG_FUNCTION (this);
}

Ipv4InterfaceContainer
WifiSetup::AssignIpv4ToUe (const std::unique_ptr<Ipv4AddressHelper> &address) const
{
  NS_LOG_FUNCTION (this);
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  /* Internet stack*/
  InternetStackHelper stack;
  stack.Install (GetUeNodes ());

  Ipv4InterfaceContainer staInterface;
  staInterface = address->Assign (GetUeDev ());

  for (auto it = GetUeNodes ().Begin (); it != GetUeNodes ().End (); ++it)
    {
      Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting ((*it)->GetObject<Ipv4> ());
      remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("0.0.0.0"), Ipv4Mask ("0.0.0.0"), 1);
    }

  return staInterface;
}

Ipv4InterfaceContainer
WifiSetup::AssignIpv4ToStations (const std::unique_ptr<Ipv4AddressHelper> &address) const
{
  NS_LOG_FUNCTION (this);
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  InternetStackHelper stack;
  stack.Install (GetGnbNodes ());
  Ipv4InterfaceContainer apInterface;
  apInterface = address->Assign (GetGnbDev ());

  return apInterface;
}

Ipv4InterfaceContainer
WifiSetup::ConnectToRemotes (const NodeContainer &remoteHosts, const std::string &base)
{
  NS_LOG_FUNCTION (this << remoteHosts.GetN () << base);
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (2500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.000)));

  std::cout << "Connecting remote hosts to the APs with a 100 Gb/s link, MTU 2500 B, delay 0 s" << std::endl;

  NetDeviceContainer internetDevices;
  for (auto it = remoteHosts.Begin (); it != remoteHosts.End (); ++it)
    {
      for (auto it2 = GetGnbNodes ().Begin (); it2 != GetGnbNodes ().End (); ++it2)
        {
          internetDevices.Add (p2ph.Install (*it2, *it));
        }
    }

  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase (base.c_str (), "255.255.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);

  return internetIpIfaces;
}

void
WifiSetup::SetChannelRTACallback (const ChannelRequestToAccessDurationCb &cb)
{
  m_craCb = cb;
}

void
WifiSetup::PhyRxDrop (std::string context, Ptr<const Packet> p, WifiPhyRxfailureReason)
{
  if (!m_macTxDataFailedCb.IsNull ())
    {
      m_macTxDataFailedCb (static_cast<uint32_t> (std::stoul (context)), p->GetSize ());
    }
}

void
WifiSetup::PhyTxDrop (std::string context, Ptr<const Packet> p)
{
  if (!m_macTxDataFailedCb.IsNull ())
    {
      m_macTxDataFailedCb (static_cast<uint32_t> (std::stoul (context)), p->GetSize ());
    }
}

void
WifiSetup::TxOccupancy (std::string context, Time start, Time duration, WifiPhyState newState)
{
  if (newState == WifiPhyState::TX)
  {
    if (!m_channelOccupancyTimeCb.IsNull ())
      {
        m_channelOccupancyTimeCb (static_cast<uint32_t> (std::stoul (context)), duration);
      }
  }
}

void
WifiSetup::RxDataTrace (std::string context, Ptr<const Packet>, uint16_t, WifiTxVector, MpduInfo, SignalNoiseDbm signalNoise, uint16_t terminal_id)
{
  if (!m_sinrCb.IsNull ())
    {
      m_sinrCb (static_cast<uint32_t> (std::stoul (context)), signalNoise.signal / signalNoise.noise);
    }
}

void
WifiSetup::ChannelRequestToAccessDuration (std::string context, Time oldValue, Time newValue)
{
  if (!m_craCb.IsNull ())
    {
      m_craCb (static_cast<uint32_t> (std::stoul (context)), newValue);
    }
}

Ptr<NrHelper> WifiSetup::GetNrHelper ()
{
  Ptr<NrHelper> nrHelperNull = CreateObject<NrHelper> ();
  return nrHelperNull;
}

Ptr<ns3::NrPointToPointEpcHelper> WifiSetup::GetEpcHelper ()
{
  Ptr<NrPointToPointEpcHelper> epcHelperNull = CreateObject<NrPointToPointEpcHelper> ();
  return epcHelperNull;
}

WifiHelper WifiSetup::GetWifiHelper ()
{
  return m_wifiHelper;
}

WifiMacHelper WifiSetup::GetWifiMacHelper ()
{
  return m_wifiMacHelper;
}

SpectrumWifiPhyHelper WifiSetup::GetSpectrumWifiPhyHelper ()
{
  return m_spectrumWifiPhyHelper;
}

Ptr<QosTxop> WifiSetup::GetTxOp ()
{
  return m_beTxop;
}

BandwidthPartInfoPtrVector WifiSetup::GetBwpVector ()
{
  BandwidthPartInfoPtrVector allBwpNull;
  return allBwpNull;
}

void WifiSetup::ChangeGnbTxPower (double newTxPower)
{
  // std::cout << "Debug ChangeGnbTxPower()" << std::endl;
  for(uint32_t i = 0; i < m_gnbDev.GetN(); i++)
  {
    auto netDevice =  DynamicCast<WifiNetDevice> (m_gnbDev.Get (i));
    auto apWifiPhy = DynamicCast<SpectrumWifiPhy> (netDevice->GetPhy ());
    apWifiPhy->SetAttribute ("TxPowerStart", DoubleValue(newTxPower));
    apWifiPhy->SetAttribute ("TxPowerEnd", DoubleValue(newTxPower));
  }
} 

void WifiSetup::ChangeMcot (Time newMcot)
{
}

void WifiSetup::ChangeBackoffType (uint32_t newBackoffType)
{
}

void WifiSetup::ChangeDeferTime (Time newDeferTime)
{
}

void WifiSetup::ChangeMinCw (uint32_t newMinCw)
{
}

void WifiSetup::ChangeMcs (uint32_t mcsValue)
{
  // std::cout << "Debug ChangeMcs()" << std::endl;
  std::ostringstream ossDataMode;
  ossDataMode << "HeMcs" << mcsValue;
  StringValue ctrlRate = StringValue(ossDataMode.str());
  for(uint32_t i = 0; i < m_gnbDev.GetN(); i++)
  {
    auto netDevice =  DynamicCast<WifiNetDevice> (m_gnbDev.Get (i));
    netDevice->GetRemoteStationManager()->SetAttribute("DataMode",StringValue(ossDataMode.str()));
    netDevice->GetRemoteStationManager()->SetAttribute("ControlMode",ctrlRate);
  }
}

void WifiSetup::ChangeAmpdu (uint32_t newAmpdu)
{
  // std::cout << "Debug ChangeAmpdu()" << std::endl;
  for(uint32_t i = 0; i < m_gnbDev.GetN(); i++)
  {
    auto netDevice =  DynamicCast<WifiNetDevice> (m_gnbDev.Get (i));
    auto apWifiMac = StaticCast<ApWifiMac> (netDevice->GetMac ());
    apWifiMac->SetAttribute ("BE_MaxAmpduSize", UintegerValue(newAmpdu));
  }
}

void WifiSetup::ChangeAmsdu (uint32_t newAmsdu)
{
  // std::cout << "Debug ChangeAmsdu()" << std::endl;
  for(uint32_t i = 0; i < m_gnbDev.GetN(); i++)
  {
    auto netDevice =  DynamicCast<WifiNetDevice> (m_gnbDev.Get (i));
    auto apWifiMac = StaticCast<ApWifiMac> (netDevice->GetMac ());
    apWifiMac->SetAttribute ("BE_MaxAmsduSize", UintegerValue(newAmsdu));
  }
}

void WifiSetup::ChangeEdThreshold (double newEdThreshold)
{
  // std::cout << "Debug ChangeEdThreshold()" << std::endl;
  for(uint32_t i = 0; i < m_gnbDev.GetN(); i++)
  {
    auto netDevice =  DynamicCast<WifiNetDevice> (m_gnbDev.Get (i));
    auto apWifiPhy = DynamicCast<SpectrumWifiPhy> (netDevice->GetPhy ());
    apWifiPhy->SetAttribute ("CcaEdThreshold", DoubleValue(newEdThreshold));
  }
}

void WifiSetup::ChangeSlotTime (uint32_t newSlotTime)
{
  for(uint32_t i = 0; i < m_gnbDev.GetN(); i++)
  {
    auto netDevice =  DynamicCast<WifiNetDevice> (m_gnbDev.Get (i));
    auto apWifiPhy = DynamicCast<SpectrumWifiPhy> (netDevice->GetPhy ());
    apWifiPhy->SetAttribute ("Slot", TimeValue (MicroSeconds (newSlotTime)));
  }
}
} // namespace ns3
