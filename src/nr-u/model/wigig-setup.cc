/* Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; */
/*
 *   Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
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
 */
#include "wigig-setup.h"
#include <ns3/wifi-80211ad-nist-module.h>
#include <ns3/multi-model-spectrum-channel.h>
#include <ns3/internet-stack-helper.h>
#include <ns3/ipv4-static-routing-helper.h>
#include <ns3/point-to-point-helper.h>
#include <ns3/ipv4-address-helper.h>
#include <ns3/log.h>
#include <ns3/names.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("WigigSetup");

void
BIStarted (Ptr<DmgApWifiMac> wifiMac, Mac48Address address)
{
  NS_LOG_DEBUG ("\n\nBTI started at:" << Simulator::Now ().GetSeconds () << " seconds.");
}

void
DTIStarted (Ptr<DmgApWifiMac> wifiMac, Mac48Address address, Time time)
{
  NS_LOG_DEBUG ("\nDTI started at:" << Simulator::Now ().GetSeconds () << " seconds.");
}


void ContentionPeriodStarted (Ptr<DmgWifiMac> dmgMac, Time started, uint8_t allocationID, Time duration)
{
  NS_LOG_DEBUG ("\nCBAP started at:" << started.GetSeconds () << " seconds, allocationID:" << +allocationID <<
                " duration:" << duration.GetMilliSeconds () << " ms, reported by DMG:" << dmgMac->GetAddress ());
}


void SSWReceivedFromResponder (Ptr<DmgWifiMac> wifiMac, Time time, Mac48Address address)
{
  NS_LOG_DEBUG ("\nSSW received at: " << wifiMac->GetAddress () << " from responder: " << address << " at:" << Simulator::Now ().GetSeconds () << " seconds.");
}

void SSWReceivedFromInitiator (Ptr<DmgWifiMac> wifiMac, Time time, Mac48Address address)
{
  NS_LOG_DEBUG ("\nSSW received at:" << wifiMac->GetAddress () << " from initiator: " << address << " at:" << Simulator::Now ().GetSeconds () << " seconds.");
}

void InitiatorSSWSent (Ptr<DmgWifiMac> wifiMac, Time time, Mac48Address address)
{
  NS_LOG_DEBUG ("\nInitiator SSW sent from: " << wifiMac->GetAddress () << " to: " << address << " at: " << Simulator::Now ().GetSeconds () << " seconds.");
}

void ResponderSSWSent (Ptr<DmgWifiMac> wifiMac, Time time, Mac48Address address, uint8_t antennaID, uint8_t sectorID)
{
  NS_LOG_DEBUG ("\nResponder SSW sent from: " << wifiMac->GetAddress () << " to: " << address << " at: " << Simulator::Now ().GetSeconds () << " seconds." << " AntennaID:" << +antennaID <<
                " , sectorID:" << +sectorID);
}

void ABFTStarted (Ptr<DmgApWifiMac> wifiMac,  Time time, uint8_t ssSlotsPerABFT)
{
  NS_LOG_DEBUG ("\nABFT started at:" << time.GetSeconds () << " SS slots per ABFT:" << +ssSlotsPerABFT);
}

void SSSlotStarted (Ptr<DmgApWifiMac> wifiMac,  Time time)
{
  NS_LOG_DEBUG ("\nSS slot started at:" << time.GetSeconds ());
}

void
SLSCompleted (Ptr<DmgWifiMac> wifiMac, Mac48Address address, ChannelAccessPeriod accessPeriod,
              BeamformingDirection beamformingDirection, bool isInitiatorTxss, bool isResponderTxss,
              SECTOR_ID sectorId, ANTENNA_ID antennaId)
{
  if (DynamicCast<DmgApWifiMac> (wifiMac) != nullptr)
    {
      NS_LOG_DEBUG ( "\nSLS COMPLETED for DMG AP address: " << wifiMac->GetAddress () << " completed SLS phase with device having the address:" << address << " SectorID=" <<
                     uint (sectorId) << ", AntennaID=" << uint (antennaId) << " at:" << Simulator::Now ().GetSeconds ());
    }
  else if (DynamicCast<DmgStaWifiMac> (wifiMac) != nullptr)
    {
      NS_LOG_DEBUG ( "\nSLS COMPLETED for DMG STA: " << wifiMac->GetAddress () << " completed SLS phase with device having the address:" <<
                     address << " SectorID=" << uint (sectorId) << ", AntennaID=" << uint (antennaId) << " at:" << Simulator::Now ().GetSeconds ());
    }
  else
    {
      NS_LOG_DEBUG ("\nAI programming error, unexpected zombi mac!");
    }
}


void DataReceived (Ptr<DmgWifiMac> wifiMac, Mac48Address address, ANTENNA_ID antennaId, SECTOR_ID sectorId)
{
  NS_LOG_DEBUG ( "\nDATA received from : " << address << " SectorID=" <<
                 uint (sectorId) << ", AntennaID=" << uint (antennaId) << " at device:" << wifiMac->GetAddress ());
}

void
StationAssociated (Ptr<DmgStaWifiMac> staWifiMac, Mac48Address address)
{
  static std::map<Mac48Address, Ptr<DmgApWifiMac> > associatedStaAp;
  NS_LOG_DEBUG ( "\nDMG STA " << staWifiMac->GetAddress () << " associated with DMG AP: " << address);
}

void
BeaconArrival (Ptr<DmgStaWifiMac> staWifiMac, Time timeOld, uint8_t antennaID, uint8_t sectorID)
{

  NS_LOG_DEBUG ( "\nDMG STA " << staWifiMac->GetAddress () << " reports beacon arrival at:" <<
                 timeOld.GetSeconds () << " antennaID:" << +antennaID <<
                 " , sectorID:" << +sectorID );
}

void
BeaconSent (Ptr<DmgApWifiMac> apWifiMac, Time timeOld, uint8_t antennaID, uint8_t sectorID)
{

  NS_LOG_DEBUG ( "\nDMG AP " << apWifiMac->GetAddress () << " sends beacon at:" <<
                 timeOld.GetSeconds () << " antennaID:" << +antennaID <<
                 " , sectorID:" << +sectorID );
}


WigigSetup::WigigSetup (const NodeContainer &gnbNodes, const NodeContainer &ueNodes,
                        const Ptr<SpectrumChannel> &inputSpectrumChannel,
                        const Ptr<PropagationLossModel> &inputPathloss,
                        const Ptr<MmWave3gppChannel> &inputMmWaveChannel, double freq,
                        double bw, double apTxPower, double staTxPower, double ccaThresholdAp,
                        double ccaThresholdSta, const std::string &ssID,
                        uint16_t apAntennaDim1, uint16_t apAntennaDim2,
                        uint16_t staAntennaDim1, uint16_t staAntennaDim2)
  : L2Setup (gnbNodes, ueNodes)
{

  std::cout << "Configuring Wigig with " << gnbNodes.GetN ()
            << " gnbs, and " << ueNodes.GetN () << " UEs" << std::endl;

  for (auto it = gnbNodes.Begin(); it != gnbNodes.End(); ++it)
    {
      std::stringstream ss;
      ss << "GNB-WIGIG-" << ssID << "-" << (*it)->GetId();
      Names::Add(ss.str (), *it);
    }

  for (auto it = ueNodes.Begin(); it != ueNodes.End(); ++it)
    {
      std::stringstream ss;
      ss << "UE-WIGIG-" << ssID << "-" << (*it)->GetId();
      Names::Add(ss.str (), *it);
    }

  std::string phyMode = "DMG_MCS8";
  DmgWifiHelper wifi;

  Ptr<SpectrumChannel> channel;
  Ptr<PropagationLossModel> propagation;
  Ptr<MmWave3gppChannel> threegppChannel;

  if (inputSpectrumChannel == nullptr && inputPathloss == nullptr && inputMmWaveChannel == nullptr)
    {
      channel = CreateObject<MultiModelSpectrumChannel> ();
      propagation = CreateObject<MmWave3gppPropagationLossModel>();
      propagation->SetAttributeFailSafe ("Frequency", DoubleValue (freq));
      channel->AddPropagationLossModel (propagation);

      threegppChannel = CreateObject<MmWave3gppChannel> ();
      threegppChannel->SetPathlossModel (propagation);
      threegppChannel->SetAttribute ("CenterFrequency", DoubleValue (freq));

      channel->AddSpectrumPropagationLossModel (threegppChannel);
    }
  else if (inputSpectrumChannel != nullptr && inputPathloss != nullptr && inputMmWaveChannel != nullptr)
    {
      // already configured
      channel = inputSpectrumChannel;
      propagation = inputPathloss;
      threegppChannel = inputMmWaveChannel;
    }
  else
    {
      NS_FATAL_ERROR ("Conf not supported");
    }

  SpectrumWifiPhyHelper apSpectrumWifiPhy = DmgWifiPhyHelper::DefaultSpectrum ();
  apSpectrumWifiPhy.SetChannel (channel);
  apSpectrumWifiPhy.Set ("TxPowerStart", DoubleValue (apTxPower));
  apSpectrumWifiPhy.Set ("TxPowerEnd", DoubleValue (apTxPower));
  apSpectrumWifiPhy.Set ("TxPowerLevels", UintegerValue (1));
  apSpectrumWifiPhy.Set ("TxGain", DoubleValue (0));
  apSpectrumWifiPhy.Set ("RxGain", DoubleValue (0));
  /* Sensitivity model includes implementation loss and noise figure */
  apSpectrumWifiPhy.Set ("RxNoiseFigure", DoubleValue (7));
  apSpectrumWifiPhy.Set ("CcaMode1Threshold", DoubleValue (ccaThresholdAp));
  //spectrumWifiPhy.Set ("EnergyDetectionThreshold", DoubleValue (-79 + 3));
  apSpectrumWifiPhy.Set ("ChannelWidth", UintegerValue (static_cast<uint16_t> (bw / 1e6)));
  /* Set the phy layer error model */
  apSpectrumWifiPhy.SetErrorRateModel ("ns3::SensitivityModel60GHz");
  /* Set default algorithm for all nodes to be constant rate */
  //wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "ControlMode", StringValue (phyMode),
  //                              "DataMode", StringValue (phyMode));

  SpectrumWifiPhyHelper staSpectrumWifiPhy = DmgWifiPhyHelper::DefaultSpectrum ();
  staSpectrumWifiPhy.SetChannel (channel);
  staSpectrumWifiPhy.Set ("TxPowerStart", DoubleValue (staTxPower));
  staSpectrumWifiPhy.Set ("TxPowerEnd", DoubleValue (staTxPower));
  staSpectrumWifiPhy.Set ("TxPowerLevels", UintegerValue (1));
  staSpectrumWifiPhy.Set ("TxGain", DoubleValue (0));
  staSpectrumWifiPhy.Set ("RxGain", DoubleValue (0));
  /* Sensitivity model includes implementation loss and noise figure */
  staSpectrumWifiPhy.Set ("RxNoiseFigure", DoubleValue (7));
  staSpectrumWifiPhy.Set ("CcaMode1Threshold", DoubleValue (ccaThresholdSta));
  //staSpectrumWifiPhy.Set ("EnergyDetectionThreshold", DoubleValue (-79 + 3));
  staSpectrumWifiPhy.Set ("ChannelWidth", UintegerValue (static_cast<uint16_t> (bw / 1e6)));
  /* Set the phy layer error model */
  staSpectrumWifiPhy.SetErrorRateModel ("ns3::SensitivityModel60GHz");


  wifi.SetRemoteStationManager ("ns3::IdealWifiManager");

  /**** Allocate DMG Wifi MAC ****/
  DmgWifiMacHelper wifiMac = DmgWifiMacHelper::Default ();

  wifiMac.SetType ("ns3::DmgApWifiMac",
                   "Ssid", SsidValue (Ssid (ssID)),
                   "BE_MaxAmpduSize", UintegerValue (262143), //Enable A-MPDU with the highest maximum size allowed by the standard
                   "BE_MaxAmsduSize", UintegerValue (7935),   //Enable A-MSDU with the highest maximum size (in Bytes) allowed by the standard
                   //"BE_MaxAmsduSize", UintegerValue (0),
                   "SSSlotsPerABFT", UintegerValue (8),
                   "SSFramesPerSlot", UintegerValue (16),
                   "AnnounceCapabilities", BooleanValue (false),
                   "ScheduleElement", BooleanValue (false),
                   "BeaconInterval", TimeValue (MicroSeconds (102400)),
                   "BeaconTransmissionInterval", TimeValue (MicroSeconds (1200)),
                   "ATIDuration", TimeValue (MicroSeconds (0)),
                   "ATIPresent", BooleanValue (false));

  /* Set codebook for the AP */
  wifi.SetCodebook ("ns3::CodebookAntennaArray",
                    "AntennasDim1", UintegerValue (apAntennaDim1),
                    "AntennasDim2", UintegerValue (apAntennaDim2),
                    "BeamAngleStep", DoubleValue (30));

  for (auto it = GetGnbNodes ().Begin (); it != GetGnbNodes ().End (); ++it)
    {
      m_gnbDev.Add (wifi.Install (apSpectrumWifiPhy, wifiMac, (*it)));
    }


  wifiMac.SetType ("ns3::DmgStaWifiMac",
                   "Ssid", SsidValue (Ssid (ssID)),
                   "ActiveProbing", BooleanValue (false),
                   "BE_MaxAmpduSize", UintegerValue (262143),   //Enable A-MPDU with the highest maximum size allowed by the standard
                   "BE_MaxAmsduSize", UintegerValue (7935));   //Enable A-MSDU with the highest maximum size (in Bytes) allowed by the standard
                   //"BE_MaxAmsduSize", UintegerValue (0));


  /* Set codebook for the STA */
  wifi.SetCodebook ("ns3::CodebookAntennaArray",
                    "AntennasDim1", UintegerValue (staAntennaDim1),
                    "AntennasDim2", UintegerValue (staAntennaDim2),
                    "BeamAngleStep", DoubleValue (30));

  for (auto it = GetUeNodes ().Begin (); it != GetUeNodes ().End (); ++it)
    {
      m_ueDev.Add (wifi.Install (staSpectrumWifiPhy, wifiMac, *it));
    }

  for (uint32_t i = 0; i < m_gnbDev.GetN (); ++i)
    {
      auto wndAp =  DynamicCast<WifiNetDevice> (m_gnbDev.Get (i));
      auto apWifiMac = StaticCast<DmgApWifiMac> (wndAp->GetMac ());
      auto apWifiPhy = DynamicCast<SpectrumWifiPhy> (wndAp->GetPhy ());
      apWifiMac->TraceConnectWithoutContext ("SLSCompleted", MakeBoundCallback (&SLSCompleted, apWifiMac));

      apWifiMac->TraceConnectWithoutContext ("BIStarted", MakeBoundCallback (&BIStarted, apWifiMac));

      apWifiMac->TraceConnectWithoutContext ("DTIStarted", MakeBoundCallback (&DTIStarted, apWifiMac));


      apWifiMac->TraceConnectWithoutContext ("BeaconSent", MakeBoundCallback (&BeaconSent, apWifiMac));

      apWifiMac->TraceConnectWithoutContext ("ContentionPeriodStarted", MakeBoundCallback (&ContentionPeriodStarted, apWifiMac));

      apWifiMac->TraceConnectWithoutContext ("SSWReceivedFromResponder", MakeBoundCallback (&SSWReceivedFromResponder, apWifiMac));
      apWifiMac->TraceConnectWithoutContext ("SSWReceivedFromInitiator", MakeBoundCallback (&SSWReceivedFromInitiator, apWifiMac));

      apWifiMac->TraceConnectWithoutContext ("ABFTStarted", MakeBoundCallback (&ABFTStarted, apWifiMac));
      apWifiMac->TraceConnectWithoutContext ("SSSlotStarted", MakeBoundCallback (&SSSlotStarted, apWifiMac));


      apWifiMac->TraceConnectWithoutContext ("InitiatorSSWSent", MakeBoundCallback (&InitiatorSSWSent, apWifiMac));


      apWifiMac->TraceConnectWithoutContext ("ResponderSSWSent", MakeBoundCallback (&ResponderSSWSent, apWifiMac));

      apWifiMac->TraceConnectWithoutContext ("DataReceived", MakeBoundCallback (&DataReceived, apWifiMac));

      std::stringstream nodeId;
      nodeId << wndAp->GetNode ()->GetId ();
      apWifiPhy->TraceConnect ("PhyRxDrop", nodeId.str (),
                               MakeCallback (&WigigSetup::PhyRxDrop, this));
      apWifiPhy->TraceConnect ("PhyTxDrop", nodeId.str (),
                               MakeCallback (&WigigSetup::PhyTxDrop, this));
      apWifiPhy->TraceConnect ("TxDataTrace", nodeId.str (),
                               MakeCallback (&WigigSetup::TxDataTrace, this));
      apWifiPhy->TraceConnect ("RxDataTrace", nodeId.str (),
                               MakeCallback (&WigigSetup::RxDataTrace, this));

      apWifiMac->GetDcfManager ()->TraceConnect ("ChannelRequestToAccessDuration", nodeId.str (),
                                                 MakeCallback (&WigigSetup::ChannelRequestToAccessDuration, this));
    }

  for (uint32_t i = 0; i < m_ueDev.GetN (); ++i)
    {
      auto wndSta1 =  DynamicCast<WifiNetDevice> (m_ueDev.Get (i));
      auto staWifiMac1 = StaticCast<DmgStaWifiMac> (wndSta1->GetMac ());
      auto staWifiPhy = DynamicCast<SpectrumWifiPhy> (wndSta1->GetPhy ());

      staWifiMac1->TraceConnectWithoutContext ("SLSCompleted", MakeBoundCallback (&SLSCompleted, staWifiMac1));

      staWifiMac1->TraceConnectWithoutContext ("Assoc", MakeBoundCallback (&StationAssociated, staWifiMac1));

      staWifiMac1->TraceConnectWithoutContext ("BeaconArrival", MakeBoundCallback (&BeaconArrival, staWifiMac1));

      staWifiMac1->TraceConnectWithoutContext ("ContentionPeriodStarted", MakeBoundCallback (&ContentionPeriodStarted, staWifiMac1));


      staWifiMac1->TraceConnectWithoutContext ("SSWReceivedFromResponder", MakeBoundCallback (&SSWReceivedFromResponder, staWifiMac1));


      staWifiMac1->TraceConnectWithoutContext ("SSWReceivedFromInitiator", MakeBoundCallback (&SSWReceivedFromInitiator, staWifiMac1));
      staWifiMac1->TraceConnectWithoutContext ("ResponderSSWSent", MakeBoundCallback (&ResponderSSWSent, staWifiMac1));
      staWifiMac1->TraceConnectWithoutContext ("InitiatorSSWSent", MakeBoundCallback (&InitiatorSSWSent, staWifiMac1));
      staWifiMac1->TraceConnectWithoutContext ("DataReceived", MakeBoundCallback (&DataReceived, staWifiMac1));

      std::stringstream nodeId;
      nodeId << wndSta1->GetNode ()->GetId ();
      staWifiPhy->TraceConnect ("PhyRxDrop", nodeId.str (),
                                MakeCallback (&WigigSetup::PhyRxDrop, this));
      staWifiPhy->TraceConnect ("PhyTxDrop", nodeId.str (),
                                MakeCallback (&WigigSetup::PhyTxDrop, this));
      staWifiPhy->TraceConnect ("TxDataTrace", nodeId.str (),
                                MakeCallback (&WigigSetup::TxDataTrace, this));
      staWifiPhy->TraceConnect ("RxDataTrace", nodeId.str (),
                                MakeCallback (&WigigSetup::RxDataTrace, this));
    }

  // registering AP devices to 3gpp model
  for (uint32_t i = 0; i < m_gnbDev.GetN (); ++i)
    {
      // AP and its codebook
      Ptr<WifiNetDevice> wndAp =  DynamicCast<WifiNetDevice> (m_gnbDev.Get (i));
      Ptr<CodebookAntennaArray> codebookAp = DynamicCast<CodebookAntennaArray> (DynamicCast<DmgWifiMac> (wndAp->GetMac ())->GetCodebook ());
      threegppChannel->RegisterDevicesAntennaArray (wndAp, codebookAp->GetAntennaArray ());
    }

  // registering STAs devices to 3gpp model
  for (uint32_t i = 0; i < m_ueDev.GetN (); ++i)
    {
      Ptr<WifiNetDevice> wndSta1 =  DynamicCast<WifiNetDevice> (m_ueDev.Get (i));
      Ptr<CodebookAntennaArray> codebookSta1 = DynamicCast<CodebookAntennaArray> (DynamicCast<DmgWifiMac> (wndSta1->GetMac ())->GetCodebook ());
      threegppChannel->RegisterDevicesAntennaArray (wndSta1, codebookSta1->GetAntennaArray (), true);
    }
}

WigigSetup::~WigigSetup ()
{

}

Ipv4InterfaceContainer
WigigSetup::AssignIpv4ToUe (const std::unique_ptr<Ipv4AddressHelper> &address) const
{
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
WigigSetup::AssignIpv4ToStations (const std::unique_ptr<Ipv4AddressHelper> &address) const
{
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  InternetStackHelper stack;
  stack.Install (GetGnbNodes ());
  Ipv4InterfaceContainer apInterface;
  apInterface = address->Assign (GetGnbDev ());

  return apInterface;
}

Ipv4InterfaceContainer
WigigSetup::ConnectToRemotes (const NodeContainer &remoteHosts, const std::string &base)
{
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
WigigSetup::SetChannelRTACallback(const ChannelRequestToAccessDurationCb &cb)
{
  m_craCb = cb;
}

void
WigigSetup::PhyRxDrop (std::string context, Ptr<const Packet> p)
{
  if (!m_macTxDataFailedCb.IsNull ())
    {
      m_macTxDataFailedCb (static_cast<uint32_t> (std::stoul (context)), p->GetSize ());
    }
}

void
WigigSetup::PhyTxDrop (std::string context, Ptr<const Packet> p)
{
  if (!m_macTxDataFailedCb.IsNull ())
    {
      m_macTxDataFailedCb (static_cast<uint32_t> (std::stoul (context)), p->GetSize ());
    }
}

void WigigSetup::TxDataTrace (std::string context, Time t)
{
  if (!m_channelOccupancyTimeCb.IsNull ())
    {
      m_channelOccupancyTimeCb (static_cast<uint32_t> (std::stoul (context)), t);
    }
}

void WigigSetup::RxDataTrace (std::string context, double sinr)
{
  if (!m_sinrCb.IsNull ())
    {
      m_sinrCb (static_cast<uint32_t> (std::stoul (context)), sinr);
    }
}

void
WigigSetup::ChannelRequestToAccessDuration (std::string context, Time oldValue, Time newValue)
{
  if (!m_craCb.IsNull ())
    {
      m_craCb (static_cast<uint32_t> (std::stoul (context)), newValue);
    }
}

} // namespace ns3
