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

#include "simulation-helper.h"
#include <ns3/log.h>
#include <ns3/nr-u-module.h>
#include <ns3/nr-module.h>
#include <ns3/wifi-80211ad-nist-module.h>
#include <ns3/internet-module.h>
#include <ns3/flow-monitor-module.h>
#include <ns3/multi-model-spectrum-channel.h>
#include <ns3/applications-module.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("CttcNrWigigInterferenceExample");

static const uint32_t PACKET_SIZE = 1000;

static void
ConfigureDefaultValues (bool cellScan = true, double beamSearchAngleStep = 10.0,
                        uint32_t eesmTable = 1,
                        const std::string &errorModel = "ns3::NrEesmErrorModel",
                        double cat2EDThreshold = - 69.0, double cat3and4EDThreshold = -79.0,
                        const std::string & rlcModel = "RlcTmAlways")
{
  Config::SetDefault ("ns3::MmWave3gppPropagationLossModel::ChannelCondition",
                      StringValue("l"));
  Config::SetDefault ("ns3::MmWave3gppPropagationLossModel::Scenario",
                      StringValue("InH-OfficeMixed"));
  Config::SetDefault ("ns3::MmWave3gppPropagationLossModel::Shadowing",
                      BooleanValue(false));

  Config::SetDefault ("ns3::MmWave3gppChannel::CellScan",
                      BooleanValue(cellScan));
  Config::SetDefault ("ns3::MmWave3gppChannel::UpdatePeriod",
                      TimeValue(MilliSeconds(0)));
  Config::SetDefault ("ns3::MmWave3gppChannel::BeamSearchAngleStep",
                      DoubleValue(beamSearchAngleStep));

  Config::SetDefault ("ns3::MmWaveEnbPhy::AntennaNumDim1", UintegerValue (8));
  Config::SetDefault ("ns3::MmWaveEnbPhy::AntennaNumDim2", UintegerValue (8));

  Config::SetDefault ("ns3::MmWaveUePhy::AntennaNumDim1", UintegerValue (4));
  Config::SetDefault ("ns3::MmWaveUePhy::AntennaNumDim2", UintegerValue (4));

  // gNB noise figure shall be set to 7 dB
  Config::SetDefault("ns3::MmWaveEnbPhy::NoiseFigure", DoubleValue (7));
  // UE noise figure shall be set to 7 dB
  Config::SetDefault("ns3::MmWaveUePhy::NoiseFigure", DoubleValue (7));

  Config::SetDefault("ns3::AntennaArrayModel::AntennaOrientation", EnumValue (AntennaArrayModel::X0));

  Config::SetDefault ("ns3::MmWaveSpectrumPhy::UnlicensedMode", BooleanValue (false));

  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize",
                      UintegerValue(999999999));
  Config::SetDefault ("ns3::LteRlcTm::MaxTxBufferSize",
                      UintegerValue (999999999));
  Config::SetDefault ("ns3::MmWaveHelper::NumberOfComponentCarriers", UintegerValue (1));

  Config::SetDefault("ns3::PointToPointEpcHelper::S1uLinkDelay", TimeValue (MilliSeconds(0)));
  Config::SetDefault("ns3::PointToPointEpcHelper::X2LinkDelay", TimeValue (MilliSeconds(0)));
  Config::SetDefault("ns3::LteEnbRrc::EpsBearerToRlcMapping",  StringValue (rlcModel));

  if (eesmTable == 1)
    {
      Config::SetDefault("ns3::NrEesmErrorModel::McsTable", EnumValue (NrEesmErrorModel::McsTable1));
    }
  else if (eesmTable == 2)
    {
      Config::SetDefault("ns3::NrEesmErrorModel::McsTable", EnumValue (NrEesmErrorModel::McsTable2));
    }
  else
    {
      NS_FATAL_ERROR ("Valid tables are 1 or 2, you set " << eesmTable);
    }

  Config::SetDefault("ns3::NrAmc::ErrorModelType", TypeIdValue (TypeId::LookupByName(errorModel)));
  Config::SetDefault("ns3::NrAmc::AmcModel", EnumValue (NrAmc::ShannonModel));
  Config::SetDefault("ns3::MmWaveSpectrumPhy::ErrorModelType", TypeIdValue (TypeId::LookupByName(errorModel)));

  /* Global params: no fragmentation, no RTS/CTS, fixed rate for all packets */
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("999999"));
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("999999"));

  Config::SetDefault ("ns3::DmgWifiMac::BeamReciprocity", BooleanValue (true));
  Config::SetDefault ("ns3::DmgApWifiMac::EnableBeaconIntervalJitter", BooleanValue (true));

  // Cat3 and Cat 4 ED treshold, Cat2 has its own attribute, maybe in future each could have its own
  Config::SetDefault ("ns3::NrLbtAccessManager::EnergyDetectionThreshold", DoubleValue (cat3and4EDThreshold));
  // Cat2 ED threshold
  Config::SetDefault ("ns3::NrCat2LbtAccessManager::Cat2EDThreshold", DoubleValue (cat2EDThreshold));
}



static void TimePasses ()
{
  time_t t = time (nullptr);
  struct tm tm = *localtime (&t);

  std::cout << "Simulation Time: " << Simulator::Now ().GetSeconds () << " real time: "
            << tm.tm_hour << "h, " << tm.tm_min << "m, " << tm.tm_sec << "s." << std::endl;
  Simulator::Schedule (MilliSeconds (100), &TimePasses);
}

static bool m_nrIsOccupying = false;
static bool m_wigigIsOccupying = false;
static Time m_nrOccupancy;
static Time m_wigigOccupancy;
static OutputManager *outputManager;

static void ResetNrOccupancy ()
{
  m_nrIsOccupying = false;
}

static void ResetWigigOccupancy ()
{
  m_wigigIsOccupying = false;
}

static void
NrOccupancy (uint32_t nodeId, const Time & time)
{
  outputManager->UidIsTxing (nodeId);
  if (m_wigigIsOccupying)
    {
      outputManager->SimultaneousTxOtherTechnology (nodeId);
    }

  if (m_nrIsOccupying)
    {
      outputManager->SimultaneousTxSameTechnology (nodeId);
    }

  m_nrOccupancy += time;
  m_nrIsOccupying = true;
  Simulator::Schedule (time, &ResetNrOccupancy);
}

static void
WigigOccupancy (uint32_t nodeId, const Time & time)
{
  outputManager->UidIsTxing (nodeId);
  if (m_nrIsOccupying)
    {
      outputManager->SimultaneousTxOtherTechnology (nodeId);
    }
  if (m_wigigIsOccupying)
    {
      outputManager->SimultaneousTxSameTechnology (nodeId);
    }

  m_wigigOccupancy += time;
  m_wigigIsOccupying = true;
  Simulator::Schedule (time, &ResetWigigOccupancy);
}

int
main (int argc, char *argv[])
{
  bool cellScan = true;
  double beamSearchAngleStep = 30.0;
  double totalTxPower = 4;
  double ueTxPower = 2;
  uint16_t numerologyBwp1 = 3;
  double frequencyBwp1 = 58.32e9;
  double bandwidthBwp1 = 2160e6;
  double ueX = 5.0;

  double simTime = 1.5; // seconds
  double udpAppStartTime = 0.5; //seconds
  uint32_t scenarioId = 0;
  uint32_t runId = 0;
  uint32_t seed = 1;
  bool enableNr = false;
  bool enableWigig = false;
  bool doubleTechnology = false;
  bool allLink = true;
  double cat2EDThreshold = -69.0;
  double cat3and4EDThreshold = -79.0;

  std::string rlcModel = "RlcUmAlways";
  std::string errorModel = "ns3::NrLteMiErrorModel";
  uint32_t eesmTable = 1;
  std::string nodeRate = "500kbps";
  std::string gnbCamType = "ns3::NrAlwaysOnAccessManager";
  std::string ueCamType = "ns3::NrAlwaysOnAccessManager";

  CommandLine cmd;

  cmd.AddValue ("simTime", "Simulation time", simTime);
  cmd.AddValue ("cellScan",
                "Use beam search method to determine beamforming vector,"
                " the default is long-term covariance matrix method"
                " true to use cell scanning method, false to use the default"
                " power method.",
                cellScan);
  cmd.AddValue ("beamSearchAngleStep",
                "Beam search angle step for beam search method",
                beamSearchAngleStep);
  cmd.AddValue ("totalTxPower",
                "total tx power that will be proportionally assigned to"
                " bandwidth parts depending on each BWP bandwidth ",
                totalTxPower);
  cmd.AddValue("errorModelType",
               "Error model type: ns3::NrEesmErrorModel , ns3::NrLteErrorModel",
               errorModel);
  cmd.AddValue("eesmTable",
               "Table to use when error model is Eesm (1 for McsTable1 or 2 for McsTable2)",
               eesmTable);
  cmd.AddValue ("rlcModel", "The NR RLC Model: RlcTmAlways or RlcUmAlways", rlcModel);
  cmd.AddValue("ueX",
               "X position of any UE",
               ueX);
  cmd.AddValue("scenario",
               "Scenario (0 = simple interference, 1 = new position",
               scenarioId);
  cmd.AddValue ("seed", "Simulation seed", seed);
  cmd.AddValue ("runId", "Simulation Run ID", runId);
  cmd.AddValue ("enableNr", "enable nr node", enableNr);
  cmd.AddValue ("enableWigig", "Enable Wigig nodes", enableWigig);
  cmd.AddValue ("nodeRate", "The rate of every node in the network", nodeRate);
  cmd.AddValue ("gnbCamType", "The GNB CAM", gnbCamType);
  cmd.AddValue ("ueCamType", "The UE CAM", ueCamType);
  cmd.AddValue ("doubleTechnology", "Double the technology", doubleTechnology);
  cmd.AddValue("allLink", "Interference with all links?", allLink);
  cmd.AddValue ("cat2EDThreshold", "The ED threshold to be used by Lbt category 2 algorithm [dBm]. Allowed range [-100.0, 0.0]., ", cat2EDThreshold);
  cmd.AddValue ("cat3and4EDTreshold", "The ED threshold to be used by Lbt category 3 and 4 algorithm [dBm]. Allowed range [-100.0, 0.0].", cat3and4EDThreshold);

  cmd.Parse (argc, argv);

  RngSeedManager::SetSeed (seed);
  RngSeedManager::SetRun (runId);
  ConfigureDefaultValues (cellScan, beamSearchAngleStep, eesmTable, errorModel, cat2EDThreshold, cat3and4EDThreshold, rlcModel);
  if (allLink)
    {
      Config::SetDefault ("ns3::MmWave3gppChannel::EnableAllChannels", BooleanValue (true));
      Config::SetDefault ("ns3::MmWaveSpectrumPhy::EnableAllInterferences", BooleanValue (true));
    }

  NodeDistributionScenario *scenario;

  // Place of the node inside the UE/STA container
  uint32_t nrNodePlace = 0;
  uint32_t wigigNodePlace = 0;

  NS_ASSERT (! (enableNr && enableWigig && doubleTechnology));

  uint32_t nodes = 0;
  if (enableNr)
    {
      ++nodes;
      wigigNodePlace = 1;
    }
  if (enableWigig)
    {
      ++nodes;
    }

  if ((enableNr && doubleTechnology))
    {
      ++nodes;
      wigigNodePlace = 1;
    }
  else if (enableWigig && doubleTechnology)
    {
      ++nodes;
      nrNodePlace = 1;
    }

  if (scenarioId == 0)
    {
      scenario = new SinglePairNodeScenario (nodes, Vector (0, 0, 1.5), ueX);
    }
  else if (scenarioId == 1)
    {
      NS_ASSERT (doubleTechnology || (enableNr && enableWigig));
      scenario = new InFrontNodeDistribution (Vector (0, 0, 1.5), 10.0, ueX);
    }
  else
    {
      NS_FATAL_ERROR ("Scenario no recognized");
    }

  std::stringstream ss;
  std::string technology = "";
  if (enableNr)
    {
      technology += "with-nr-";
    }
  else
    {
      technology += "without-nr-";
    }
  if (enableWigig)
    {
      technology += "with-wigig-";
    }
  else
    {
      technology += "without-wigig-";
    }

  if (doubleTechnology)
    {
      technology += "doubled-";
    }
  std::string link;
  if (allLink)
    {
      link = "allLink-";
    }
  else
    {
      link = "noAllLink-";
    }

  Packet::EnablePrinting();

  ss << "cttc-nr-wigig-interference-example-" << scenarioId << "-" << technology;
  ss << link << gnbCamType << "-" << ueCamType << "-" << nodeRate << "-" << rlcModel << ".db";

  SqliteOutputManager manager (ss.str(), ss.str (), ueX, seed, runId);
  outputManager = &manager;

  L2Setup *nr, *wigig;

  Ptr<MultiModelSpectrumChannel> channel = CreateObject<MultiModelSpectrumChannel> ();
  Ptr<PropagationLossModel> propagation= CreateObject<MmWave3gppPropagationLossModel>();
  Ptr<MmWave3gppChannel> threegppChannel = CreateObject<MmWave3gppChannel>();

  propagation->SetAttributeFailSafe("Frequency", DoubleValue(frequencyBwp1));
  channel->AddPropagationLossModel (propagation);

  threegppChannel->SetPathlossModel (propagation);
  threegppChannel->SetAttribute ("CenterFrequency", DoubleValue (frequencyBwp1));

  channel->AddSpectrumPropagationLossModel (threegppChannel);

  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  Ipv4InterfaceContainer ueIpIface;

  if (enableNr)
    {
      std::unique_ptr<Ipv4AddressHelper> address = std::unique_ptr<Ipv4AddressHelper> (new Ipv4AddressHelper ());
      NodeContainer gnbs;
      NodeContainer ues;
      std::unordered_map<uint32_t, uint32_t> connections;

      if (doubleTechnology)
        {
          gnbs = NodeContainer (scenario->GetGnbs());
          ues = NodeContainer (scenario->GetUes());
          connections = { { 0, 0}, {1, 1}};
        }
      else
        {
          gnbs = NodeContainer (scenario->GetGnbs().Get(nrNodePlace));
          ues = NodeContainer (scenario->GetUes().Get(nrNodePlace));
          connections = { { 0, 0} };
        }

      nr = new NrSingleBwpSetup (gnbs,ues, channel, propagation, threegppChannel,
                                 frequencyBwp1, bandwidthBwp1, numerologyBwp1, totalTxPower, ueTxPower, connections,
                                 gnbCamType, ueCamType, "ns3::MmWaveMacSchedulerTdmaPF");
      ueIpIface.Add (nr->AssignIpv4ToUe(address));
      nr->AssignIpv4ToStations(address); // Not used
      nr->ConnectToRemotes (remoteHostContainer, "1.0.0.0");
      nr->SetSinrCallback (MakeCallback (&OutputManager::SinrStore, &manager));
      nr->SetMacTxDataFailedCb(MakeCallback (&OutputManager::MacDataTxFailed, &manager));
      nr->SetChannelOccupancyCallback (MakeCallback (&NrOccupancy));
    }

  if (enableWigig)
    {
      std::unique_ptr<Ipv4AddressHelper> address = std::unique_ptr<Ipv4AddressHelper> (new Ipv4AddressHelper ());
      address->SetBase("10.0.0.0", "255.255.255.0");
      wigig = new WigigSetup (NodeContainer (scenario->GetGnbs().Get(wigigNodePlace)),
                              NodeContainer (scenario->GetUes().Get(wigigNodePlace)),
                              channel, propagation, threegppChannel,
                              frequencyBwp1, bandwidthBwp1, totalTxPower, totalTxPower, -79.0, -79.0,
                              "primero");
      ueIpIface.Add (wigig->AssignIpv4ToUe(address));
      wigig->AssignIpv4ToStations(address);
      wigig->ConnectToRemotes(remoteHostContainer, "2.0.0.0");
      wigig->SetSinrCallback (MakeCallback (&OutputManager::SinrStore, &manager));
      wigig->SetMacTxDataFailedCb(MakeCallback (&OutputManager::MacDataTxFailed, &manager));
      wigig->SetChannelOccupancyCallback (MakeCallback (&WigigOccupancy));
      dynamic_cast<WigigSetup*> (wigig)->SetChannelRTACallback (MakeCallback (&OutputManager::ChannelRequestTime, &manager));

      if (doubleTechnology)
        {
          address->SetBase("10.0.1.0", "255.255.255.0");
          auto secondWigig = new WigigSetup (NodeContainer (scenario->GetGnbs().Get(nrNodePlace)),
                                             NodeContainer (scenario->GetUes().Get(nrNodePlace)),
                                             channel, propagation, threegppChannel,
                                             frequencyBwp1, bandwidthBwp1, totalTxPower, totalTxPower,
                                             -79.0, -79.0, "segundo");
          ueIpIface.Add (secondWigig->AssignIpv4ToUe(address));
          secondWigig->AssignIpv4ToStations(address);
          secondWigig->ConnectToRemotes(remoteHostContainer, "2.1.0.0");
          secondWigig->SetSinrCallback (MakeCallback (&OutputManager::SinrStore, &manager));
          secondWigig->SetMacTxDataFailedCb(MakeCallback (&OutputManager::MacDataTxFailed, &manager));
          secondWigig->SetChannelOccupancyCallback (MakeCallback (&WigigOccupancy));
          dynamic_cast<WigigSetup*> (secondWigig)->SetChannelRTACallback (MakeCallback (&OutputManager::ChannelRequestTime, &manager));
        }
    }

  uint32_t ifId = 1;
  if (enableNr)
    {
      Ipv4StaticRoutingHelper ipv4RoutingHelper;
      for (auto it = remoteHostContainer.Begin(); it != remoteHostContainer.End(); ++it)
        {
          Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting ((*it)->GetObject<Ipv4> ());
          remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), ifId);
        }
      ifId++;
    }

  if (enableWigig)
    {
      Ipv4StaticRoutingHelper ipv4RoutingHelper;
      for (auto it = remoteHostContainer.Begin(); it != remoteHostContainer.End(); ++it)
        {
          Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting ((*it)->GetObject<Ipv4> ());
          remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("10.0.0.0"), Ipv4Mask ("255.255.255.0"), ifId);
          ifId++;
          if (doubleTechnology)
            {
              remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("10.0.1.0"), Ipv4Mask ("255.255.255.0"), ifId);
            }
        }
    }

  uint16_t dlPort = 1234;
  ApplicationContainer clientApps, serverApps;

  PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory",
                                       Address (InetSocketAddress (Ipv4Address::GetAny (), dlPort)));
  dlPacketSinkHelper.SetAttribute("EnableE2EStats", BooleanValue (true));

  if (enableNr)
    {
      serverApps.Add (dlPacketSinkHelper.Install (scenario->GetUes().Get (nrNodePlace)));
      if (doubleTechnology)
        {
          serverApps.Add (dlPacketSinkHelper.Install (scenario->GetUes().Get (wigigNodePlace)));
        }
    }
  if (enableWigig)
    {
      serverApps.Add (dlPacketSinkHelper.Install (scenario->GetUes ().Get (wigigNodePlace)));
      if (doubleTechnology)
        {
          serverApps.Add (dlPacketSinkHelper.Install (scenario->GetUes().Get (nrNodePlace)));
        }
    }

  serverApps.Start (Seconds (udpAppStartTime));

  for (uint32_t j = 0; j < scenario->GetUes().GetN(); ++j)
    {
      OnOffHelper onoff ("ns3::UdpSocketFactory",
                         Address (InetSocketAddress (ueIpIface.GetAddress (j), dlPort)));
      onoff.SetConstantRate (DataRate (nodeRate), PACKET_SIZE);
      onoff.SetAttribute("EnableE2EStats", BooleanValue (true));
      clientApps.Add (onoff.Install (remoteHost));
    }

  if (enableNr)
    {
      clientApps.Get(nrNodePlace)->SetStartTime (Seconds (udpAppStartTime));
      if (doubleTechnology)
        {
          clientApps.Get(wigigNodePlace)->SetStartTime (Seconds (udpAppStartTime));
        }
    }

  if (enableWigig)
    {
      clientApps.Get (wigigNodePlace)->SetStartTime (Seconds (udpAppStartTime)); // Manual HaCK
      if (doubleTechnology)
        {
          clientApps.Get(nrNodePlace)->SetStartTime (Seconds (udpAppStartTime));
        }
    }

  serverApps.Stop (Seconds (simTime));
  clientApps.Stop (Seconds (simTime));

  PopulateArpCache ();

  std::cout << "UE" << std::endl;
  PrintIpAddress (scenario->GetUes());
  PrintRoutingTable (scenario->GetUes());
  std::cout << "GNB" << std::endl;
  PrintIpAddress (scenario->GetGnbs());
  PrintRoutingTable (scenario->GetGnbs());
  std::cout << "REMOTE" << std::endl;
  PrintIpAddress (remoteHostContainer);
  PrintRoutingTable (remoteHostContainer);

  Simulator::Schedule (MicroSeconds (100), &TimePasses);

  // Flow monitor
  FlowMonitorHelper flowHelper;
  auto monitor = flowHelper.InstallAll ();

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();

  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowHelper.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
    {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
      bool isNr = ((t.sourceAddress.Get() >> 24) & 0xff) == 1;
      bool isWigig = ((t.sourceAddress.Get() >> 24) & 0xff) != 1;
      if (isNr || isWigig)
        {
          std::string technology = isNr ? "nr" : "wigig";
          double thMbps = i->second.rxBytes * 8.0 / (simTime - udpAppStartTime) / 1e6;
          double delay = 0.0;
          double jitter = 0.0;
          if (i->second.rxPackets > 1)
            {
              delay = i->second.delaySum.GetMicroSeconds () / i->second.rxPackets;
              jitter = i->second.jitterSum.GetMicroSeconds () / (i->second.rxPackets - 1);
            }
          std::stringstream addr;
          t.destinationAddress.Print(addr);
          if (i->second.txBytes > 300)
            {
              manager.StoreE2EStatsFor(technology, thMbps, i->second.txBytes, i->second.rxBytes,
                                       delay, jitter, addr.str ());
            }
        }
    }

  if (enableNr)
    {
      manager.StoreChannelOccupancyRateFor ("nr", m_nrOccupancy.GetSeconds() / (simTime - udpAppStartTime));
    }

  if (enableWigig)
    {
      manager.StoreChannelOccupancyRateFor ("wigig", m_wigigOccupancy.GetSeconds() / (simTime - udpAppStartTime));
    }

  manager.Close ();


  Simulator::Destroy ();
  return 0;
}

