/* Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; */
/*
 *   Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *   Copyright (c) 2021 Orange Labs
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
#include <ns3/wifi-module.h>
#include <ns3/internet-module.h>
#include <ns3/flow-monitor-module.h>
#include <ns3/multi-model-spectrum-channel.h>
#include <ns3/applications-module.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("CttcNrWifiInterferenceExample");

static const uint32_t PACKET_SIZE = 1000;

static void
ConfigureDefaultValues (bool cellScan = true, double beamSearchAngleStep = 10.0,
                        const std::string &errorModel = "ns3::NrEesmErrorModel",
                        double cat2EDThreshold = -69.0, double cat3and4EDThreshold = -79.0,
                        const std::string & rlcModel = "RlcTmAlways")
{
  Config::SetDefault ("ns3::ThreeGppPropagationLossModel::ShadowingEnabled",
                      BooleanValue (false));

  if (cellScan)
    {
      Config::SetDefault ("ns3::IdealBeamformingHelper::BeamformingMethod", TypeIdValue (CellScanBeamforming::GetTypeId ()));
    }
  else
    {
      Config::SetDefault ("ns3::IdealBeamformingHelper::BeamformingMethod", TypeIdValue (DirectPathBeamforming::GetTypeId ()));
    }
  Config::SetDefault ("ns3::IdealBeamformingHelper::BeamformingPeriodicity",
                      TimeValue (MilliSeconds (1000))); //seldom updated
  Config::SetDefault ("ns3::CellScanBeamforming::BeamSearchAngleStep",
                      DoubleValue (beamSearchAngleStep));

  Config::SetDefault ("ns3::UniformPlanarArray::NumColumns", UintegerValue (2));
  Config::SetDefault ("ns3::UniformPlanarArray::NumRows", UintegerValue (2));

  Config::SetDefault ("ns3::NrGnbPhy::NoiseFigure", DoubleValue (7));
  Config::SetDefault ("ns3::NrUePhy::NoiseFigure", DoubleValue (7));
  Config::SetDefault ("ns3::WifiPhy::RxNoiseFigure", DoubleValue (7));

  Config::SetDefault ("ns3::IsotropicAntennaModel::Gain", DoubleValue (0));
  Config::SetDefault ("ns3::WifiPhy::TxGain", DoubleValue (0));
  Config::SetDefault ("ns3::WifiPhy::RxGain", DoubleValue (0));

  Config::SetDefault ("ns3::NrSpectrumPhy::UnlicensedMode", BooleanValue (true));

  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize",
                      UintegerValue (999999999));
  Config::SetDefault ("ns3::LteRlcTm::MaxTxBufferSize",
                      UintegerValue (999999999));

  Config::SetDefault ("ns3::PointToPointEpcHelper::S1uLinkDelay", TimeValue (MilliSeconds (0)));
  Config::SetDefault ("ns3::NoBackhaulEpcHelper::X2LinkDelay", TimeValue (MilliSeconds (0)));
  Config::SetDefault ("ns3::LteEnbRrc::EpsBearerToRlcMapping",  StringValue (rlcModel));

  Config::SetDefault ("ns3::NrAmc::ErrorModelType", TypeIdValue (TypeId::LookupByName (errorModel)));
  Config::SetDefault ("ns3::NrAmc::AmcModel", EnumValue (NrAmc::ShannonModel));
  Config::SetDefault ("ns3::NrSpectrumPhy::ErrorModelType", TypeIdValue (TypeId::LookupByName (errorModel)));

  /* Global params: no fragmentation, no RTS/CTS, fixed rate for all packets */
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("999999"));
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("999999"));

  Config::SetDefault ("ns3::ApWifiMac::EnableBeaconJitter", BooleanValue (true));

  // Cat3 and Cat 4 ED treshold, Cat2 has its own attribute, maybe in future each could have its own
  Config::SetDefault ("ns3::NrLbtAccessManager::EnergyDetectionThreshold", DoubleValue (cat3and4EDThreshold));
  // Cat2 ED threshold
  Config::SetDefault ("ns3::NrCat2LbtAccessManager::Cat2EDThreshold", DoubleValue (cat2EDThreshold));
}

static void
TimePasses ()
{
  time_t t = time (nullptr);
  struct tm tm = *localtime (&t);

  std::cout << "Simulation Time: " << Simulator::Now ().As (Time::S) << " real time: "
            << tm.tm_hour << "h, " << tm.tm_min << "m, " << tm.tm_sec << "s." << std::endl;
  Simulator::Schedule (MilliSeconds (100), &TimePasses);
}

static bool m_nrIsOccupying = false;
static bool m_wifiIsOccupying = false;
static Time m_nrOccupancy;
static Time m_wifiOccupancy;
static OutputManager *outputManager;

static void
ResetNrOccupancy ()
{
  m_nrIsOccupying = false;
}

static void
ResetWifiOccupancy ()
{
  m_wifiIsOccupying = false;
}

static void
NrOccupancy (uint32_t nodeId, const Time & time)
{
  outputManager->UidIsTxing (nodeId);
  if (m_wifiIsOccupying)
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
WifiOccupancy (uint32_t nodeId, const Time & time)
{
  outputManager->UidIsTxing (nodeId);
  if (m_nrIsOccupying)
    {
      outputManager->SimultaneousTxOtherTechnology (nodeId);
    }
  if (m_wifiIsOccupying)
    {
      outputManager->SimultaneousTxSameTechnology (nodeId);
    }

  m_wifiOccupancy += time;
  m_wifiIsOccupying = true;
  Simulator::Schedule (time, &ResetWifiOccupancy);
}

int
main (int argc, char *argv[])
{
  bool cellScan = true;
  double beamSearchAngleStep = 30.0; // degrees
  double totalTxPower = 4; // dBm
  double ueTxPower = 2; // dBm
  uint16_t numerologyBwp1 = 0; //FIXME: Negative delay in NrPhy::GetSymbolPeriod with numerology = 1...
  double frequencyBwp1 = 5.2e9; // Hz Wi-Fi channel 42
  double bandwidthBwp1 = 20e6; // Hz
  double ueX = 5.0; // meters

  double simTime = 1.5; // seconds
  double udpAppStartTime = 0.5; //seconds
  uint32_t scenarioId = 0;
  uint32_t runId = 0;
  uint32_t seed = 1;
  bool enableNr = true;
  bool enableWifi = true;
  bool doubleTechnology = false;
  double cat2EDThreshold = -69.0; // dBm
  double cat3and4EDThreshold = -79.0; // dBm

  std::string rlcModel = "RlcUmAlways";
  std::string errorModel = "ns3::NrEesmIrT1";
  std::string nodeRate = "500kbps";
  std::string gnbCamType = "ns3::NrCat4LbtAccessManager";
  std::string ueCamType = "ns3::NrAlwaysOnAccessManager"; //this should be kept always on since grant-free UL is not supported yet
  std::string wifiStandard = "11ax";

  CommandLine cmd;

  cmd.AddValue ("simTime", "Simulation time (seconds)", simTime);
  cmd.AddValue ("cellScan",
                "Use beam search method to determine beamforming vector,"
                " the default is long-term covariance matrix method"
                " true to use cell scanning method, false to use the default"
                " power method.",
                cellScan);
  cmd.AddValue ("beamSearchAngleStep",
                "Beam search angle step (degrees) for beam search method",
                beamSearchAngleStep);
  cmd.AddValue ("totalTxPower",
                "Total TX power (dBm) that will be proportionally assigned to"
                " bandwidth parts depending on each BWP bandwidth ",
                totalTxPower);
  cmd.AddValue ("errorModelType",
                "Error model type: ns3::NrEesmCcT1, ns3::NrEesmCcT2, ns3::NrEesmIrT1, ns3::NrEesmIrT2",
                errorModel);
  cmd.AddValue ("rlcModel", "The NR RLC Model: RlcTmAlways or RlcUmAlways", rlcModel);
  cmd.AddValue ("ueX", "X position (meters) of any UE", ueX);
  cmd.AddValue ("scenario",
                "Scenario (0 = simple interference, 1 = new position)",
                scenarioId);
  cmd.AddValue ("seed", "Simulation seed", seed);
  cmd.AddValue ("runId", "Simulation Run ID", runId);
  cmd.AddValue ("enableNr", "Enable NR node", enableNr);
  cmd.AddValue ("enableWifi", "Enable Wi-Fi nodes", enableWifi);
  cmd.AddValue ("nodeRate", "The rate of every node in the network", nodeRate);
  cmd.AddValue ("gnbCamType", "The gNB CAM", gnbCamType);
  cmd.AddValue ("doubleTechnology", "Double the technology", doubleTechnology);
  cmd.AddValue ("cat2EDThreshold", "The ED threshold to be used by Lbt category 2 algorithm (dBm). Allowed range [-100.0, 0.0]., ", cat2EDThreshold);
  cmd.AddValue ("cat3and4EDTreshold", "The ED threshold to be used by Lbt category 3 and 4 algorithm (dBm). Allowed range [-100.0, 0.0].", cat3and4EDThreshold);
  cmd.AddValue ("wifiStandard", "The Wi-Fi standard to use (11ac, 11ax).", wifiStandard);

  cmd.Parse (argc, argv);

  RngSeedManager::SetSeed (seed);
  RngSeedManager::SetRun (runId);
  ConfigureDefaultValues (cellScan, beamSearchAngleStep, errorModel, cat2EDThreshold, cat3and4EDThreshold, rlcModel);

  enum WifiStandard standard = WIFI_STANDARD_80211ax_5GHZ;
  if (wifiStandard == "11ac")
    {
      standard = WIFI_STANDARD_80211ac;
    }
  else if (wifiStandard != "11ax")
    {
      NS_ABORT_MSG ("Unsupported Wi-Fi standard");
    }

  NodeDistributionScenario *scenario;

  // Place of the node inside the UE/STA container
  uint32_t nrNodePlace = 0;
  uint32_t wifiNodePlace = 0;

  NS_ASSERT (!(enableNr && enableWifi && doubleTechnology));

  uint32_t nodes = 0;
  if (enableNr)
    {
      ++nodes;
      wifiNodePlace = 1;
    }
  if (enableWifi)
    {
      ++nodes;
    }

  if ((enableNr && doubleTechnology))
    {
      ++nodes;
      wifiNodePlace = 1;
    }
  else if (enableWifi && doubleTechnology)
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
      NS_ASSERT (doubleTechnology || (enableNr && enableWifi));
      scenario = new InFrontNodeDistribution (Vector (0, 0, 1.5), 10.0, ueX);
    }
  else
    {
      NS_FATAL_ERROR ("Scenario not recognized");
    }

  std::stringstream ss;
  std::string technology = "";
  std::string gnbCam = "";
  if (enableNr)
    {
      technology += "with-nr-";
    }
  else
    {
      technology += "without-nr-";
    }
  if (enableWifi)
    {
      technology += "with-wifi-" + wifiStandard + "-";
    }
  else
    {
      technology += "without-wifi-";
    }

  if (doubleTechnology)
    {
      technology += "nr-wifi-" + wifiStandard + "-";
    }
  if (gnbCamType == "ns3::NrCat4LbtAccessManager")
    {
      gnbCam = "Cat4Lbt";
    }
  else if (gnbCamType == "ns3::NrCat3LbtAccessManager")
    {
      gnbCam = "Cat3Lbt";
    }
  else if (gnbCamType == "ns3::NrCat2LbtAccessManager")
    {
      gnbCam = "Cat2Lbt";
    }
  else if (gnbCamType == "ns3::NrOnOffAccessManager")
    {
      gnbCam = "OnOff";
    }
  else if (gnbCamType == "ns3::NrAlwaysOnAccessManager")
    {
      gnbCam = "AlwaysOn";
    }
  else
    {
      gnbCam = "Unknown";
      NS_ABORT_MSG ("Unknown gnbCamType: " << gnbCamType);
    }

  Packet::EnablePrinting ();

  ss << "cttc-nr-wifi-interference-example-" << scenarioId << "-" << technology;
  ss << gnbCam << "-" << nodeRate << "-" << rlcModel << ".db";

  SqliteOutputManager manager (ss.str (), ss.str (), ueX, seed, runId);
  outputManager = &manager;

  L2Setup *nr, *wifi;

  // Will be instantiated and configured by NrSingleBwpSetup
  Ptr<MultiModelSpectrumChannel> channel = CreateObject<MultiModelSpectrumChannel> ();
  Ptr<ThreeGppPropagationLossModel> propagation = CreateObject<ThreeGppIndoorOfficePropagationLossModel> ();
  Ptr<ThreeGppSpectrumPropagationLossModel> spectrumPropagation = CreateObject<ThreeGppSpectrumPropagationLossModel> ();

  propagation->SetAttributeFailSafe ("Frequency", DoubleValue (frequencyBwp1));
  spectrumPropagation->SetChannelModelAttribute ("Frequency", DoubleValue (frequencyBwp1));

  BandwidthPartInfo::Scenario channelScenario = BandwidthPartInfo::InH_OfficeMixed;
  Ptr<ChannelConditionModel> channelConditionModel = CreateObject<ThreeGppIndoorMixedOfficeChannelConditionModel> ();
  spectrumPropagation->SetChannelModelAttribute ("Scenario", StringValue ("InH-OfficeMixed"));
  spectrumPropagation->SetChannelModelAttribute ("ChannelConditionModel", PointerValue (channelConditionModel));
  propagation->SetChannelConditionModel (channelConditionModel);

  channel->AddPropagationLossModel (propagation);
  channel->AddSpectrumPropagationLossModel (spectrumPropagation);

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
          gnbs = NodeContainer (scenario->GetGnbs ());
          ues = NodeContainer (scenario->GetUes ());
          connections = { { 0, 0}, {1, 1}};
        }
      else
        {
          gnbs = NodeContainer (scenario->GetGnbs ().Get (nrNodePlace));
          ues = NodeContainer (scenario->GetUes ().Get (nrNodePlace));
          connections = { { 0, 0} };
        }

      nr = new NrSingleBwpSetup (gnbs,ues, channel, propagation, spectrumPropagation,
                                 frequencyBwp1, bandwidthBwp1, numerologyBwp1, totalTxPower, ueTxPower, connections,
                                 gnbCamType, ueCamType, "ns3::NrMacSchedulerTdmaPF", channelScenario);
      ueIpIface.Add (nr->AssignIpv4ToUe (address));
      nr->AssignIpv4ToStations (address); // Not used
      nr->ConnectToRemotes (remoteHostContainer, "1.0.0.0");
      nr->SetSinrCallback (MakeCallback (&OutputManager::SinrStore, &manager));
      nr->SetMacTxDataFailedCb (MakeCallback (&OutputManager::MacDataTxFailed, &manager));
      nr->SetChannelOccupancyCallback (MakeCallback (&NrOccupancy));
    }
  if (enableWifi)
    {
      std::unique_ptr<Ipv4AddressHelper> address = std::unique_ptr<Ipv4AddressHelper> (new Ipv4AddressHelper ());
      address->SetBase ("10.0.0.0", "255.255.255.0");
      wifi = new WifiSetup (NodeContainer (scenario->GetGnbs ().Get (wifiNodePlace)),
                            NodeContainer (scenario->GetUes ().Get (wifiNodePlace)),
                            channel, propagation, spectrumPropagation,
                            frequencyBwp1, bandwidthBwp1, totalTxPower, totalTxPower, -62.0, -62.0,
                            standard, "primero");
      ueIpIface.Add (wifi->AssignIpv4ToUe (address));
      wifi->AssignIpv4ToStations (address);
      wifi->ConnectToRemotes (remoteHostContainer, "2.0.0.0");
      wifi->SetSinrCallback (MakeCallback (&OutputManager::SinrStore, &manager));
      wifi->SetMacTxDataFailedCb (MakeCallback (&OutputManager::MacDataTxFailed, &manager));
      wifi->SetChannelOccupancyCallback (MakeCallback (&WifiOccupancy));
      dynamic_cast<WifiSetup*> (wifi)->SetChannelRTACallback (MakeCallback (&OutputManager::ChannelRequestTime, &manager));

      if (doubleTechnology)
        {
          address->SetBase ("10.0.1.0", "255.255.255.0");
          auto secondWifi = new WifiSetup (NodeContainer (scenario->GetGnbs ().Get (nrNodePlace)),
                                           NodeContainer (scenario->GetUes ().Get (nrNodePlace)),
                                           channel, propagation, spectrumPropagation,
                                           frequencyBwp1, bandwidthBwp1, totalTxPower, totalTxPower,
                                           -62.0, -62.0,
                                           standard, "segundo");
          ueIpIface.Add (secondWifi->AssignIpv4ToUe (address));
          secondWifi->AssignIpv4ToStations (address);
          secondWifi->ConnectToRemotes (remoteHostContainer, "2.1.0.0");
          secondWifi->SetSinrCallback (MakeCallback (&OutputManager::SinrStore, &manager));
          secondWifi->SetMacTxDataFailedCb (MakeCallback (&OutputManager::MacDataTxFailed, &manager));
          secondWifi->SetChannelOccupancyCallback (MakeCallback (&WifiOccupancy));
          dynamic_cast<WifiSetup*> (secondWifi)->SetChannelRTACallback (MakeCallback (&OutputManager::ChannelRequestTime, &manager));
        }
    }

  uint32_t ifId = 1;
  if (enableNr)
    {
      Ipv4StaticRoutingHelper ipv4RoutingHelper;
      for (auto it = remoteHostContainer.Begin (); it != remoteHostContainer.End (); ++it)
        {
          Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting ((*it)->GetObject<Ipv4> ());
          remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), ifId);
        }
      ifId++;
    }

  if (enableWifi)
    {
      Ipv4StaticRoutingHelper ipv4RoutingHelper;
      for (auto it = remoteHostContainer.Begin (); it != remoteHostContainer.End (); ++it)
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
  dlPacketSinkHelper.SetAttribute ("EnableSeqTsSizeHeader", BooleanValue (true));

  if (enableNr)
    {
      serverApps.Add (dlPacketSinkHelper.Install (scenario->GetUes ().Get (nrNodePlace)));
      if (doubleTechnology)
        {
          serverApps.Add (dlPacketSinkHelper.Install (scenario->GetUes ().Get (wifiNodePlace)));
        }
    }
  if (enableWifi)
    {
      serverApps.Add (dlPacketSinkHelper.Install (scenario->GetUes ().Get (wifiNodePlace)));
      if (doubleTechnology)
        {
          serverApps.Add (dlPacketSinkHelper.Install (scenario->GetUes ().Get (nrNodePlace)));
        }
    }

  serverApps.Start (Seconds (udpAppStartTime));

  for (uint32_t j = 0; j < scenario->GetUes ().GetN (); ++j)
    {
      OnOffHelper onoff ("ns3::UdpSocketFactory",
                         Address (InetSocketAddress (ueIpIface.GetAddress (j), dlPort)));
      onoff.SetConstantRate (DataRate (nodeRate), PACKET_SIZE);
      onoff.SetAttribute ("EnableSeqTsSizeHeader", BooleanValue (true));
      clientApps.Add (onoff.Install (remoteHost));
    }

  if (enableNr)
    {
      clientApps.Get (nrNodePlace)->SetStartTime (Seconds (udpAppStartTime));
      if (doubleTechnology)
        {
          clientApps.Get (wifiNodePlace)->SetStartTime (Seconds (udpAppStartTime));
        }
    }

  if (enableWifi)
    {
      clientApps.Get (wifiNodePlace)->SetStartTime (Seconds (udpAppStartTime)); // Manual HaCK
      if (doubleTechnology)
        {
          clientApps.Get (nrNodePlace)->SetStartTime (Seconds (udpAppStartTime));
        }
    }

  serverApps.Stop (Seconds (simTime));
  clientApps.Stop (Seconds (simTime));

  PopulateArpCache ();

  std::cout << "UE" << std::endl;
  PrintIpAddress (scenario->GetUes ());
  PrintRoutingTable (scenario->GetUes ());
  std::cout << "GNB" << std::endl;
  PrintIpAddress (scenario->GetGnbs ());
  PrintRoutingTable (scenario->GetGnbs ());
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
      bool isNr = ((t.sourceAddress.Get () >> 24) & 0xff) == 1;
      bool isWigig = ((t.sourceAddress.Get () >> 24) & 0xff) != 1;
      if (isNr || isWigig)
        {
          std::string technology = isNr ? "nr" : "wifi";
          double thMbps = i->second.rxBytes * 8.0 / (simTime - udpAppStartTime) / 1e6;
          double delay = 0.0;
          double jitter = 0.0;
          if (i->second.rxPackets > 1)
            {
              delay = i->second.delaySum.GetMicroSeconds () / i->second.rxPackets;
              jitter = i->second.jitterSum.GetMicroSeconds () / (i->second.rxPackets - 1);
            }
          std::stringstream addr;
          t.destinationAddress.Print (addr);
          if (i->second.txBytes > 300)
            {
              manager.StoreE2EStatsFor (technology, thMbps, i->second.txBytes, i->second.rxBytes,
                                        delay, jitter, addr.str ());
            }
        }
    }

  if (enableNr)
    {
      manager.StoreChannelOccupancyRateFor ("nr", m_nrOccupancy.GetSeconds () / (simTime - udpAppStartTime));
    }

  if (enableWifi)
    {
      manager.StoreChannelOccupancyRateFor ("wifi", m_wifiOccupancy.GetSeconds () / (simTime - udpAppStartTime));
    }

  manager.Close ();


  Simulator::Destroy ();
  return 0;
}

