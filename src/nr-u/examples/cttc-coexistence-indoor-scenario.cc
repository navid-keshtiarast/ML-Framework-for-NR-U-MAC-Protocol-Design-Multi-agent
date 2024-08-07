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


/**
 * \brief The 3gpp indoor office scenario layout
 *
 * The scenario layout is inspired by 3GPP TR 38.889 v1.0.0 8.1.1 indoor scenario.
 * According to this scenario there are 3 nodes (gNBs or APs) per operator and
 * they are distributed as in the following figure.
 *
 *
 * \verbatim
 *
 * In the following Figure is illustrated the scenario with the gNB/AP positions
 * which are represented with "x". The UE nodes are randomly uniformly dropped
 * in the area. With [A] are denoted nodes belonging to the operator A,
 * and with [B] are denoted nodes belonging to the operator B. Note that Distances
 * d1 and d2 are configurable. Also, the number of gNBs/APs/UEs/STA is configurable.
 *
 * <pre>
 *   +---------------------  d1 ---------------------- +
 *   |                                                 |
 *   |      [B]              [A]              [B]      |
 *   |       x ---- d1/3 ---- x ---- d1/3 ---- x       |
 *   |       |                                         |
 *   |       |                                         |
 *  d2      d2/2                                       |
     |       |                                         |
     |       |                                         |
 *   |       x                x                x       |
 *   |      [A]              [B]              [A]      |
 *   |                                                 |
 *   +------------------------------------------------++
 *
 *
 \endverbatim
 *
 * To run the coexistence simulation Nr/Wigig with the default parameters for
 * scenario setup and each technology, one could execute the following:
 *
 * ./waf --run cttc-coexistence-indoor-scenario --command-template="%s --enableNr=true --enableWigig=true"
 *
 * The execution of the simulation with these default parameters may take around 20 minutes.
 *
 * The parameters that can be configured through the command line can be found by using the following command:
 *
 * ./waf --run cttc-coexistence-indoor-scenario --command-template="%s --help"
 *
 * An example of configuring various parameters for a single simulation run is given in the following:
 *
 * ./waf --run cttc-coexistence-indoor-scenario --command-template="%s --enableNr=true --enableWigig=true
 * --gnbCamType=ns3::NrCat4LbtAccessManager --ueCamType=ns3::NrCat2LbtAccessManager
 * --numerology=2 --cat2EDThreshold=-20 --cat3and4EDTreshold=-30 --cat3and4Mcot=4
 * --nrBandwidth=200e6 --harqMethod=HarqIr --gnbHeight=2.5 --ueHeight=1.4
 * --d1=30 --d2=20 --amcModel=ShannonModel
 * --gnbAntennaDim1=8 --gnbAntennaDim2=4 --ueAntennaDim1=3 --ueAntennaDim2=2
 * --apAntennaDim1=8 --apAntennaDim2=6 --staAntennaDim1=4 --staAntennaDim2=4"
 *
 * To configure different transmission powers per type of node one could use the following parameters:
 * ./waf --run cttc-coexistence-indoor-scenario --command-template="%s --enableNr=true --enableWigig=true
 * --gnbCamType=ns3::NrCat4LbtAccessManager --ueCamType=ns3::NrCat2LbtAccessManager
 * --gnbTxPower=10 --ueTxPower=10 --apTxPower=10 --staTxPower=10"
 *
 * The output of the simulation is written into the database which consists of various
 * tables that are used to collect different types of statistics:
 *
 * 1) channel_occupancy table - shows the channel occupancy in time domain per operator
 * 2) collision - shows the occurrences of the simultaneous transmissions
 * 3) e2e - shows e2e KPIs such as: throughput in Mbps, txBytes, rxBytes, latency,
 * jitter per each flow, designating also by which technology is served each flow
 * 4) mac_data_tx_failed - shows the number of failed transmissions per node
 * 5) sinr - shows traced SINR values along the course of the simulation
 *
 * Additionally, positions of operator A gNBs and UEs are written to files:
 * gnb-layout-a-${run}.data and ue-layout-a-${run}.data, respectively.
 * Accordingly, positions of nodes belonging to operator B are written to files:
 * gnb-layout-b-${run}.data and ue-layout-b-${run}.data. ${run} corresponds to
 * random seed number used for specific simulation execution. Default ${run} number is 0.
 *
 * In order to display 2D/3D positions of all gNBs and UEs one could use e.g.
 * gnuplot tool in the following way:
 *
 * $ gnuplot -e "set terminal pngcairo; set output '| display png:-'; plot 'gnb-layout-a-0.data' u 2:3, 'ue-layout-a-0.data' u 2:3, 'gnb-layout-b-0.data' u 2:3,'ue-layout-b-0.data' u 2:3"
 *
 * (To plot 3D positions one could use splot in previous command instead of plot,
 * and add the column 4 to the using directive: "u 2:3:4"). It is also possible
 * to add labels to the node (they are stored in column 1) in this way (extend
 * to your necessities:) "plot 'gnb-layout-a-0.data' u 2:3:1 with labels offset char 1,1, '' u 2:3 w p notitle".
 *
 * In order to save directly to .png file one could use the following command:
 * $ gnuplot -e "set terminal pngcairo; set output 'topology.png'; plot 'gnb-layout-a-0.data', 'ue-layout-a-0.data', 'gnb-layout-b-0.data','ue-layout-b-0.data'"
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

NS_LOG_COMPONENT_DEFINE ("CttcCoexistenceIndoorScenario");

static void
ConfigureDefaultValues (bool cellScan = true, double beamSearchAngleStep = 10.0,
                        uint32_t eesmTable = 1,
                        const std::string & errorModel = "ns3::NrEesmErrorModel",
                        double cat2EDThreshold = - 69.0, double cat3and4EDThreshold = -79.0,
                        uint16_t cat3and4Mcot = 9, int nrFixedMcs = -1, const std::string &harqMethod = "HarqCc",
                        const std::string & amcModel = "ShannonModel",
                        uint16_t gnbAntennaDim1 = 8, uint16_t gnbAntennaDim2 = 8,
                        uint16_t ueAntennaDim1 = 4, uint16_t ueAntennaDim2 = 4,
                        const std::string & rlcModel = "RlcTmAlways",
                        const std::string & scheduler = "ns3::MmWaveMacSchedulerTdmaPF",
                        bool shadowing = false, const std::string &channelCond = "l",
                        const std::string &propScenario = "InH-OfficeMixed",
                        uint32_t updateChannelMs = 0, uint64_t rlcBufSize = 999999999,
                        uint32_t reorderingWindowMs = 10)
{
  Config::SetDefault ("ns3::MmWave3gppPropagationLossModel::ChannelCondition",
                      StringValue(channelCond));
  Config::SetDefault ("ns3::MmWave3gppPropagationLossModel::Scenario",
                      StringValue(propScenario));
  Config::SetDefault ("ns3::MmWave3gppPropagationLossModel::Shadowing",
                      BooleanValue(shadowing));

  Config::SetDefault ("ns3::MmWave3gppChannel::CellScan",
                      BooleanValue(cellScan));
  Config::SetDefault ("ns3::MmWave3gppChannel::UpdatePeriod",
                      TimeValue(MilliSeconds(updateChannelMs)));
  Config::SetDefault ("ns3::MmWave3gppChannel::BeamSearchAngleStep",
                      DoubleValue(beamSearchAngleStep));

  Config::SetDefault ("ns3::MmWaveEnbPhy::AntennaNumDim1", UintegerValue (gnbAntennaDim1));
  Config::SetDefault ("ns3::MmWaveEnbPhy::AntennaNumDim2", UintegerValue (gnbAntennaDim2));

  Config::SetDefault ("ns3::MmWaveUePhy::AntennaNumDim1", UintegerValue (ueAntennaDim1));
  Config::SetDefault ("ns3::MmWaveUePhy::AntennaNumDim2", UintegerValue (ueAntennaDim2));

  // gNB noise figure shall be set to 7 dB
  Config::SetDefault("ns3::MmWaveEnbPhy::NoiseFigure", DoubleValue (7));
  // UE noise figure shall be set to 7 dB
  Config::SetDefault("ns3::MmWaveUePhy::NoiseFigure", DoubleValue (7));

  Config::SetDefault("ns3::AntennaArrayModel::AntennaOrientation", EnumValue (AntennaArrayModel::X0));

  Config::SetDefault ("ns3::MmWaveSpectrumPhy::UnlicensedMode", BooleanValue (false));

  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize",
                      UintegerValue(rlcBufSize));
  Config::SetDefault ("ns3::LteRlcTm::MaxTxBufferSize",
                      UintegerValue (rlcBufSize));
  Config::SetDefault ("ns3::LteRlcUm::ReorderingTimer",
                      TimeValue (MilliSeconds (reorderingWindowMs)));
  Config::SetDefault("ns3::LteEnbRrc::EpsBearerToRlcMapping",  StringValue (rlcModel));

  Config::SetDefault ("ns3::MmWaveHelper::NumberOfComponentCarriers", UintegerValue (1));

  Config::SetDefault("ns3::PointToPointEpcHelper::S1uLinkDelay", TimeValue (MilliSeconds(0)));
  Config::SetDefault("ns3::NoBackhaulEpcHelper::X2LinkDelay", TimeValue (MilliSeconds(0)));

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

  if (amcModel == "ShannonModel")
    {
      Config::SetDefault("ns3::NrAmc::AmcModel", EnumValue (NrAmc::ShannonModel));
    }
  else if (amcModel == "ErrorModel")
    {
      Config::SetDefault("ns3::NrAmc::AmcModel", EnumValue (NrAmc::ErrorModel));
    }
  else
    {
      NS_FATAL_ERROR ("AMC model not valid, you set " << amcModel << ". Allowed options are ShannonModel and ErrorModel");
    }

  if (harqMethod == "HarqCc")
     {
       Config::SetDefault("ns3::NrEesmErrorModel::HarqMethod", EnumValue (NrEesmErrorModel::HarqCc));
     }
   else if (harqMethod == "HarqIr")
     {
       Config::SetDefault("ns3::NrEesmErrorModel::HarqMethod", EnumValue (NrEesmErrorModel::HarqIr));
     }
   else
     {
       NS_FATAL_ERROR ("HARQ method not valid, you set " << harqMethod);
     }

  Config::SetDefault("ns3::MmWaveSpectrumPhy::ErrorModelType", TypeIdValue (TypeId::LookupByName(errorModel)));

  /* Global params: no fragmentation, no RTS/CTS, fixed rate for all packets */
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("999999"));
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("999999"));

  Config::SetDefault ("ns3::DmgWifiMac::BeamReciprocity", BooleanValue (true));
  Config::SetDefault ("ns3::DmgApWifiMac::EnableBeaconIntervalJitter", BooleanValue (true));

  Config::SetDefault ("ns3::MmWavePhyMacCommon::MacSchedulerType", TypeIdValue (TypeId::LookupByName(scheduler)));

  // Cat3 and Cat 4 ED treshold, Cat2 has its own attribute, maybe in future each could have its own
  Config::SetDefault ("ns3::NrLbtAccessManager::EnergyDetectionThreshold", DoubleValue (cat3and4EDThreshold));
  Config::SetDefault ("ns3::NrLbtAccessManager::Mcot", TimeValue (MilliSeconds(cat3and4Mcot)));
  // Cat2 ED threshold
  Config::SetDefault ("ns3::NrCat2LbtAccessManager::Cat2EDThreshold", DoubleValue (cat2EDThreshold));

  if (nrFixedMcs > 0)
    {
      Config::SetDefault ("ns3::MmWaveMacSchedulerNs3::FixedMcsDl", BooleanValue (true));
      Config::SetDefault ("ns3::MmWaveMacSchedulerNs3::FixedMcsUl", BooleanValue (true));
      Config::SetDefault ("ns3::MmWaveMacSchedulerNs3::StartingMcsDl", UintegerValue (static_cast<uint32_t> (nrFixedMcs)));
      Config::SetDefault ("ns3::MmWaveMacSchedulerNs3::StartingMcsUl", UintegerValue (static_cast<uint32_t> (nrFixedMcs)));
    }
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

  m_nrIsOccupying = true;
  m_nrOccupancy += time;
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

static Ptr<Node>
GetClosestAp (Ptr<Node> ueDevice, const NodeContainer &apDevices)
{

  NS_ASSERT_MSG (apDevices.GetN () > 0, "empty ap device container");
  Vector uepos = ueDevice->GetObject<MobilityModel> ()->GetPosition ();
  double minDistance = std::numeric_limits<double>::infinity ();
  Ptr<Node> closestApDevice;
  for (auto i = apDevices.Begin (); i != apDevices.End (); ++i)
    {
      Vector enbpos = (*i)->GetObject<MobilityModel> ()->GetPosition ();
      double distance = CalculateDistance (uepos, enbpos);
      if (distance < minDistance)
        {
          minDistance = distance;
          closestApDevice = (*i);
        }
    }
  NS_ASSERT (closestApDevice != nullptr);
  return closestApDevice;
}

int
main (int argc, char *argv[])
{
  bool cellScan = true;
  double beamSearchAngleStep = 30.0;
  double gnbTxPower = -76.34; // corresponds to 17 dBm
  double ueTxPower = -76.34;
  double apTxPower = -76.34;
  double staTxPower = -76.34;
  uint16_t numerologyBwp1 = 3;
  /* configured following WIGIG channels */
  double frequencyBwp1 = 58.32e9;
  double bandwidthBwp1 = 2160e6;
  double wigigBandwidth = 2160e6; // only currently supported bandwidth for wigig
  double ueX = 5.0;

  double simTime = 1.45; // seconds
  double udpAppStartTime = 0.4; //seconds
  uint32_t seed = 1;
  bool enableNr = false;
  bool enableWigig = false;
  bool doubleTechnology = false;
  bool allLink = true;
  double d1 = 60, d2 = 20, gnbHeight = 1.5, ueHeight = 1.5;
  uint32_t gnbCount = 3, ueCount = 12;

  std::string errorModel = "ns3::NrLteMiErrorModel";
  std::string harqMethod = "HarqCc";
  std::string amcModel = "ShannonModel";
  std::string rlcModel = "RlcUmAlways";

  uint32_t eesmTable = 1;
  std::string gnbCamType = "ns3::NrAlwaysOnAccessManager";
  std::string ueCamType = "ns3::NrAlwaysOnAccessManager";

  double cat2EDThreshold = -69.0;
  double cat3and4EDThreshold = -79.0;
  uint16_t cat3and4Mcot = 9;

  uint16_t gnbAntennaDim1 = 8;
  uint16_t gnbAntennaDim2 = 8;
  uint16_t ueAntennaDim1 = 4;
  uint16_t ueAntennaDim2 = 4;
  uint16_t apAntennaDim1 = 8;
  uint16_t apAntennaDim2 = 8;
  uint16_t staAntennaDim1 = 4;
  uint16_t staAntennaDim2 = 4;

  double apCcaThreshold = -79.0;
  double staCcaThreshold = -79.0;

  int nrFixedMcs = -1;

  std::string outputDir = "./";  // UNIX ONLY !!
  std::string outputDb = "";

  std::string nrScheduler = "ns3::MmWaveMacSchedulerTdmaPF";
  bool shadowing = false;
  std::string channelCondition = "l";
  std::string propagationScenario = "InH-OfficeMixed";
  uint32_t updateChannelMs = 0;
  uint64_t rlcBufSize = 999999999;
  uint32_t rlcReorderingWindow = 10;
  uint32_t segmentSize = 1000;

  uint32_t lambda = 1;
  uint32_t fileSize = 1250000;
  std::string transport = "udp";
  std::string appModel = "M3";
  std::string appDistribution = "constant";

  std::string aux1 = "n";
  std::string aux2 = "n";
  std::string aux3 = "n";
  std::string aux4 = "n";
  std::string aux5 = "n";
  std::string aux6 = "n";
  std::string aux7 = "n";
  std::string aux8 = "n";
  std::string aux9 = "n";

  CommandLine cmd;

  cmd.AddValue ("simTime", "Simulation time [seconds]", simTime);
  cmd.AddValue ("cellScan",
                " Determines which beam search method to use to determine beamforming vector for NR devices."
                " The default is long-term covariance matrix method. Set to true to use cell scanning method, "
                " and false to use the default power method.",
                cellScan);
  cmd.AddValue ("beamSearchAngleStep",
                "Beam search angle step (vertical tilt) for beam search cell scanning method [degrees].",
                beamSearchAngleStep);
  cmd.AddValue ("gnbTxPower",
                "The total gNB tx power that will be proportionally assigned to"
                " bandwidth parts depending on each BWP bandwidth. By "
                " default, currently, there is only one BWP specified in "
                " the indoor simulation script [dBm].",
                gnbTxPower);
  cmd.AddValue ("ueTxPower",
                "The total UE tx power that will be proportionally assigned to"
                " bandwidth parts depending on each BWP bandwidth. By "
                " default, currently, there is only one BWP specified in "
                " the indoor simulation script [dBm].",
                ueTxPower);
  cmd.AddValue ("apTxPower",
                "The total AP tx power [dBm].",
                apTxPower);
  cmd.AddValue ("staTxPower",
                "The total STA tx power [dBm].",
                staTxPower);
  cmd.AddValue("errorModelType",
               "Error model type: ns3::NrEesmErrorModel , ns3::NrLteErrorModel",
               errorModel);
  cmd.AddValue("eesmTable",
               "Table to use when error model is Eesm (1 for McsTable1 or 2 for McsTable2)",
               eesmTable);
  cmd.AddValue("harqMethod", "The HARQ method to be used in case of Eesm (HarqCc or HarqIr)",
                harqMethod);
  cmd.AddValue ("amcModel", "The AMC model to be used, available models are: ShannolModel (based on Shannon bound) and ErrorModel (based on target BLER).", amcModel);
  cmd.AddValue ("seed", "Simulation seed", seed);
  cmd.AddValue ("enableNr", "Enable NR operator", enableNr);
  cmd.AddValue ("enableWigig", "Enable Wigig operator", enableWigig);
  cmd.AddValue ("gnbCamType", "The GNB CAM that can be ns3::NrCat3LbtAccessManager (for Lbt Category 3), "
                               "ns3::NrCat4LbtAccessManager (for Lbt Category 4), "
                               "ns3::NrAlwaysOnAccessManager (for always on mode, which means NO Lbt), "
                               "or ns3::NrOnOffAccessManager (for ON/OFF mode, which means NO Lbt being used and "
                               "which by default consists of pattern: 9 ms ON and 9 ms OFF)", gnbCamType);
  cmd.AddValue ("ueCamType", "The UE CAM that can be ns3::NrCat2LbtAccessManager (performs Lbt Category 1/Category 2 depending on "
                              "the time gap between DL and UL transmissions),"
                              "ns3::NrOnOffAccessManager (for ON/OFF mode, which means NO lbt being used and "
                               "which by default consists of pattern: 9 ms ON and 9 ms OFF) "
                               "or ns3::NrAlwaysOnAccessManager (for always on mode, NO Lbt)", ueCamType);
  cmd.AddValue ("doubleTechnology", "The parameter needs to be set to true and used in combination with either enableNr or nableWigig parameter being set to true "
                                    "to specify that both operators, A and B, are of the same technology. If doubleTechnology and enableNr are set to true, then "
                                    "both operators are NR-Us, otherwise if doubleTechnology and enableWigig are set to true then both operators are Wigig. ", doubleTechnology);
  cmd.AddValue ("allLink", "Interference with all links?", allLink);
  cmd.AddValue ("d1", "Indoor scenario larger dimension d1 [m].", d1);
  cmd.AddValue ("d2", "Indoor scenario smaller dimension d2 [m].", d2);
  cmd.AddValue ("gnbHeight", "gNB height [m]", gnbHeight);
  cmd.AddValue ("ueHeight", "UE height [m]", ueHeight);
  cmd.AddValue ("gnbCount", "The number of gNBs/APs per operator.", gnbCount);
  cmd.AddValue ("ueCount", "The number of UEs/STAs per operator.", ueCount);
  cmd.AddValue ("nrBandwidth", "The total bandwidth being used by NR devices [Hz].", bandwidthBwp1);
  cmd.AddValue ("numerology", "The NR numerology to be used.", numerologyBwp1);
  cmd.AddValue ("rlcModel", "The NR RLC Model: RlcTmAlways or RlcUmAlways", rlcModel);
  cmd.AddValue ("cat2EDThreshold", "The ED threshold to be used by Lbt category 2 algorithm [dBm]. Allowed range [-100.0, 0.0]., ", cat2EDThreshold);
  cmd.AddValue ("cat3and4EDTreshold", "The ED threshold to be used by Lbt category 3 and 4 algorithm [dBm]. Allowed range [-100.0, 0.0].", cat3and4EDThreshold);
  cmd.AddValue ("cat3and4Mcot", "The MCOT to be used by Lbt category 3 and 4 algorithm [ms]. Allowed range of values [2ms, 20ms].", cat3and4Mcot);
  // antenna dimensions
  cmd.AddValue ("gnbAntennaDim1", "gNB antenna array dimension 1 (called M in 3GPP nomenclature, we assume P=1, Ng = Mg = 1) [number of antenna elements]", gnbAntennaDim1);
  cmd.AddValue ("gnbAntennaDim2","gNB antenna array dimension 2 (called N in 3GPP nomenclature, we assume P=1, Ng = Mg = 1) [number of antenna elements]", gnbAntennaDim2);
  cmd.AddValue ("ueAntennaDim1", "UE antenna array dimension 1 (called M in 3GPP nomenclature, we assume P=1, Ng = Mg = 1) [number of antenna elements]", ueAntennaDim1);
  cmd.AddValue ("ueAntennaDim2", "UE antenna array dimension 2 (called N in 3GPP nomenclature, we assume P=1, Ng = Mg = 1) [number of antenna elements]", ueAntennaDim2);
  cmd.AddValue ("apAntennaDim1", "AP antenna array dimension 1 [number of antenna elements]", apAntennaDim1);
  cmd.AddValue ("apAntennaDim2","AP antenna array dimension 2 [number of antenna elements]", apAntennaDim2);
  cmd.AddValue ("staAntennaDim1", "STA antenna array dimension 1 [number of antenna elements]", staAntennaDim1);
  cmd.AddValue ("staAntennaDim2", "STA antenna array dimension 2 [number of antenna elements]", staAntennaDim2);
  cmd.AddValue ("apCcaThreshold", "AP CcaMode1Threshold", apCcaThreshold);
  cmd.AddValue ("staCcaThreshold", "STA CcaMode1Threshold", staCcaThreshold);
  cmd.AddValue ("nrFixedMcs", "If set to a number > 0, fixes the MCS for all the NR nodes", nrFixedMcs);
  cmd.AddValue ("outputDir", "Output directory", outputDir);
  cmd.AddValue ("outputDb", "Output DB name", outputDb);
  cmd.AddValue ("nrScheduler", "NR scheduler", nrScheduler);
  cmd.AddValue ("shadowing", "Channel shadowing enabled/disabled", shadowing);
  cmd.AddValue ("channelCondition", "3GPP channel condition (l, a, n)", channelCondition);
  cmd.AddValue ("propagationScenario", "3GPP propagation scenario", propagationScenario);
  cmd.AddValue ("updateChannelMs", "Channel update time (0 to never update)", updateChannelMs);
  cmd.AddValue ("rlcBufSize", "RLC buffer size", rlcBufSize);
  cmd.AddValue ("rlcReorderingWindow", "RLC UM reordering window", rlcReorderingWindow);
  cmd.AddValue ("segmentSize", "Application/Transport segment size", segmentSize);

  cmd.AddValue ("lambda", "TD", lambda);
  cmd.AddValue ("fileSize", "TD", fileSize);
  cmd.AddValue ("transport", "TD", transport);
  cmd.AddValue ("appModel", "TD", appModel);
  cmd.AddValue ("appDistribution", "TD", appDistribution);
  cmd.AddValue ("aux1", "TD", aux1);
  cmd.AddValue ("aux2", "TD", aux2);
  cmd.AddValue ("aux3", "TD", aux3);
  cmd.AddValue ("aux4", "TD", aux4);
  cmd.AddValue ("aux5", "TD", aux5);
  cmd.AddValue ("aux6", "TD", aux6);
  cmd.AddValue ("aux7", "TD", aux7);
  cmd.AddValue ("aux8", "TD", aux8);
  cmd.AddValue ("aux9", "TD", aux9);

  cmd.Parse (argc, argv);

  NS_ABORT_IF (beamSearchAngleStep > 180);

  std::stringstream tmp;
  tmp << fileSize * 8 * lambda << "bps";
  std::string nodeRate = tmp.str ();

  RngSeedManager::SetSeed (seed);
  ConfigureDefaultValues (cellScan, beamSearchAngleStep, eesmTable, errorModel, cat2EDThreshold, cat3and4EDThreshold, cat3and4Mcot, nrFixedMcs, harqMethod, amcModel,
                          gnbAntennaDim1, gnbAntennaDim2, ueAntennaDim1, ueAntennaDim2, rlcModel, nrScheduler, shadowing,
                          channelCondition, propagationScenario, updateChannelMs, rlcBufSize, rlcReorderingWindow);

  if (allLink)
    {
      Config::SetDefault ("ns3::MmWave3gppChannel::EnableAllChannels", BooleanValue (true));
      Config::SetDefault ("ns3::MmWaveSpectrumPhy::EnableAllInterferences", BooleanValue (true));
    }

  ns3::Vector maxArea = {d1, d2, 0};
  std::unique_ptr<HalfMatrixLayout> operatorALayout, operatorBLayout;
  std::stringstream ss;
  std::string technology = "";
  std::string link;
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

  if (allLink)
    {
      link = "allLink-";
    }
  else
    {
      link = "noAllLink-";
    }

  if (outputDb.empty())
    {
      ss << "cttc-coexistence-indoor-" << technology;
      ss << link << gnbCamType << "-" << ueCamType << "-" << nodeRate << "-" << rlcModel << ".db";
    }
  else
    {
      ss << outputDb;
    }

  std::cout << "Simulation configuration started. The output will be stored in "
            << outputDir + ss.str () << std::endl;

  SqliteOutputManager manager (outputDir + ss.str(), ss.str (), ueX, seed,
                               static_cast<uint32_t> (RngSeedManager::GetRun ()));
  outputManager = &manager;

  L2Setup *nr;
  std::vector<std::unique_ptr<WigigSetup>> wigig;

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
  std::stringstream ssRemote;
  ssRemote << "REMOTE-" << remoteHost->GetId ();
  Names::Add (ssRemote.str(), remoteHost);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  Ipv4InterfaceContainer ueIpIfaceOperatorA, ueIpIfaceOperatorB;

  bool operatorAUsed = false;

  gnbTxPower = 10 * log10(bandwidthBwp1 * std::pow (10, gnbTxPower/10));
  ueTxPower = 10 * log10(bandwidthBwp1 * std::pow (10, ueTxPower/10));
  apTxPower = 10 * log10(wigigBandwidth * std::pow (10, apTxPower/10));
  staTxPower = 10 * log10(wigigBandwidth * std::pow (10, staTxPower/10));

  std::cout << "Derived power";

  if (enableNr)
    {
      std::cout << " GNB: " << gnbTxPower << " dBm, UE: "
                << ueTxPower << " dBm,";
    }
  if (enableWigig)
    {
      std::cout << " AP: " << apTxPower << " dBm, STA: " << staTxPower << " dBm.";
    }

  std::cout << std::endl;

  if (enableNr)
    {
      Ipv4InterfaceContainer totalContainer;
      std::unique_ptr<Ipv4AddressHelper> address = std::unique_ptr<Ipv4AddressHelper> (new Ipv4AddressHelper ());
      NodeContainer gnbs;
      NodeContainer ues;
      std::unordered_map<uint32_t, uint32_t> connections;

      operatorALayout = std::unique_ptr<HalfMatrixLayout> (new HalfMatrixLayout(2, gnbCount, ueCount, ns3::Vector (0,0,0), maxArea,
                                                                                ns3::HalfMatrixLayout::TOP, gnbHeight, ueHeight));

      gnbs.Add (operatorALayout->GetGnbs ());
      ues.Add (operatorALayout->GetUes ());
      operatorAUsed = true;

      if (doubleTechnology)
        {
          operatorBLayout = std::unique_ptr<HalfMatrixLayout> (new HalfMatrixLayout(2, gnbCount, ueCount, ns3::Vector (0,0,0), maxArea,
                                                                                     ns3::HalfMatrixLayout::BOTTOM, gnbHeight, ueHeight));
          gnbs.Add (operatorBLayout->GetGnbs());
          ues.Add (operatorBLayout->GetUes());
        }

      nr = new NrSingleBwpSetup (gnbs,ues, channel, propagation, threegppChannel,
                                 frequencyBwp1, bandwidthBwp1, numerologyBwp1, gnbTxPower, ueTxPower, connections,
                                 gnbCamType, ueCamType, nrScheduler);
      totalContainer.Add (nr->AssignIpv4ToUe(address));
      nr->AssignIpv4ToStations(address); // Not used
      nr->ConnectToRemotes (remoteHostContainer, "1.0.0.0");
      nr->SetSinrCallback (MakeCallback (&OutputManager::SinrStore, &manager));
      nr->SetMacTxDataFailedCb(MakeCallback (&OutputManager::MacDataTxFailed, &manager));
      nr->SetChannelOccupancyCallback (MakeCallback (&NrOccupancy));

      if (doubleTechnology)
        {
          for (auto itIp = totalContainer.Begin (); itIp != totalContainer.End (); ++itIp)
            {
              bool found = false;
              for (auto it = operatorBLayout->GetUes().Begin(); it != operatorBLayout->GetUes().End(); ++it)
                {
                  NS_ASSERT ((*itIp).first->GetNInterfaces() > 0);

                  if ((*itIp).first->GetNetDevice(0)->GetNode()->GetId () == (*it)->GetId ())
                    {
                      found = true;
                      break;
                    }
                }
              if (found)
                {
                  ueIpIfaceOperatorB.Add (*itIp);
                }
              else
                {
                  ueIpIfaceOperatorA.Add(*itIp);
                }
            }
        }
      else
        {
          ueIpIfaceOperatorA.Add (totalContainer);
        }
    }

  std::unordered_map<uint32_t, NodeContainer> networkWigigMap;
  std::unordered_map<uint32_t, Ptr<Node>> apMap;
  if (enableWigig)
    {
      // We have to create n networks, one for each AP. So, use a "closest"
      // approach to identify what UEs are in what network.

      NodeContainer gnbs;
      NodeContainer ues;

      if (operatorAUsed)
        {
          NS_ASSERT(operatorBLayout == nullptr);
          operatorBLayout = std::unique_ptr<HalfMatrixLayout> (new HalfMatrixLayout(2, gnbCount, ueCount, ns3::Vector (0,0,0), maxArea,
                                                                                     ns3::HalfMatrixLayout::BOTTOM, gnbHeight, ueHeight));
          gnbs.Add (operatorBLayout->GetGnbs ());
          ues.Add (operatorBLayout->GetUes ());
        }
      else
        {
          NS_ASSERT(operatorALayout == nullptr);
          operatorALayout = std::unique_ptr<HalfMatrixLayout> (new HalfMatrixLayout(2, gnbCount, ueCount, ns3::Vector (0,0,0), maxArea,
                                                                                    ns3::HalfMatrixLayout::TOP, gnbHeight, ueHeight));

          gnbs.Add (operatorALayout->GetGnbs ());
          ues.Add (operatorALayout->GetUes ());
        }

      if (doubleTechnology)
        {
          operatorBLayout = std::unique_ptr<HalfMatrixLayout> (new HalfMatrixLayout(2, gnbCount, ueCount, ns3::Vector (0,0,0), maxArea,
                                                                                     ns3::HalfMatrixLayout::BOTTOM, gnbHeight, ueHeight));
          gnbs.Add (operatorBLayout->GetGnbs());
          ues.Add (operatorBLayout->GetUes());
        }

      for (auto it = ues.Begin(); it != ues.End(); ++it)
        {
          Ptr<Node> closestAp = GetClosestAp (*it, gnbs);
          uint32_t closestApId = closestAp->GetId();
          std::cout << (*it)->GetId() << " connected with " << closestAp->GetId() << std::endl;
          auto v = networkWigigMap.find (closestApId);
          if (v == networkWigigMap.end ())
            {
              v = networkWigigMap.emplace (std::make_pair (closestApId, NodeContainer ())).first;
            }
          v->second.Add (*it);

          if (apMap.find (closestApId) == apMap.end ())
            {
              apMap.insert (std::make_pair (closestApId, closestAp));
            }
        }

      uint32_t startingClass = 1;

      for (const auto & v : networkWigigMap)
        {
          std::stringstream base, ssid, remoteAddr;
          base << "10." << startingClass << ".0.0";
          remoteAddr << "2." << startingClass << ".0.0";
          ssid << "coex" << startingClass;
          startingClass++;

          std::unique_ptr<Ipv4AddressHelper> address = std::unique_ptr<Ipv4AddressHelper> (new Ipv4AddressHelper ());
          address->SetBase(Ipv4Address (base.str().c_str ()), "255.255.255.0");
          auto it = apMap.find (v.first);
          wigig.push_back (std::unique_ptr<WigigSetup> (new WigigSetup (NodeContainer (it->second), v.second, channel, propagation, threegppChannel,
                                                                        frequencyBwp1, wigigBandwidth, apTxPower, staTxPower,
                                                                        apCcaThreshold, staCcaThreshold, ssid.str(),
                                                                        apAntennaDim1, apAntennaDim2, staAntennaDim1, staAntennaDim2)));
          if (operatorAUsed)
            {
              ueIpIfaceOperatorB.Add (wigig.back()->AssignIpv4ToUe(address));
            }
          else
            {
              ueIpIfaceOperatorA.Add (wigig.back()->AssignIpv4ToUe(address));
            }
          wigig.back()->AssignIpv4ToStations (address);
          wigig.back()->ConnectToRemotes (remoteHostContainer, remoteAddr.str().c_str());
          wigig.back()->SetSinrCallback (MakeCallback (&OutputManager::SinrStore, &manager));
          wigig.back()->SetMacTxDataFailedCb(MakeCallback (&OutputManager::MacDataTxFailed, &manager));
          wigig.back()->SetChannelOccupancyCallback (MakeCallback (&WigigOccupancy));
          wigig.back()->SetChannelRTACallback (MakeCallback (&OutputManager::ChannelRequestTime, &manager));
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
          for (uint32_t startingClass = 1; startingClass <= networkWigigMap.size (); startingClass++)
            {
              std::stringstream base;
              base << "10." << startingClass << ".0.0";
              Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting ((*it)->GetObject<Ipv4> ());
              remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address (base.str().c_str()), Ipv4Mask ("255.255.255.0"), ifId);
              ifId++;
            }
        }
    }

  ApplicationContainer clientApps, serverApps;
  uint16_t operatorAPort = 1234;
  uint16_t operatorBPort = 1235;

  Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable> ();

  if (operatorALayout != nullptr)
    {
      PacketSinkHelper dlPacketSinkHelperA ("ns3::UdpSocketFactory",
                                           Address (InetSocketAddress (Ipv4Address::GetAny (), operatorAPort)));
      dlPacketSinkHelperA.SetAttribute("EnableE2EStats", BooleanValue (true));
      serverApps.Add(dlPacketSinkHelperA.Install (operatorALayout->GetUes()));


      for (uint32_t j = 0; j < ueIpIfaceOperatorA.GetN(); ++j)
        {
          OnOffHelper onoff ("ns3::UdpSocketFactory",
                             Address (InetSocketAddress (ueIpIfaceOperatorA.GetAddress (j), operatorAPort)));
          onoff.SetConstantRate (DataRate (nodeRate), segmentSize);
          onoff.SetAttribute("EnableE2EStats", BooleanValue (true));
          ApplicationContainer app = onoff.Install (remoteHostContainer);
          double startTime = x->GetValue (udpAppStartTime, udpAppStartTime + 0.01);
          app.Start (Seconds(startTime));
          app.Stop (Seconds (startTime + 1.0));
          clientApps.Add (app);
          std::cout << "Installing app to transmit data to the operator A node "
                    << ueIpIfaceOperatorA.GetAddress (j)
                    << ":" << operatorAPort << " at time " << startTime
                    << " s and stop at " << startTime + 1.0 << " s." << std::endl;
        }
    }

  if (operatorBLayout != nullptr)
    {
      PacketSinkHelper dlPacketSinkHelperB ("ns3::UdpSocketFactory",
                                           Address (InetSocketAddress (Ipv4Address::GetAny (), operatorBPort)));
      dlPacketSinkHelperB.SetAttribute("EnableE2EStats", BooleanValue (true));
      serverApps.Add(dlPacketSinkHelperB.Install (operatorBLayout->GetUes()));


      for (uint32_t j = 0; j < ueIpIfaceOperatorB.GetN(); ++j)
        {
          OnOffHelper onoff ("ns3::UdpSocketFactory",
                             Address (InetSocketAddress (ueIpIfaceOperatorB.GetAddress (j), operatorBPort)));
          onoff.SetConstantRate (DataRate (nodeRate), segmentSize);
          onoff.SetAttribute("EnableE2EStats", BooleanValue (true));
          ApplicationContainer app = onoff.Install (remoteHostContainer);
          double startTime = x->GetValue (udpAppStartTime, udpAppStartTime + 0.01);
          app.Start (Seconds(startTime));
          app.Stop (Seconds (startTime + 1.0));
          clientApps.Add (app);
          std::cout << "Installing app to transmit data to the operator B node "
                    << ueIpIfaceOperatorB.GetAddress (j)
                    << ":" << operatorBPort << " at time " << startTime
                    << " s and stop at " << startTime + 1.0 << " s." << std::endl;
        }
    }

  serverApps.Start (Seconds (udpAppStartTime));
  serverApps.Stop (Seconds (simTime));

  PopulateArpCache ();

  if (operatorALayout != nullptr)
    {
      std::cout << "############# OPERATOR A" << std::endl;
      std::cout << "UE: " << std::endl;
      PrintIpAddress (operatorALayout->GetUes());
      PrintRoutingTable (operatorALayout->GetUes());
      std::cout << "GNB" << std::endl;
      PrintIpAddress (operatorALayout->GetGnbs());
      PrintRoutingTable (operatorALayout->GetGnbs());
      std::stringstream n, m;
      n << "gnb-layout-a-" << RngSeedManager::GetRun () << ".data";
      operatorALayout->PrintGnbPosToFile(n.str());
      m << "ue-layout-a-" << RngSeedManager::GetRun () << ".data";
      operatorALayout->PrintUePosToFile (m.str());
    }

  if (operatorBLayout != nullptr)
    {
      std::cout << "############# OPERATOR B" << std::endl;
      std::cout << "UE: " << std::endl;
      PrintIpAddress (operatorBLayout->GetUes());
      PrintRoutingTable (operatorBLayout->GetUes());
      std::cout << "GNB" << std::endl;
      PrintIpAddress (operatorBLayout->GetGnbs());
      PrintRoutingTable (operatorBLayout->GetGnbs());
      std::stringstream n, m;
      n << "gnb-layout-b-" << RngSeedManager::GetRun () << ".data";
      operatorBLayout->PrintGnbPosToFile(n.str());
      m << "ue-layout-b-" << RngSeedManager::GetRun () << ".data";
      operatorBLayout->PrintUePosToFile (m.str());
    }

  std::cout << "############# REMOTES:" << std::endl;
  PrintIpAddress (remoteHostContainer);
  PrintRoutingTable (remoteHostContainer);


  Simulator::Schedule (MilliSeconds (1), &TimePasses);

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

      bool isNr = false, isWigig = false;
      if (enableNr)
        {
          if (t.destinationPort == operatorAPort)
            {
              isNr = true;
            }
          if (doubleTechnology && t.destinationPort == operatorBPort)
            {
              isNr = true;
            }
        }
      if (enableWigig)
        {
          if (!enableNr && t.destinationPort == operatorAPort)
            {
              isWigig = true;
            }
          else if (t.destinationPort == operatorBPort)
            {
              isWigig = true;
            }
          if (doubleTechnology && t.destinationPort == operatorAPort)
            {
              isWigig = true;
            }
        }

      NS_ASSERT ((isNr == false && isWigig == false) || (isNr != isWigig));
      std::stringstream addr;
      t.destinationAddress.Print (addr);

      if (isNr || isWigig)
        {
          std::string technology = isNr ? "nr" : "wigig";
          double thMbps = i->second.rxBytes * 8.0 / (1.0) / 1e6;
          double delay = 0.0;
          double jitter = 0.0;
          if (i->second.rxPackets > 1)
            {
              delay = i->second.delaySum.GetMicroSeconds () / i->second.rxPackets;
              jitter = i->second.jitterSum.GetMicroSeconds () / (i->second.rxPackets - 1);
            }
          manager.StoreE2EStatsFor(technology, thMbps, i->second.txBytes, i->second.rxBytes,
                                   delay, jitter, addr.str ());
        }
      else
        {
          std::cout << "Ignoring throughput for " << addr.str () << std::endl;
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

