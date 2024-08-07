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

/*
    To run the coexistence simulation Nr/Wifi with the default parameters for
    scenario setup and each technology, one could execute the following:

    ./waf --run coex-ai.cc --command-template="%s --enableNr=true --enableWifi=true --simRound=1 --simTime=2 --appStartTime=1"
 
    The parameters that can be configured through the command line can be found by using the following command:
 
    ./waf --run coex-ai.cc --command-template="%s --help"

    In this scenario it is possible to change some parameters on each AP-STA pairs with a maximum number of 6 
    pair independent of the technology (Wifi or Nru). Remember that all parameters setups the Wifi (if exist) first, so
    for example if we have 1 nru and 1 wifi, trafficType1 sets the traffic type in Wifi and trafficType2 then sets the traffic type in Nru.  

    To configure different the default tranmission power one could use the following parameters

    ./waf --run coex-ai.cc --command-template="%s --enableNr=true --enableWifi=true --numWifiPairs=2 --numNruPairs=2 --totalTxPower1=23 --totalTxPower3=33 --ueTxPower2=10 --ueTxPower4=43"    

    To configure different Wifi parameters one could use the following

    ./waf --run coex-ai.cc --command-template="%s --enableNr=true --enableWifi=true --numWifiPairs=2 --numNruPairs=2 --ccaThresholdAp=-72 --ccaThresholdSta=-72 --frameAggregation=0"

    To configure different MAC parameters for each nru GNB one could use the following 

    ./waf --run coex-ai.cc --command-template="%s --enableNr=true --enableWifi=true --numWifiPairs=2 --numNruPairs=2 --gnbCamType1=ns3::NrCat4LbtAccessManager --gnbCamType2=ns3::NrAlwaysOnAccessManager"

    To configure different transport parameters for each node one could use the following
    
    ./waf --run coex-ai.cc --command-template="%s --enableNr=true --enableWifi=true --numWifiPairs=2 --numNruPairs=2 --trafficType1=OnOff packetSize1=1500 --trafficType2=Burst --fragmentSize2=1000 --trafficType3=UDP_SAT --udpSaturationRate3=100000000 --trafficType4=OnOff --nodeRate4=100Mbps"

    sudo python3 run_nr_mac.py --use_rl --result --no-tune
*/

#include <fstream>
#include <iostream>
#include <string>
#include <algorithm>

#include "ns3/log.h"
#include "ns3/core-module.h"
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

#include "ns3/multi-model-spectrum-channel.h"
#include "simulation-helper.h"

#include "ns3/seq-ts-size-frag-header.h"
#include "ns3/bursty-helper.h"
#include "ns3/burst-sink-helper.h"
#include "ns3/three-gpp-ftp-m1-helper.h"
#include "ns3/three-gpp-http-client.h"
#include "ns3/three-gpp-http-helper.h"
#include "ns3/three-gpp-http-server.h"
#include "ns3/three-gpp-http-variables.h"
#include "ns3/traffic-generator-ngmn-ftp-multi.h"
#include "ns3/traffic-generator-ngmn-gaming.h"
#include "ns3/traffic-generator-ngmn-video.h"
#include "ns3/traffic-generator-ngmn-voip.h"
#include "ns3/v4ping-helper.h"

#include "ns3/ai-module.h"
#include "nr-mac-env.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("nr_ai_mac");

static const uint32_t PACKET_SIZE = 1000;

static void
ConfigureDefaultValues (bool cellScan = true, double beamSearchAngleStep = 10.0,
                        const std::string &errorModel = "ns3::NrEesmErrorModel",
                        double cat2EDThreshold = -69.0, double cat3and4EDThreshold = -79,
                        const std::string &rlcModel = "RlcTmAlways",std::string SimTag="HI")
{
  Config::SetDefault ("ns3::ThreeGppPropagationLossModel::ShadowingEnabled",
                      BooleanValue (false));
  // if (cellScan)
  //   {
  //     Config::SetDefault ("ns3::IdealBeamformingHelper::BeamformingMethod", TypeIdValue (CellScanBeamforming::GetTypeId ()));
  //   }
  // else
  //   {
  //     Config::SetDefault ("ns3::IdealBeamformingHelper::BeamformingMethod", TypeIdValue (DirectPathBeamforming::GetTypeId ()));
  //   }
  // Config::SetDefault ("ns3::IdealBeamformingHelper::BeamformingPeriodicity",
  //                     TimeValue (MilliSeconds (1000))); //seldom updated
  // Config::SetDefault ("ns3::CellScanBeamforming::BeamSearchAngleStep",
  //                     DoubleValue (beamSearchAngleStep));

  Config::SetDefault ("ns3::UniformPlanarArray::NumColumns", UintegerValue (1));
  Config::SetDefault ("ns3::UniformPlanarArray::NumRows", UintegerValue (1));

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
  //Config::SetDefault ("ns3::NrAmc::AmcModel", EnumValue (NrAmc::ShannonModel));
  Config::SetDefault ("ns3::NrSpectrumPhy::ErrorModelType", TypeIdValue (TypeId::LookupByName (errorModel)));

  /* Global params: no fragmentation, no RTS/CTS, fixed rate for all packets */
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("999999"));
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("999999"));

  Config::SetDefault ("ns3::ApWifiMac::EnableBeaconJitter", BooleanValue (false));

  //Cat3 and Cat 4 ED treshold, Cat2 has its own attribute, maybe in future each could have its own
  Config::SetDefault ("ns3::NrLbtAccessManager::EnergyDetectionThreshold", DoubleValue (cat3and4EDThreshold));
  // Cat2 ED threshold
  Config::SetDefault ("ns3::NrCat2LbtAccessManager::Cat2EDThreshold", DoubleValue (cat2EDThreshold));
  Config::SetDefault ("ns3::NrLbtAccessManager::Mcot", TimeValue (MilliSeconds(8)));
  Config::SetDefault ("ns3::NrPhyRxTrace::SimTag", StringValue(SimTag));
  //Config::SetDefault ("ns3::ThresholdPreambleDetectionModel::MinimumRssi",DoubleValue (-62));
  
  Config::SetDefault ("ns3::UdpSocket::RcvBufSize", UintegerValue(0xFFFFFFFF));
  Config::SetDefault ("ns3::ArpCache::PendingQueueSize", UintegerValue(0xFFFFFFFF));
  Config::SetDefault ("ns3::ArpCache::MaxRetries", UintegerValue(0xFFFFFFFF));

  Config::SetDefault ("ns3::FlowMonitor::MaxPerHopDelay", TimeValue(Seconds(1000)));
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

void readPathLoss (std::string pathlossDir, uint32_t numApStaPairs, NodeContainer* staNodes, NodeContainer* apNodes, Ptr<MatrixPropagationLossModel> pathloss, uint32_t simRound, uint32_t scenarioType, uint32_t scenarioId)
{
  uint32_t indexAp[18];
  double pathlossApAp[18][18];
  double pathlossApSta [18][18];
  double pathlossStaSta [18][18];

  std::stringstream filenameIndexAp, filenameApAp, filenameApSta, filenameStaSta;

  if (scenarioType == 1)
        {
            if (scenarioId == 1)
            {
                filenameApAp << pathlossDir << "Scenario1/pathlossMatrixGnbGnb.txt";
                filenameApSta << pathlossDir << "Scenario1/pathlossMatrixGnbUe_" << simRound << ".txt";
                filenameIndexAp << pathlossDir << "Scenario1/apIndex_" << simRound << ".txt";
                filenameStaSta << pathlossDir << "Scenario1/pathlossMatrixUeUe_" << simRound << ".txt";
            }
            if (scenarioId == 2)
            {
                filenameApAp << pathlossDir << "Scenario2/pathlossMatrixGnbGnb.txt";
                filenameApSta << pathlossDir << "Scenario2/pathlossMatrixGnbUe_" << simRound << ".txt";
                filenameIndexAp << pathlossDir << "Scenario2/apIndex_" << simRound << ".txt";
                filenameStaSta << pathlossDir << "Scenario2/pathlossMatrixUeUe_" << simRound << ".txt";
            }
            if (scenarioId == 3)
            {
                filenameApAp << pathlossDir << "Scenario3/pathlossMatrixGnbGnb_" << simRound << ".txt";
                filenameApSta << pathlossDir << "Scenario3/pathlossMatrixGnbUe_" << simRound << ".txt";
                filenameIndexAp << pathlossDir << "Scenario3/apIndex_" << simRound << ".txt";
                filenameStaSta << pathlossDir << "Scenario3/pathlossMatrixUeUe_" << simRound << ".txt";
            }
            if (scenarioId == 4)
            {
                filenameApAp << pathlossDir << "Scenario4/pathlossMatrixGnbGnb.txt";
                filenameApSta << pathlossDir << "Scenario4/pathlossMatrixGnbUe_" << simRound << ".txt";
                filenameIndexAp << pathlossDir << "Scenario4/apIndex_" << simRound << ".txt";
                filenameStaSta << pathlossDir << "Scenario4/pathlossMatrixUeUe_" << simRound << ".txt";
            }
        }
        if (scenarioType == 2)
        {
            if (scenarioId == 1)
            {
                filenameApAp << pathlossDir << "onlyLOS/pathlossMatrixGnbGnb.txt";
                filenameApSta << pathlossDir << "onlyLOS/pathlossMatrixGnbUe_" << simRound << ".txt";
                filenameIndexAp << pathlossDir << "onlyLOS/apIndex_" << simRound << ".txt";
                filenameStaSta << pathlossDir << "onlyLOS/pathlossMatrixUeUe_" << simRound << ".txt";
            }
            if (scenarioId == 2)
            {
                filenameApAp << pathlossDir << "notOnlyLOS/pathlossMatrixGnbGnb.txt";
                filenameApSta << pathlossDir << "notOnlyLOS/pathlossMatrixGnbUe_" << simRound << ".txt";
                filenameIndexAp << pathlossDir << "notOnlyLOS/apIndex_" << simRound << ".txt";
                filenameStaSta << pathlossDir << "notOnlyLOS/pathlossMatrixUeUe_" << simRound << ".txt";
            }
        }

  std::ifstream inFile;
    
    inFile.open(filenameIndexAp.str(), std::ios_base::in);
    if (!inFile.is_open ()) 
    { 
        NS_LOG_ERROR ("Can't open file!!!" ); 
        return ;
    }
    std::cout << "AP Index : ";
    for (uint32_t i = 0; i < 18; i++)
    { 
        NS_ABORT_MSG_UNLESS (!inFile.eof(), "End of file!!!");
        inFile >> indexAp[i];
        std::cout << indexAp[i] << " ";
	}  
    std::cout << std::endl;
    inFile.close();

    inFile.open(filenameApAp.str(), std::ios_base::in);
    if (!inFile.is_open ()) 
    { 
        NS_LOG_ERROR ("Can't open file!!!" ); 
        return ;
    }
    for (uint32_t i = 0; i < 18; i++)
    { 
        for (uint32_t j = 0; j < 18; j++)
        	{
         	    NS_ABORT_MSG_UNLESS (!inFile.eof(), "End of file!!!");
         	    inFile >> pathlossApAp[i][j];
		    }
	}  
    inFile.close();  

    inFile.open(filenameApSta.str(), std::ios_base::in);
    if (!inFile.is_open ()) 
    { 
        NS_LOG_ERROR ("Can't open file!!!" ); 
        return ;
    }
    for (uint32_t i = 0; i < 18; i++)
    { 
        for (uint32_t j = 0; j < 18; j++)
        	{
         	    NS_ABORT_MSG_UNLESS (!inFile.eof(), "End of file!!!");
         	    inFile >> pathlossApSta[i][j];
		    }
	}  
    inFile.close();  

    inFile.open(filenameStaSta.str(), std::ios_base::in);
    if (!inFile.is_open ()) 
    { 
        NS_LOG_ERROR ("Can't open file!!!" ); 
        return ;
    }
    for (uint32_t i = 0; i < 18; i++)
    { 
        for (uint32_t j = 0; j < 18; j++)
        	{
         	    NS_ABORT_MSG_UNLESS (!inFile.eof(), "End of file!!!");
         	    inFile >> pathlossStaSta[i][j];
		    }
	}  
    inFile.close();
  
    //Set path loss between APs-STAs

    for (uint16_t i = 0; i < apNodes->GetN(); i++)
    {
        for (uint16_t j = 0; j < staNodes->GetN(); j++)	
		{   
            std::cout << pathlossApSta [indexAp[i]-1][indexAp[j]-1] << ", ";
            pathloss -> SetLoss (apNodes -> Get(i)->GetObject<MobilityModel> (), staNodes ->Get (j) ->GetObject<MobilityModel> (), pathlossApSta[indexAp[i]-1][indexAp[j]-1]);
        }
        std::cout << std::endl;
    }

    //Set path loss between APs-APs
    for (uint16_t i = 0; i < apNodes->GetN (); i++)
    {
        for (uint16_t j = 0; j< apNodes->GetN (); j++)	
		{
            if (j != i)
            {
                std::cout << pathlossApAp [indexAp[i]-1][indexAp[j]-1] << ", ";
                pathloss -> SetLoss (apNodes -> Get(i)->GetObject<MobilityModel> (), apNodes ->Get (j) ->GetObject<MobilityModel> (), pathlossApAp[indexAp[i]-1][indexAp[j]-1]);
            }
            else 
            {
                std::cout << "0.0, ";
            }
        }
        std::cout << std::endl;
    }

    //Set path loss between STAs-STAs
    for (uint16_t i = 0; i < staNodes->GetN (); i++)
    {
        for (uint16_t j = 0; j < staNodes->GetN (); j++)	
		{
            if (i != j)
            {
                std::cout << pathlossStaSta [indexAp[i]-1][indexAp[j]-1] << ", ";
                pathloss -> SetLoss (staNodes -> Get(i)->GetObject<MobilityModel> (), staNodes ->Get (j) ->GetObject<MobilityModel> (), pathlossStaSta[indexAp[i]-1][indexAp[j]-1]);
            }
            else 
            {
                std::cout << "0.0, ";
            }
        }
        std::cout << std::endl;
    }
}

//Global values
std::vector<uint32_t> g_nodeIndex_popA;
std::vector<uint32_t> g_nodeIndex_popB;
int numb_of_ue;
std::vector<std::string> ipAddrForSinr;
std::vector<double> sinrForDataPack;
std::vector<uint32_t> rxNodeForSinr;

std::vector<std::string> MCS_WIFI;
std::vector<std::string> ipadd_get;
std::vector<uint32_t> node_num;
std::vector<int> numaggrpacket;
uint32_t num_ofpacket=0;

std::string
AddressToString (const Address &addr)
{
  std::stringstream addressStr;
  addressStr << InetSocketAddress::ConvertFrom (addr).GetIpv4 () << ":"
             << InetSocketAddress::ConvertFrom (addr).GetPort ();
  return addressStr.str ();
}

void
FragmentTx (Ptr<const Packet> fragment, const Address &from, const Address &to,
            const SeqTsSizeFragHeader &header)
{
  NS_LOG_INFO ("Sent fragment " << header.GetFragSeq () << "/" << header.GetFrags ()
                                << " of burst seq=" << header.GetSeq ()
                                << " of header.GetSize ()=" << header.GetSize ()
                                << " (fragment->GetSize ()=" << fragment->GetSize ()
                                << ") bytes from " << AddressToString (from) << " to "
                                << AddressToString (to) << " at " << header.GetTs ().As (Time::S));
}

void
FragmentRx (Ptr<const Packet> fragment, const Address &from, const Address &to,
            const SeqTsSizeFragHeader &header)
{
  NS_LOG_INFO ("Received fragment "
               << header.GetFragSeq () << "/" << header.GetFrags () << " of burst seq="
               << header.GetSeq () << " of header.GetSize ()=" << header.GetSize ()
               << " (fragment->GetSize ()=" << fragment->GetSize () << ") bytes from "
               << AddressToString (from) << " to " << AddressToString (to) << " at "
               << header.GetTs ().As (Time::S));
}

void
BurstTx (Ptr<const Packet> burst, const Address &from, const Address &to,
         const SeqTsSizeFragHeader &header)
{
  NS_LOG_INFO ("Sent burst seq=" << header.GetSeq () << " of header.GetSize ()="
                                 << header.GetSize () << " (burst->GetSize ()=" << burst->GetSize ()
                                 << ") bytes from " << AddressToString (from) << " to "
                                 << AddressToString (to) << " at " << header.GetTs ().As (Time::S));
}

void
BurstRx (Ptr<const Packet> burst, const Address &from, const Address &to,
         const SeqTsSizeFragHeader &header)
{
  NS_LOG_INFO ("Received burst seq="
               << header.GetSeq () << " of header.GetSize ()=" << header.GetSize ()
               << " (burst->GetSize ()=" << burst->GetSize () << ") bytes from "
               << AddressToString (from) << " to " << AddressToString (to) << " at "
               << header.GetTs ().As (Time::S));
}

ApplicationContainer setUdpServer (NodeContainer serverNodes, uint16_t port)
{
    ApplicationContainer serverApps;

    UdpServerHelper udpServer (port);
    serverApps.Add(udpServer.Install (serverNodes));

    return serverApps;
}

ApplicationContainer setUdpPoissonServer (NodeContainer serverNodes, uint16_t port)
{
    ApplicationContainer serverApps;

    UdpServerHelper udpServer (port);
    serverApps.Add(udpServer.Install (serverNodes));

    return serverApps;
}

ApplicationContainer setOnOffServer (NodeContainer serverNodes, uint16_t port)
{

    ApplicationContainer serverApps;

    PacketSinkHelper onoffServer ("ns3::UdpSocketFactory", Address (InetSocketAddress (Ipv4Address::GetAny (), port)));
    serverApps.Add(onoffServer.Install (serverNodes));

    return serverApps;
}

ApplicationContainer setTcpServer (NodeContainer serverNodes, uint16_t port, Ipv4InterfaceContainer remoteIpIface)
{

    ApplicationContainer serverApps;

    PacketSinkHelper tcpServer ("ns3::TcpSocketFactory", Address (InetSocketAddress (remoteIpIface.GetAddress (0,0), port)));
    serverApps.Add(tcpServer.Install (serverNodes));

    return serverApps;
}

ApplicationContainer setBurstServer (NodeContainer serverNodes, uint16_t port, Ipv4InterfaceContainer serverIpIface, uint32_t fragmentSize, uint32_t frameRate, std::string dataRate, NodeContainer remoteHostContainer)
{

    ApplicationContainer serverApps;

    for (uint32_t i = 0; i < serverNodes.GetN() ; i++)
    {
        BurstyHelper burstyServer ("ns3::UdpSocketFactory", InetSocketAddress (serverIpIface.GetAddress (i, 0), port));
        burstyServer.SetAttribute ("FragmentSize", UintegerValue (fragmentSize));
        burstyServer.SetBurstGenerator ("ns3::VrBurstGenerator",
                                  "FrameRate", DoubleValue (frameRate),
                                  "TargetDataRate", DataRateValue (DataRate (dataRate)));
        serverApps.Add(burstyServer.Install (remoteHostContainer));

        Ptr<BurstyApplication> burstyApp = serverApps.Get (i)->GetObject<BurstyApplication> ();
        burstyApp->TraceConnectWithoutContext ("FragmentTx", MakeCallback (&FragmentTx));
        burstyApp->TraceConnectWithoutContext ("BurstTx", MakeCallback (&BurstTx)); 
    } 
    
    return serverApps;
}

ApplicationContainer setBurstServerAi (NodeContainer serverNodes, uint16_t port, Ipv4InterfaceContainer serverIpIface, uint32_t fragmentSize, uint32_t frameRate, std::string dataRate, NodeContainer remoteHostContainer, double startTime)
{

    ApplicationContainer serverApps;
    Ptr<UniformRandomVariable> randomVariable = CreateObject<UniformRandomVariable> ();

    for (uint32_t i = 0; i < serverNodes.GetN() ; i++)
    {
        BurstyHelper burstyServer ("ns3::UdpSocketFactory", InetSocketAddress (serverIpIface.GetAddress (i, 0), port));
        burstyServer.SetAttribute ("FragmentSize", UintegerValue (fragmentSize));
        double clientStartTime = randomVariable->GetValue (startTime, startTime + 0.01); 
        burstyServer.SetAttribute ("StartTime", TimeValue (Seconds (clientStartTime)));
        burstyServer.SetBurstGenerator ("ns3::VrBurstGenerator",
                                  "FrameRate", DoubleValue (frameRate),
                                  "TargetDataRate", DataRateValue (DataRate (dataRate)));
        serverApps.Add(burstyServer.Install (remoteHostContainer));

        Ptr<BurstyApplication> burstyApp = serverApps.Get (i)->GetObject<BurstyApplication> ();
        //burstyApp->TraceConnectWithoutContext ("FragmentTx", MakeCallback (&FragmentTx));
        //burstyApp->TraceConnectWithoutContext ("BurstTx", MakeCallback (&BurstTx)); 
    } 
    
    return serverApps;
}

ApplicationContainer setNgmnFtpServer (NodeContainer serverNodes, uint16_t port, std::string transportProtocol)
{
    ApplicationContainer serverApps;

    InetSocketAddress localAddress(Ipv4Address::GetAny(), port);
    PacketSinkHelper packetSinkHelper(transportProtocol, localAddress);

    for (uint32_t index = 0; index < serverNodes.GetN(); index++)
    {
            serverApps.Add(packetSinkHelper.Install(serverNodes.Get(index)));
    }

    return serverApps;
}

ApplicationContainer setNgmnVideoServer (NodeContainer serverNodes, uint16_t port, std::string transportProtocol)
{
    ApplicationContainer serverApps;

    InetSocketAddress localAddress(Ipv4Address::GetAny(), port);
    PacketSinkHelper packetSinkHelper(transportProtocol, localAddress);

    for (uint32_t index = 0; index < serverNodes.GetN(); index++)
    {
            Ptr<PacketSink> ps = packetSinkHelper.Install(serverNodes.Get(index)).Get(0)->GetObject<PacketSink>();
            serverApps.Add(ps);
    }

    return serverApps;
}

ApplicationContainer setNgmnGamingServer (NodeContainer serverNodes, uint16_t port, std::string transportProtocol)
{
    ApplicationContainer serverApps;

    InetSocketAddress localAddress(Ipv4Address::GetAny(), port);
    PacketSinkHelper packetSinkHelper(transportProtocol, localAddress);

    for (uint32_t index = 0; index < serverNodes.GetN(); index++)
    {
            serverApps.Add(packetSinkHelper.Install(serverNodes.Get(index)));
    }

    return serverApps;
}

ApplicationContainer setNgmnVoipServer (NodeContainer serverNodes, uint16_t port, std::string transportProtocol)
{
    ApplicationContainer serverApps;

    InetSocketAddress localAddress(Ipv4Address::GetAny(), port);
    PacketSinkHelper packetSinkHelper(transportProtocol, localAddress);

    for (uint32_t index = 0; index < serverNodes.GetN(); index++)
    {
            serverApps.Add(packetSinkHelper.Install(serverNodes.Get(index)));
    }

    return serverApps;
}

ApplicationContainer setUdpClient (NodeContainer clientNodes, NodeContainer remoteHostContainer, Ipv4InterfaceContainer serverIpIface, uint16_t port, Time interval, double startTime, uint32_t packetSize)
{
    ApplicationContainer clientApps;
    Ptr<UniformRandomVariable> randomVariable = CreateObject<UniformRandomVariable> ();

    for (uint32_t i = 0; i < clientNodes.GetN (); i++)
    {
        UdpClientHelper clientHelper (Address ((InetSocketAddress (serverIpIface.GetAddress (i), port))));
        clientHelper.SetAttribute ("MaxPackets", UintegerValue (0xFFFFFFFF));
        clientHelper.SetAttribute ("Interval", TimeValue (interval/2));
        double clientStartTime = randomVariable->GetValue (startTime, startTime + 0.01); 
        clientHelper.SetAttribute ("StartTime", TimeValue (Seconds (clientStartTime)));
        clientHelper.SetAttribute ("PacketSize", UintegerValue (packetSize));
        clientHelper.SetAttribute ("RemotePort", UintegerValue(port));
        Ipv4Address ip = serverIpIface.GetAddress (i, 0);
        clientHelper.SetAttribute ("RemoteAddress", AddressValue (ip));
        clientApps.Add (clientHelper.Install (remoteHostContainer));

        std::cout << "Installing app to transmit data to node "
                  << serverIpIface.GetAddress (i, 0)
                  << ":" << port << " at time " << clientStartTime
                  << " s " << std::endl;
    }

    return clientApps;
}

ApplicationContainer setUdpPoissonClient (NodeContainer clientNodes, NodeContainer remoteHostContainer, Ptr<Node> remoteHost, Ipv4InterfaceContainer serverIpIface, uint16_t port, uint32_t udpLambda, uint32_t packetSize, double startTime)
{
    ApplicationContainer clientApps;
    Ptr<UniformRandomVariable> randomVariable = CreateObject<UniformRandomVariable> ();

    UdpClientHelper dlClientLowLat;
    dlClientLowLat.SetAttribute("RemotePort", UintegerValue(port));
    dlClientLowLat.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
    dlClientLowLat.SetAttribute("PacketSize", UintegerValue(packetSize));

    EpsBearer lowLatBearer(EpsBearer::NGBR_VIDEO_TCP_DEFAULT);

    Ptr<EpcTft> lowLatTft = Create<EpcTft>();
    EpcTft::PacketFilter dlpfLowLat;

    dlpfLowLat.localPortStart = port;
    dlpfLowLat.localPortEnd = port;
    dlpfLowLat.direction = EpcTft::DOWNLINK;

    lowLatTft->Add(dlpfLowLat);

    for (uint32_t i = 0; i < clientNodes.GetN (); i++)
    {
        double clientStartTime = randomVariable->GetValue (startTime, startTime + 0.01);
        dlClientLowLat.SetAttribute ("StartTime", TimeValue (Seconds (clientStartTime)));
        dlClientLowLat.SetAttribute("Interval",TimeValue(Seconds(1.0 / udpLambda)));
        Address ueAddress = serverIpIface.GetAddress(i);

        dlClientLowLat.SetAttribute("RemoteAddress", AddressValue(ueAddress));
        clientApps.Add(dlClientLowLat.Install(remoteHost));
        
        std::cout << "Installing app to transmit data to node "
                  << serverIpIface.GetAddress (i, 0)
                  << ":" << port << " at time " << clientStartTime
                  << " s " << std::endl;

    }
    
    return clientApps;
}

ApplicationContainer setUdpPoissonClientAi (NodeContainer clientNodes, NodeContainer remoteHostContainer, Ptr<Node> remoteHost, Ipv4InterfaceContainer serverIpIface, uint16_t port, uint32_t udpLambda, uint32_t packetSize, double startTime)
{
    ApplicationContainer clientApps;
    Ptr<UniformRandomVariable> randomVariable = CreateObject<UniformRandomVariable> ();

    UdpClientHelper dlClientLowLat;
    dlClientLowLat.SetAttribute("RemotePort", UintegerValue(port));
    dlClientLowLat.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
    dlClientLowLat.SetAttribute("PacketSize", UintegerValue(packetSize));

    EpsBearer lowLatBearer(EpsBearer::NGBR_VIDEO_TCP_DEFAULT);

    Ptr<EpcTft> lowLatTft = Create<EpcTft>();
    EpcTft::PacketFilter dlpfLowLat;

    dlpfLowLat.localPortStart = port;
    dlpfLowLat.localPortEnd = port;
    dlpfLowLat.direction = EpcTft::DOWNLINK;

    lowLatTft->Add(dlpfLowLat);

    for (uint32_t i = 0; i < clientNodes.GetN (); i++)
    {
        double clientStartTime = randomVariable->GetValue (startTime, startTime + 0.01);
        dlClientLowLat.SetAttribute ("StartTime", TimeValue (Seconds (clientStartTime)));
        dlClientLowLat.SetAttribute("Interval",TimeValue(Seconds(1.0 / udpLambda)));
        Address ueAddress = serverIpIface.GetAddress(i);

        dlClientLowLat.SetAttribute("RemoteAddress", AddressValue(ueAddress));
        clientApps.Add(dlClientLowLat.Install(remoteHost));
        
        std::cout << "Installing app to transmit data to node "
                  << serverIpIface.GetAddress (i, 0)
                  << ":" << port << " at time " << clientStartTime
                  << " s " << std::endl;
    }

    return clientApps;
}

ApplicationContainer setOnOffClient (NodeContainer clientNodes, NodeContainer remoteHostContainer, Ipv4InterfaceContainer serverIpIface, uint16_t port, double startTime, uint32_t packetSize, std::string nodeRate)
{
    ApplicationContainer clientApps;
    Ptr<UniformRandomVariable> randomVariable = CreateObject<UniformRandomVariable> ();

    for (uint32_t i = 0; i < clientNodes.GetN (); i++)
    {
        OnOffHelper clientHelper ("ns3::UdpSocketFactory", Address ((InetSocketAddress (serverIpIface.GetAddress (i, 0), port))));
        clientHelper.SetConstantRate (DataRate (nodeRate), packetSize);
        ApplicationContainer app = clientHelper.Install (remoteHostContainer);
        double clientStartTime = randomVariable->GetValue (startTime, startTime + 0.01); 
        app.Start (Seconds(startTime));
        //app.Stop (Seconds (startTime + 1.0));
        clientApps.Add (app);

        std::cout << "Installing app to transmit data to node "
                  << serverIpIface.GetAddress (i, 0)
                  << ":" << port << " at time " << clientStartTime
                  << " s " << std::endl;
    }

    return clientApps;
}

ApplicationContainer setTcpClient (NodeContainer clientNodes, NodeContainer remoteHostContainer, Ipv4InterfaceContainer serverIpIface, uint16_t port, double startTime, uint32_t packetSize, std::string nodeRate)
{
    ApplicationContainer clientApps;
    Ptr<UniformRandomVariable> randomVariable = CreateObject<UniformRandomVariable> ();

    for (uint32_t i = 0; i < clientNodes.GetN (); i++)
    {
        OnOffHelper clientHelper ("ns3::TcpSocketFactory", Address ((InetSocketAddress (serverIpIface.GetAddress (i, 0), port))));
        clientHelper.SetConstantRate (DataRate (nodeRate), packetSize);
        ApplicationContainer app = clientHelper.Install (remoteHostContainer);
        double clientStartTime = randomVariable->GetValue (startTime, startTime + 0.01); 
        app.Start (Seconds(startTime));
        //app.Stop (Seconds (startTime + 1.0));
        clientApps.Add (app);

        std::cout << "Installing app to transmit data to node "
                  << serverIpIface.GetAddress (i, 0)
                  << ":" << port << " at time " << clientStartTime
                  << " s " << std::endl;
    }

    return clientApps;
}

ApplicationContainer setBurstClient (NodeContainer remoteHostContainer, Ipv4InterfaceContainer remoteIpIface, NodeContainer  clientNodes, uint16_t port)
{
    ApplicationContainer clientApps;
    Ptr<UniformRandomVariable> randomVariable = CreateObject<UniformRandomVariable> ();

    for (uint32_t i = 0; i < remoteHostContainer.GetN (); i++)
    {
        BurstSinkHelper burstSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (remoteIpIface.GetAddress (i, 0), port));
        ApplicationContainer app = burstSinkHelper.Install (clientNodes);

        clientApps.Add(app);
        Ptr<BurstSink> burstSink = clientApps.Get (i)->GetObject<BurstSink> ();
        burstSink->TraceConnectWithoutContext ("BurstRx", MakeCallback (&BurstRx));
        burstSink->TraceConnectWithoutContext ("FragmentRx", MakeCallback (&FragmentRx)); 
    }

    return clientApps;
}

ApplicationContainer setNgmnFtpClient (NodeContainer clientNodes, NodeContainer remoteHostContainer, Ipv4InterfaceContainer serverIpIface, uint16_t port, uint32_t maxFileSize, uint32_t packetSize, std::string transportProtocol)
{
    ApplicationContainer clientApps;
    ApplicationContainer pingApps;

    Ptr<UniformRandomVariable> randomVariable = CreateObject<UniformRandomVariable> ();

    TrafficGeneratorHelper ftpHelper(transportProtocol, Address(), TrafficGeneratorNgmnFtpMulti::GetTypeId());
    ftpHelper.SetAttribute("PacketSize", UintegerValue(packetSize));
    ftpHelper.SetAttribute("MaxFileSize", UintegerValue(maxFileSize));

    for (uint32_t i = 0; i < clientNodes.GetN (); i++)
    {
        Ipv4Address ipAddress = serverIpIface.GetAddress(i, 0);
        AddressValue ueAddress(InetSocketAddress(ipAddress, port));
        ftpHelper.SetAttribute("Remote", ueAddress);
        clientApps.Add(ftpHelper.Install(remoteHostContainer));
        // Seed the ARP cache by pinging early in the simulation
        // This is a workaround until a static ARP capability is provided
        V4PingHelper ping(ipAddress);
        pingApps.Add(ping.Install(remoteHostContainer)); 
    }

    return clientApps;
}

ApplicationContainer setNgmnVideoClient (NodeContainer clientNodes, NodeContainer remoteHostContainer, Ipv4InterfaceContainer serverIpIface, uint16_t port, Time interval, uint32_t packetSize, std::string transportProtocol)
{
    ApplicationContainer clientApps;
    ApplicationContainer pingApps;

    Ptr<UniformRandomVariable> randomVariable = CreateObject<UniformRandomVariable> ();

    TrafficGeneratorHelper trafficGeneratorHelper(transportProtocol, Address(), TrafficGeneratorNgmnVideo::GetTypeId());
    trafficGeneratorHelper.SetAttribute("NumberOfPacketsInFrame", UintegerValue(packetSize));
    trafficGeneratorHelper.SetAttribute("InterframeIntervalTime", TimeValue(interval));

    for (uint32_t i = 0; i < clientNodes.GetN (); i++)
    {
        Ipv4Address ipAddress = serverIpIface.GetAddress(i, 0);
        AddressValue remoteAddress(InetSocketAddress(ipAddress, port));
        trafficGeneratorHelper.SetAttribute("Remote", remoteAddress);
        clientApps.Add(trafficGeneratorHelper.Install(remoteHostContainer));
        // Seed the ARP cache by pinging early in the simulation
        // This is a workaround until a static ARP capability is provided
        V4PingHelper ping(ipAddress);
        pingApps.Add(ping.Install(remoteHostContainer)); 
    }

    return clientApps;
}

ApplicationContainer setNgmnGamingClient (NodeContainer clientNodes, NodeContainer remoteHostContainer, Ipv4InterfaceContainer serverIpIface, uint16_t port, std::string transportProtocol)
{
    ApplicationContainer clientApps;
    ApplicationContainer pingApps;

    Ptr<UniformRandomVariable> randomVariable = CreateObject<UniformRandomVariable> ();

    TrafficGeneratorHelper trafficGeneratorHelper(transportProtocol, Address(), TrafficGeneratorNgmnGaming::GetTypeId());
    trafficGeneratorHelper.SetAttribute("IsDownlink", BooleanValue(true));
    trafficGeneratorHelper.SetAttribute("aParamPacketSizeDl", UintegerValue(120));
    trafficGeneratorHelper.SetAttribute("bParamPacketSizeDl", DoubleValue(36));
    trafficGeneratorHelper.SetAttribute("aParamPacketArrivalDl", DoubleValue(45));
    trafficGeneratorHelper.SetAttribute("bParamPacketArrivalDl", DoubleValue(5.7));
    trafficGeneratorHelper.SetAttribute("InitialPacketArrivalMin", UintegerValue(0));
    trafficGeneratorHelper.SetAttribute("InitialPacketArrivalMax", UintegerValue(40));

    for (uint32_t i = 0; i < clientNodes.GetN (); i++)
    {
        Ipv4Address ipAddress = serverIpIface.GetAddress(i, 0);
        AddressValue remoteAddress(InetSocketAddress(ipAddress, port));
        trafficGeneratorHelper.SetAttribute("Remote", remoteAddress);
        clientApps.Add(trafficGeneratorHelper.Install(remoteHostContainer));
        // Seed the ARP cache by pinging early in the simulation
        // This is a workaround until a static ARP capability is provided
        V4PingHelper ping(ipAddress);
        pingApps.Add(ping.Install(remoteHostContainer)); 
    }

    return clientApps;
}

ApplicationContainer setNgmnVoipClient (NodeContainer clientNodes, NodeContainer remoteHostContainer, Ipv4InterfaceContainer serverIpIface, uint16_t port, std::string transportProtocol)
{
    ApplicationContainer clientApps;
    ApplicationContainer pingApps;

    Ptr<UniformRandomVariable> randomVariable = CreateObject<UniformRandomVariable> ();

    TrafficGeneratorHelper trafficGeneratorHelper(transportProtocol, Address(), TrafficGeneratorNgmnVoip::GetTypeId());
    trafficGeneratorHelper.SetAttribute("EncoderFrameLength", UintegerValue(20));
    trafficGeneratorHelper.SetAttribute("MeanTalkSpurtDuration", UintegerValue(2000));
    trafficGeneratorHelper.SetAttribute("VoiceActivityFactor", DoubleValue(0.5));
    trafficGeneratorHelper.SetAttribute("VoicePayload", UintegerValue(40));
    trafficGeneratorHelper.SetAttribute("SIDPeriodicity", UintegerValue(160));
    trafficGeneratorHelper.SetAttribute("SIDPayload", UintegerValue(15));

    for (uint32_t i = 0; i < clientNodes.GetN (); i++)
    {
        Ipv4Address ipAddress = serverIpIface.GetAddress(i, 0);
        AddressValue remoteAddress(InetSocketAddress(ipAddress, port));
        trafficGeneratorHelper.SetAttribute("Remote", remoteAddress);
        clientApps.Add(trafficGeneratorHelper.Install(remoteHostContainer));
        // Seed the ARP cache by pinging early in the simulation
        // This is a workaround until a static ARP capability is provided
        V4PingHelper ping(ipAddress);
        pingApps.Add(ping.Install(remoteHostContainer)); 
    }

    return clientApps;
}

bool g_connect = false;
Ptr<NrMacTimeStepEnv> g_env;
std::vector<std::pair<uint32_t, double>> g_throughput(4);
std::vector<std::pair<uint32_t, double>> g_delay(4);
std::vector<std::pair<uint32_t, double>> g_jitter(4);
std::vector<uint32_t> g_macType(2);
std::vector<uint32_t> g_trafficType(2);
Time g_timeStep = MilliSeconds(500);
Time g_timeStepMac = MilliSeconds(500);
std::vector<L2Setup*> nr(4);
std::vector<double> g_throughputDiff(2);
std::vector<double> g_jitterDiff(2);
std::vector<double> g_delayDiff(2);
double g_averageThroughput;
double g_averageDelay;
double g_averageJitter;
std::vector<uint32_t> g_mcot(2);
std::vector<uint32_t> g_minCw(2);

ApplicationContainer g_nruServerNodes;
ApplicationContainer g_nruClientNodes;
std::vector<uint32_t> g_udpLambda(2);
std::vector<uint32_t> g_packetSize(2);

void 
CreateEnv()
{
  g_env = CreateObject<NrMacTimeStepEnv> ();
}

void
GiveThroughputAlt (FlowMonitorHelper *flowHelper, Ptr<FlowMonitor> flowMonitor, uint32_t numNruPairs)
{
  if(!g_connect)
  {
    g_connect = true;
    CreateEnv();
  }
  
  flowMonitor->SetAttribute("DelayBinWidth", DoubleValue(0.001));
  flowMonitor->SetAttribute("JitterBinWidth", DoubleValue(0.001));
  flowMonitor->SetAttribute("PacketSizeBinWidth", DoubleValue(20));

  flowMonitor->CheckForLostPackets();

  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowHelper->GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = flowMonitor->GetFlowStats ();

  uint32_t vecNum = 0;
  std::vector<std::pair<uint32_t, double>> newThroughput(numNruPairs);
  std::vector<std::pair<uint32_t, double>> newJitter(numNruPairs);
  std::vector<std::pair<uint32_t, double>> newDelay(numNruPairs);

  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
  {
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
    
    std::cout << "Flow " << i->first << " (" << t.sourceAddress << ":" << t.sourcePort << " -> " << t.destinationAddress << ":" << t.destinationPort << ")" << std::endl;
        
        std::cout << "  Tx Packets: " << i->second.txPackets << "\n";
        std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
        std::cout << "  Tx Offered:  "
                  << i->second.txBytes * 8.0 / 1000.0 /
                         1000.0
                  << " Mbps\n";
        std::cout << "  Rx Packets: " << i->second.rxPackets << "\n";
        std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
        std::cout << "  Lost Packets: " << i->second.lostPackets << "\n";
        
    double throughputMbps = 0;
    double jitter = 0;
    double delay = 0;
    if (i->second.rxPackets > 0)
    {
      throughputMbps = i->second.rxBytes * 8.0 / g_timeStep.GetSeconds() / 1e6;
      std::cout << "  Throughput : "<< throughputMbps << " mbps" << std::endl;
    }
    if (i->second.rxPackets > 1)
    {
      delay = i->second.delaySum.GetMicroSeconds () / i->second.rxPackets;
      std::cout << "  Mean delay : "<< delay << " microseconds" << std::endl;
      jitter = i->second.jitterSum.GetMicroSeconds () / (i->second.rxPackets -1);
      std::cout << "  Mean jitter : "<< jitter << " microseconds" << std::endl;
      
      newThroughput[vecNum] = std::make_pair(t.destinationAddress.Get (), throughputMbps);
      newDelay[vecNum] = std::make_pair(t.destinationAddress.Get (), delay);
      newJitter[vecNum] = std::make_pair(t.destinationAddress.Get (), jitter);
    }
    vecNum++;
  } 
  
  double throughputDiff;
  double delayDiff;
  double jitterDiff;
  if (g_trafficType[0] == 0)
  {
    for (uint32_t j = 0; j < newThroughput.size(); j++)
    {
        if (newThroughput[j].first == 117440514)
        {
            for (uint32_t i = 0; i < g_throughput.size(); i++)
            {
                if (g_throughput[i].first == 117440514)
                {
                    throughputDiff = newThroughput[j].second - g_throughput[i].second;
                    delayDiff = newDelay[j].second - g_delay[i].second;
                    jitterDiff = newJitter[j].second - g_jitter[i].second;
                    g_throughputDiff[0] = throughputDiff;
                    g_delayDiff[0] = delayDiff;
                    g_jitterDiff[0] = jitterDiff;
                    std::cout << "Throughput diff agent 0 : " << g_throughputDiff[0] << std::endl;
                    std::cout << "Delay diff agent 0 : " << g_delayDiff[0] << std::endl;
                    std::cout << "Jitter diff agent 0 : " << g_jitterDiff[0] << std::endl;
                    goto secondLoop;
                }
            }
            throughputDiff = newThroughput[j].second;
            delayDiff = newDelay[j].second;
            jitterDiff = newJitter[j].second;
            g_throughputDiff[0] = throughputDiff;
            g_delayDiff[0] = delayDiff;
            g_jitterDiff[0] = jitterDiff;
            std::cout << "Throughput diff agent 0 : " << g_throughputDiff[0] << std::endl;
            std::cout << "Delay diff agent 0 : " << g_delayDiff[0] << std::endl;
            std::cout << "Jitter diff agent 0 : " << g_jitterDiff[0] << std::endl;
            goto secondLoop;
        }
    }
  }
  else if (g_trafficType[0] == 1)
  {
    for (uint32_t j = 0; j < newThroughput.size(); j++)
    {
        if (newThroughput[j].first == 117440515)
        {
            for (uint32_t i = 0; i < g_throughput.size(); i++)
            {
                if (g_throughput[i].first == 117440515)
                {
                    throughputDiff = newThroughput[j].second - g_throughput[i].second;
                    delayDiff = newDelay[j].second - g_delay[i].second;
                    jitterDiff = newJitter[j].second - g_jitter[i].second;
                    g_throughputDiff[0] = throughputDiff;
                    g_delayDiff[0] = delayDiff;
                    g_jitterDiff[0] = jitterDiff;
                    std::cout << "Throughput diff agent 0 : " << g_throughputDiff[0] << std::endl;
                    std::cout << "Delay diff agent 0 : " << g_delayDiff[0] << std::endl;
                    std::cout << "Jitter diff agent 0 : " << g_jitterDiff[0] << std::endl;
                    goto secondLoop;
                }
            }
            throughputDiff = newThroughput[j].second;
            delayDiff = newDelay[j].second;
            jitterDiff = newJitter[j].second;
            g_throughputDiff[0] = throughputDiff;
            g_delayDiff[0] = delayDiff;
            g_jitterDiff[0] = jitterDiff;
            std::cout << "Throughput diff agent 0 : " << g_throughputDiff[0] << std::endl;
            std::cout << "Delay diff agent 0 : " << g_delayDiff[0] << std::endl;
            std::cout << "Jitter diff agent 0 : " << g_jitterDiff[0] << std::endl;
            goto secondLoop;
        }
    }
  }

secondLoop :

  if (g_trafficType[1] == 0)
  {
    for (uint32_t j = 0; j < newThroughput.size(); j++)
    {
        if (newThroughput[j].first == 117440516)
        {
            for (uint32_t i = 0; i < g_throughput.size(); i++)
            {
                if (g_throughput[i].first == 117440516)
                {
                    throughputDiff = newThroughput[j].second - g_throughput[i].second;
                    delayDiff = newDelay[j].second - g_delay[i].second;
                    jitterDiff = newJitter[j].second - g_jitter[i].second;
                    g_throughputDiff[1] = throughputDiff;
                    g_delayDiff[1] = delayDiff;
                    g_jitterDiff[1] = jitterDiff;
                    std::cout << "Throughput diff agent 1 : " << g_throughputDiff[1] << std::endl;
                    std::cout << "Delay diff agent 0 : " << g_delayDiff[1] << std::endl;
                    std::cout << "Jitter diff agent 0 : " << g_jitterDiff[1] << std::endl;
                    goto endLoop;
                }
            }
            throughputDiff = newThroughput[j].second;
            delayDiff = newDelay[j].second;
            jitterDiff = newJitter[j].second;
            g_throughputDiff[1] = throughputDiff;
            g_delayDiff[1] = delayDiff;
            g_jitterDiff[1] = jitterDiff;
            std::cout << "Throughput diff agent 1 : " << g_throughputDiff[1] << std::endl;
            std::cout << "Delay diff agent 1 : " << g_delayDiff[1] << std::endl;
            std::cout << "Jitter diff agent 1 : " << g_jitterDiff[1] << std::endl;
            goto endLoop;
        }
    }
  }
  else if (g_trafficType[1] == 1)
  {
    for (uint32_t j = 0; j < newThroughput.size(); j++)
    {
        if (newThroughput[j].first == 117440517)
        {
            for (uint32_t i = 0; i < g_throughput.size(); i++)
            {
                if (g_throughput[i].first == 117440517)
                {
                    throughputDiff = newThroughput[j].second - g_throughput[i].second;
                    delayDiff = newDelay[j].second - g_delay[i].second;
                    jitterDiff = newJitter[j].second - g_jitter[i].second;
                    g_throughputDiff[1] = throughputDiff;
                    g_delayDiff[1] = delayDiff;
                    g_jitterDiff[1] = jitterDiff;
                    std::cout << "Throughput diff agent 1 : " << g_throughputDiff[1] << std::endl;
                    std::cout << "Delay diff agent 1 : " << g_delayDiff[1] << std::endl;
                    std::cout << "Jitter diff agent 1 : " << g_jitterDiff[1] << std::endl;
                    goto endLoop;
                }
            }
            throughputDiff = newThroughput[j].second;
            delayDiff = newDelay[j].second;
            jitterDiff = newJitter[j].second;
            g_throughputDiff[1] = throughputDiff;
            g_delayDiff[1] = delayDiff;
            g_jitterDiff[1] = jitterDiff;
            std::cout << "Throughput diff agent 1 : " << g_throughputDiff[1] << std::endl;
            std::cout << "Delay diff agent 1 : " << g_delayDiff[1] << std::endl;
            std::cout << "Jitter diff agent 1 : " << g_jitterDiff[1] << std::endl;
            goto endLoop;
        }
    }
  }

endLoop :
    std::cout << "Sending throughput to environment" << g_jitterDiff[1] << std::endl;
    g_env->GiveThroughput (g_throughputDiff[0],0);
    g_env->GiveThroughput (g_throughputDiff[1],1);

    g_throughput = newThroughput;
    g_delay = newDelay;
    g_jitter = newJitter;
    g_averageThroughput = 0;
    g_averageDelay = 0;
    g_averageJitter = 0;
    for (uint32_t i = 0; i < g_trafficType.size(); i++)
    {
        g_averageThroughput += g_throughputDiff[i];
        g_averageDelay += g_delayDiff[i];
        g_averageJitter += g_jitterDiff[i];
        g_throughputDiff[i] = 0; //reset
        g_delayDiff[i] = 0;
        g_jitterDiff[i] = 0;
    }
    g_averageThroughput = g_averageThroughput/g_trafficType.size();
    g_averageDelay = g_averageDelay/g_trafficType.size();
    g_averageJitter = g_averageJitter/g_trafficType.size();

    Simulator::Schedule (g_timeStep, &GiveThroughputAlt, flowHelper, flowMonitor, numNruPairs);
}

void
ChangeMacTypeAlt()
{
  std::cout << "Debug ChangeMacTypeAlt()" << std::endl;
  if (!g_connect)
  {
    g_connect = true;
    CreateEnv();
  }
  for (uint16_t i = 0; i < g_trafficType.size(); i++)
  {
    std::cout << "ChangeMacType() new Mac parameters with iteration i " << i << ": " << std::endl;
    uint32_t newMinCw = g_env->ChangeMinCw(g_minCw[i],i);
    uint32_t newMcot = g_env->ChangeMcot(g_mcot[i],i);
    g_minCw[i] = newMinCw;
    g_mcot[i] = newMcot;
    std::cout << "  new MinCw : " << g_minCw[i] << std::endl;
    std::cout << "  new Mcot  : " << g_mcot[i] << std::endl;
  }
  
  Simulator::Schedule (g_timeStep, &ChangeMacTypeAlt);
}

void
ChangeTrafficType(ApplicationContainer serverNodes, ApplicationContainer clientNodes)
{
  if (!g_connect)
  {
    g_connect = true;
    CreateEnv();
  }
  
  ApplicationContainer splitAppServerNodes[g_trafficType.size()];
  ApplicationContainer splitAppClientNodes[g_trafficType.size()];
  
  for (uint16_t i = 0; i < g_trafficType.size(); i++)
  {
    for (uint16_t j = 0; j < 2; j++)
    {
        splitAppServerNodes[i].Add(serverNodes.Get(i*2+j));
        splitAppClientNodes[i].Add(clientNodes.Get(i*2+j));
    }
  }
  
  for (uint16_t i = 0; i < g_trafficType.size(); i++)
  {
    uint32_t newTrafficType = g_env->ChangeTrafficType(g_trafficType[i],i);
    g_trafficType[i] = newTrafficType;
    std::cout << "ChangeTrafficType() for agent " << i << " with new traffic type : " << g_trafficType[i] << std::endl;
    if (g_trafficType[i] == 0)
    {
        uint32_t newUdpLambda = g_env->ChangeUdpLambda(g_udpLambda[i],i);
        uint32_t newPacketSize = g_env->ChangePacketSize(g_packetSize[i],i);
        g_udpLambda[i] = newUdpLambda;
        g_packetSize[i] = newPacketSize;
        std::cout << "  new udpLambda : " << g_udpLambda[i] << std::endl;
        std::cout << "  new packetSize  : " << g_packetSize[i] << std::endl;       
        
        for (uint16_t j = 0; j < 2; j++)
        {
            clientNodes.Get (i*2+j) -> SetUdpInterval (Seconds(1.0/g_udpLambda[i]));
            clientNodes.Get (i*2+j) -> SetUdpPacketSize (g_packetSize[i]);
        }
    }
    if (g_trafficType[i] == 1)
    {
        uint32_t newPacketSize = g_env->ChangePacketSize(g_packetSize[i],i);
        g_packetSize[i] = newPacketSize;
        std::cout << "  new packetSize : " << g_packetSize[i] << std::endl;
       
        for (uint16_t j = 0; j < 2; j++)
        {
            serverNodes.Get (i*2+j) -> SetBurstFragmentSize (g_packetSize[i]);
        }
    }
  }
  Simulator::Schedule (g_timeStep, &ChangeTrafficType, serverNodes, clientNodes);
}

void
UpdateCamType (NodeContainer gNbNodes)
{
    uint32_t ipv4ifIndex = 1;
    for (uint32_t i = 0; i < g_macType.size(); i++)
    {
        if (g_macType[i] == 0)
        {
            nr[i*3+1]->ChangeGnbTxPower(-200);
            gNbNodes.Get (i*3+1)->GetObject<Ipv4> ()->SetDown (ipv4ifIndex);
            nr[i*3+2]->ChangeGnbTxPower(-200);
            gNbNodes.Get (i*3+2)->GetObject<Ipv4> ()->SetDown (ipv4ifIndex);
            nr[i*3]->ChangeGnbTxPower(23);
            gNbNodes.Get (i*3)->GetObject<Ipv4> ()->SetUp (ipv4ifIndex);
            
            //nr[i*3]->GetNrHelper() ->SetGnbChannelAccessManagerTypeId(TypeId::LookupByName("ns3::NrCat2LbtAccessManager"));
        }
        else if (g_macType[i] == 1)
        {
            nr[i*3]->ChangeGnbTxPower(-200);
            gNbNodes.Get (i*3)->GetObject<Ipv4> ()->SetDown (ipv4ifIndex);
            nr[i*3+2]->ChangeGnbTxPower(-200);
            gNbNodes.Get (i*3+2)->GetObject<Ipv4> ()->SetDown (ipv4ifIndex);
            nr[i*3+1]->ChangeGnbTxPower(23);
            gNbNodes.Get (i*3+1)->GetObject<Ipv4> ()->SetUp (ipv4ifIndex);
            
            nr[i*3+1]->GetNrHelper() ->SetGnbChannelAccessManagerTypeId(TypeId::LookupByName("ns3::NrCat4LbtAccessManager"));
            nr[i*3+1]->GetNrHelper() ->SetGnbChannelAccessManagerAttribute ("Mcot", TimeValue(MilliSeconds (g_mcot[i])));
            nr[i*3+1]->GetNrHelper() ->SetGnbChannelAccessManagerAttribute ("Cat4MinCw", UintegerValue (g_minCw[i]));
        }
        else if (g_macType[i] == 2)
        {
            nr[i*3]->ChangeGnbTxPower(-200);
            gNbNodes.Get (i*3)->GetObject<Ipv4> ()->SetDown (ipv4ifIndex);
            nr[i*3+1]->ChangeGnbTxPower(-200);
            gNbNodes.Get (i*3+1)->GetObject<Ipv4> ()->SetDown (ipv4ifIndex);
            nr[i*3+2]->ChangeGnbTxPower(23);
            gNbNodes.Get (i*3+2)->GetObject<Ipv4> ()->SetUp (ipv4ifIndex);
            
            //nr[i*3+2]->GetNrHelper() ->SetGnbChannelAccessManagerTypeId(TypeId::LookupByName("ns3::NrAlwaysOnAccessManager"));
        }
    }

    Simulator::Schedule (g_timeStep, &UpdateCamType, gNbNodes);
}

void
UpdateTrafficType (NodeContainer gNbNodes)
{
    std::cout << "Debug UpdateTrafficType()" << std::endl;
    uint32_t ipv4ifIndex = 1;
    for (uint32_t i = 0; i < g_trafficType.size(); i++)
    {
        if (g_trafficType[i] == 0)
        {
            nr[i*2+1]->ChangeGnbTxPower(-200);
            gNbNodes.Get (i*2+1)->GetObject<Ipv4> ()->SetDown (ipv4ifIndex);
            nr[i*2]->ChangeGnbTxPower(23);
            gNbNodes.Get (i*2)->GetObject<Ipv4> ()->SetUp (ipv4ifIndex);
            
            nr[i*2]->GetNrHelper() ->SetGnbChannelAccessManagerTypeId(TypeId::LookupByName("ns3::NrCat4LbtAccessManager"));
            nr[i*2]->GetNrHelper() ->SetGnbChannelAccessManagerAttribute ("Mcot", TimeValue(MilliSeconds (g_mcot[i])));
            nr[i*2]->GetNrHelper() ->SetGnbChannelAccessManagerAttribute ("Cat4MinCw", UintegerValue (g_minCw[i]));
        }
        else if (g_trafficType[i] == 1)
        {
            nr[i*2]->ChangeGnbTxPower(-200);
            gNbNodes.Get (i*2)->GetObject<Ipv4> ()->SetDown (ipv4ifIndex);
            nr[i*2+1]->ChangeGnbTxPower(23);
            gNbNodes.Get (i*2+1)->GetObject<Ipv4> ()->SetUp (ipv4ifIndex);
            
            nr[i*2+1]->GetNrHelper() ->SetGnbChannelAccessManagerTypeId(TypeId::LookupByName("ns3::NrCat4LbtAccessManager"));
            nr[i*2+1]->GetNrHelper() ->SetGnbChannelAccessManagerAttribute ("Mcot", TimeValue(MilliSeconds (g_mcot[i])));
            nr[i*2+1]->GetNrHelper() ->SetGnbChannelAccessManagerAttribute ("Cat4MinCw", UintegerValue (g_minCw[i]));
        }
    }
    Simulator::Schedule (g_timeStep, &UpdateTrafficType, gNbNodes);
}

void
PrintProgressAlt ()
{
    std::cout << "Debug PrintProgressAlt()" << std::endl;
    std::cout << "Evaluation per timestep............" << std::endl;
    for (uint32_t i = 0; i < g_trafficType.size(); i++)
    {
        std::cout << "Evaluation for agent " << i << " with macType Cat4Lbt"  << std::endl;
        std::cout << "  with MinCw : " << g_minCw[i] << std::endl;
        std::cout << "  with Mcot  : " << g_mcot[i] << std::endl;
        
    }
    for (uint32_t i = 0; i < g_trafficType.size(); i++)
    {
        std::cout << "Evaluation for agent " << i << " with trafficType " << g_trafficType[i] << std::endl;
        if (g_trafficType[i] == 0)
        {
          std::cout << " with udpLambda : " << g_udpLambda[i] << std::endl;
          std::cout << " with packetSize : " << g_packetSize[i] << std::endl;
        }
        if (g_trafficType[i] == 1)
        {
          std::cout << " with fragmentSize : " << g_packetSize[i] << std::endl;
        }
    }
    
    /*
    for (uint32_t i = 0; i < g_throughput.size(); i++)
    {
        std::cout << "IP Address " << g_throughput[i].first << std::endl;
        std::cout << "Throughput "  << g_throughput[i].second << std::endl;
    }
    */
    
    std::cout << "Reward for average throughput (Mbps) : " << g_averageThroughput << std::endl;
    //std::cout << "Reward for average delay (microseconds) : " << g_averageDelay << std::endl;
    //std::cout << "Reward for average jitter (microseconds) : " << g_averageJitter << std::endl;
    
    Simulator::Schedule (g_timeStep, &PrintProgressAlt);
}

void ScheduleNextStateRead ()
{
    g_env->ScheduleNextStateRead();
}

int
main (int argc, char *argv[])
{

    sinrForDataPack.clear();
    ipAddrForSinr.clear();
    rxNodeForSinr.clear();

    //Scenario parameters
    uint32_t numWifiPairs = 0;
    uint32_t numNruPairs = 4;
    bool onlyLOS = true;
    uint32_t scenarioType = 1; //1 - freespace, 2 - factory  
    uint32_t scenarioId = 4; 
    uint32_t simRound = 1;
    uint32_t seed = 1;

    //Traffic parameters
    double appStartTime = 0; //seconds
    uint64_t UDP_SATURATION_RATE = 160000000;
    //UintegerValue packetSizeValue = 1474;
    BooleanValue spreadUdpLoad = true;
    DataRateValue dataRateValue = DataRate (UDP_SATURATION_RATE);
    //UintegerValue uintegerValue = UintegerValue(packetSizeValue);
    uint64_t bitRate = dataRateValue.Get().GetBitRate ();
    uint32_t packetSize = 1474; // bytes
    double interval = static_cast<double> (packetSize * 8) / bitRate;
    Time udpInterval = Seconds (interval);
    uint16_t operatorPortWifi = 1234;
    uint16_t operatorPortNru = 1235;

    double frameRate = 30;
    std::string targetDataRate = "40Mbps";
    std::string vrAppName = "VirusPopper";
    uint32_t fragmentSize = 1200;

    std::string trafficType = "UDP_SAT";

    //Distance parameters
    double d1 = 1.0; // meters
    double d2 = 1.0; // meters
    double d3 = 1.0; // meters

    //Simulation parameters
    double simTime = 50.0; // seconds
    double ueX = 1.0; // meters
    std::string simTag = "default";
    std::string outputDir = "./";
    bool enableNr = true;
    bool enableWifi = false;
    bool doubleTechnology = false;
    double apDensity = 18;
    bool positioning = true;

    std::string pathlossDir = "/home/inets/Desktop/A/ns-allinone-3.38/ns-3.38/freespacePL/"; //main directory where pathloss Matrix is stored

    //Physical parameters
    bool cellScan = true;
    double beamSearchAngleStep = 30.0; //degrees
    uint16_t numerologyBwp = 0;
    double frequency = 5.945e9; //Hz option : 5.945e9 5.975e9
    double bandwidth = 20e6; //Hz option : 20e6 80e6
    
    //NR and Wifi parameters
    std::string rlcModel = "RlcUmAlways";
    std::string errorModel = "ns3::NrEesmIrT1";
    std::string ueCamType = "ns3::NrAlwaysOnAccessManager";
    std::string nodeRate = "150Mbps"; //"500kbps";

    double totalTxPower = 23; // dBm
    double ueTxPower = 23; // dBm
    double cat2EDThreshold = -72.0; // dBm
    double cat3and4EDThreshold = -72.0; // dBm
    int frameAggregation = 0; //0 no agg. 1 MPDU max,2 MSDU max
    int Duplex = 0;
    uint8_t nruMcs = -1;


    double ccaThresholdAp = -62.0; //dBm Wifi
    double ccaThresholdSta = -62.0; //dBm Wifi
    uint8_t wifiMcs = -1;
    std::string wifiStandard = "11ax";

    std::string gnbCamType = "ns3::NrCat4LbtAccessManager";
    double lbtEDThreshold = -79.0;
    double lbtSlotTime = 9.0; //Microseconds
    double lbtDeferTime = 16.0; //MicroSeconds
    double lbtMcot = 5.0; //MilliSeconds
    std::string lbtBackoffType = "Binary";
    double cat2ED = -69.0;
    double cat2DeferTime = 10; //MicroSeconds
    uint32_t cat3CW = 15;
    uint32_t cat4RetryLimit = 0;
    uint32_t cat4MinCw = 15;
    uint32_t cat4MaxCw = 1023;
    uint32_t cat4CwUpdateRule = NrCat4LbtAccessManager::ANY_NACK; //alternatives : NrCat4LbtAccessManager::ALL_NACKS, NrCat4LbtAccessManager::NACKS_10_PERCENT, NrCat4LbtAccessManager::NACKS_80_PERCENT 
    uint32_t cat4LinearBackoff = 3;
    double onoffChangeTime = 10.0;

    //Vector parameters
    uint32_t maxNodes = 6;

    double totalTxPowerVector[maxNodes]; // dBm
    double ueTxPowerVector[maxNodes]; // dBm
    double cat2EDThresholdVector[maxNodes]; // dBm
    double cat3and4EDThresholdVector[maxNodes]; // dBm
    int frameAggregationVector[maxNodes]; //0 no agg. 1 MPDU max,2 MSDU max (not used)

    double ccaThresholdApVector[maxNodes]; //dBm Wifi (not used)
    double ccaThresholdStaVector[maxNodes]; //dBm Wifi (not used)

    std::string gNbCamVector[maxNodes];
    double lbtEDThresholdVector[maxNodes];
    double lbtSlotTimeVector[maxNodes];
    double lbtDeferTimeVector[maxNodes];
    double lbtMcotVector[maxNodes];
    std::string lbtBackoffTypeVector[maxNodes];
    double cat2EDVector[maxNodes];
    double cat2DeferTimeVector[maxNodes];
    uint32_t cat3CWVector[maxNodes];
    uint32_t cat4RetryLimitVector[maxNodes];
    uint32_t cat4MinCwVector[maxNodes];
    uint32_t cat4MaxCwVector[maxNodes]; 
    uint32_t cat4CwUpdateRuleVector[maxNodes]; //alternatives : NrCat4LbtAccessManager::ALL_NACKS, NrCat4LbtAccessManager::NACKS_10_PERCENT, NrCat4LbtAccessManager::NACKS_80_PERCENT 
    uint32_t cat4LinearBackoffVector[maxNodes];
    double onoffChangeTimeVector[maxNodes];
    uint8_t nruMcsVector[maxNodes];

    std::string trafficTypeVector [maxNodes];
    double appStartTimeVector[maxNodes];
    uint64_t UDP_SATURATION_RATE_VECTOR[maxNodes];
    uint32_t packetSizeVector[maxNodes];
    std::string nodeRateVector[maxNodes];
    double frameRateVector[maxNodes];
    std::string targetDataRateVector[maxNodes];
    uint32_t fragmentSizeVector[maxNodes];

    //Setting up defaults
    for (uint32_t i = 0; i < maxNodes; i++)
    { 
        totalTxPowerVector[i] = totalTxPower; // dBm
        ueTxPowerVector[i] = ueTxPower; // dBm
        cat2EDThresholdVector[i] = cat2EDThreshold; // dBm
        cat3and4EDThresholdVector[i] = cat3and4EDThreshold; // dBm
        frameAggregationVector[i] = frameAggregation; //0 no agg. 1 MPDU max,2 MSDU max (not used)

        ccaThresholdApVector[i] = ccaThresholdAp; //dBm Wifi (not used)
        ccaThresholdStaVector[i] = ccaThresholdSta; //dBm Wifi (not used)

        gNbCamVector[i] = gnbCamType;
        lbtEDThresholdVector[i] = lbtEDThreshold;
        lbtSlotTimeVector[i] = lbtSlotTime;
        lbtDeferTimeVector[i] = lbtDeferTime;
        lbtMcotVector[i] = lbtMcot;
        lbtBackoffTypeVector[i] = lbtBackoffType;
        cat2EDVector[i] = cat2ED;
        cat2DeferTimeVector[i] = cat2DeferTime;
        cat3CWVector[i] = cat3CW;
        cat4RetryLimitVector[i] = cat4RetryLimit;
        cat4MinCwVector[i] = cat4MinCw;
        cat4MaxCwVector[i] = cat4MaxCw; 
        cat4CwUpdateRuleVector[i] = cat4CwUpdateRule; //alternatives : NrCat4LbtAccessManager::ALL_NACKS, NrCat4LbtAccessManager::NACKS_10_PERCENT, NrCat4LbtAccessManager::NACKS_80_PERCENT 
        onoffChangeTimeVector[i] = onoffChangeTime;
        cat4LinearBackoffVector[i] = cat4LinearBackoff;
        nruMcsVector[i] = nruMcs;

        trafficTypeVector[i] = trafficType;
        UDP_SATURATION_RATE_VECTOR[i] = UDP_SATURATION_RATE;
        packetSizeVector[i] = packetSize;
        nodeRateVector[i] = nodeRate;
        frameRateVector[i] = frameRate;
        targetDataRateVector[i] = targetDataRate;
        fragmentSizeVector[i] = fragmentSize;
    }  

    std::stringstream outputDirAutomatic;

    NS_LOG_DEBUG ("Create base stations and mobile terminals");
    NodeContainer allWirelessNodes;
    NodeContainer wifiStaNodes;
    NodeContainer wifiApNodes;
  
    CommandLine cmd;
    cmd.AddValue ("enableNr", "Enable NR operator", enableNr);
    cmd.AddValue ("enableWifi", "Enable Wigig operator", enableWifi);
    cmd.AddValue ("numWifiPairs", "number of Wifi AP-STA pairs, Wifi node comes first", numWifiPairs);
    cmd.AddValue ("numNruPairs", "number of nru AP-STA pairs", numNruPairs);
    cmd.AddValue ("simTime", "Simulation time (seconds)", simTime);
    cmd.AddValue ("appStartTime", "Time the application starts (seconds)", appStartTime);
    cmd.AddValue("frameAggregation","0 no agg. 1 MPDU max, 2 MSDU max",frameAggregation);
    /*
    cmd.AddValue("frameAggregation2","0 no agg. 1 MPDU max, 2 MSDU max",frameAggregationVector[1]);
    cmd.AddValue("frameAggregation3","0 no agg. 1 MPDU max, 2 MSDU max",frameAggregationVector[2]);
    cmd.AddValue("frameAggregation4","0 no agg. 1 MPDU max, 2 MSDU max",frameAggregationVector[3]);
    cmd.AddValue("frameAggregation5","0 no agg. 1 MPDU max, 2 MSDU max",frameAggregationVector[4]);
    cmd.AddValue("frameAggregation6","0 no agg. 1 MPDU max, 2 MSDU max",frameAggregationVector[5]);
    */
    cmd.AddValue("Duplex","0 is FDD 1 is TDD",Duplex);
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
                  "Default total TX power (dBm) that will be proportionally assigned to"
                  " bandwidth parts depending on each BWP bandwidth ",
                  totalTxPower);
    cmd.AddValue ("totalTxPower1",
                  "Total TX power (dBm) that will be proportionally assigned to"
                  " bandwidth parts depending on each BWP bandwidth ",
                  totalTxPowerVector[0]);
    cmd.AddValue ("totalTxPower2",
                  "Total TX power (dBm) that will be proportionally assigned to"
                  " bandwidth parts depending on each BWP bandwidth ",
                  totalTxPowerVector[1]);
    cmd.AddValue ("totalTxPower3",
                  "Total TX power (dBm) that will be proportionally assigned to"
                  " bandwidth parts depending on each BWP bandwidth ",
                  totalTxPowerVector[2]);
    cmd.AddValue ("totalTxPower4",
                  "Total TX power (dBm) that will be proportionally assigned to"
                  " bandwidth parts depending on each BWP bandwidth ",
                  totalTxPowerVector[3]);
    cmd.AddValue ("totalTxPower5",
                  "Total TX power (dBm) that will be proportionally assigned to"
                  " bandwidth parts depending on each BWP bandwidth ",
                  totalTxPowerVector[4]);
    cmd.AddValue ("totalTxPower6",
                  "Total TX power (dBm) that will be proportionally assigned to"
                  " bandwidth parts depending on each BWP bandwidth ",
                  totalTxPowerVector[5]);
    cmd.AddValue ("ueTxPower1",
                  "TX power (dBm) for UEs",
                   ueTxPowerVector[0]);
    cmd.AddValue ("ueTxPower2",
                  "TX power (dBm) for UEs",
                   ueTxPowerVector[1]);
    cmd.AddValue ("ueTxPower3",
                  "TX power (dBm) for UEs",
                   ueTxPowerVector[2]);
    cmd.AddValue ("ueTxPower4",
                  "TX power (dBm) for UEs",
                   ueTxPowerVector[3]);
    cmd.AddValue ("ueTxPower5",
                  "TX power (dBm) for UEs",
                   ueTxPowerVector[4]);
    cmd.AddValue ("ueTxPower6",
                  "TX power (dBm) for UEs",
                   ueTxPowerVector[5]);              
    cmd.AddValue ("errorModelType",
                  "Error model type: ns3::NrEesmCcT1, ns3::NrEesmCcT2, ns3::NrEesmIrT1, ns3::NrEesmIrT2",
                  errorModel); 
    cmd.AddValue ("rlcModel", "The NR RLC Model: RlcTmAlways or RlcUmAlways", rlcModel);  
    cmd.AddValue ("scenarioType",
                  "Scenario (1 = freespace PL, 2 = factory scenario)",
                  scenarioType);
    cmd.AddValue ("scenarioId",
                  "Scenario (1 = 1 fixedAp - 1 randomUE / onlzLOS, 2 = 1 fixedAP - 1 nearest randomUE, 3 = 1 randomAP - 1 nearest randomUE)",
                  scenarioId);
    cmd.AddValue ("seed", "Simulation seed", seed);           
    cmd.AddValue ("simRound", "Simulation Run ID", simRound);
    cmd.AddValue ("nodeRate", "The default rate of every node in the network", nodeRate);
    cmd.AddValue ("gnbCamType1", "The gNB CAM of gNb", gNbCamVector[0]);
    cmd.AddValue ("gnbCamType2", "The gNB CAM of gNb", gNbCamVector[1]);
    cmd.AddValue ("gnbCamType3", "The gNB CAM of gNb", gNbCamVector[2]);
    cmd.AddValue ("gnbCamType4", "The gNB CAM of gNb", gNbCamVector[3]);
    cmd.AddValue ("gnbCamType5", "The gNB CAM of gNb", gNbCamVector[4]);
    cmd.AddValue ("gnbCamType6", "The gNB CAM of gNb", gNbCamVector[5]);
    cmd.AddValue ("ccaThresholdAp", 
                  "Carrer sensing threshold for wifi APs (dBm).",
                   ccaThresholdAp);
    /*
    cmd.AddValue ("ccaThresholdApVector2", 
                  "Carrer sensing threshold for wifi APs (dBm).",
                   ccaThresholdApVector[1]);
    cmd.AddValue ("ccaThresholdApVector", 
                  "Carrer sensing thres3hold for wifi APs (dBm).",
                   ccaThresholdApVector[2]);
    cmd.AddValue ("ccaThresholdApVector4", 
                  "Carrer sensing threshold for wifi APs (dBm).",
                   ccaThresholdApVector[3]);
    cmd.AddValue ("ccaThresholdApVector5", 
                  "Carrer sensing threshold for wifi APs (dBm).",
                   ccaThresholdApVector[4]);
    cmd.AddValue ("ccaThresholdApVector6", 
                  "Carrer sensing threshold for wifi APs (dBm).",
                   ccaThresholdApVector[5]);
    */
    cmd.AddValue ("ccaThresholdSta", 
                  "Carrer sensing threshold for wifi STAs (dBm).",
                   ccaThresholdSta);
    /*
    cmd.AddValue ("ccaThresholdStaVector2", 
                  "Carrer sensing threshold for wifi STAs (dBm).",
                   ccaThresholdStaVector[1]);
    cmd.AddValue ("ccaThresholdStaVector3", 
                  "Carrer sensing threshold for wifi STAs (dBm).",
                   ccaThresholdStaVector[2]);
    cmd.AddValue ("ccaThresholdStaVector4", 
                  "Carrer sensing threshld for wifi STAs (dBm).",
                   ccaThresholdStaVector[3]);
    cmd.AddValue ("ccaThresholdStaVector5", 
                  "Carrer sensing threshold for wifi STAs (dBm).",
                   ccaThresholdStaVector[4]);
    cmd.AddValue ("ccaThresholdStaVector6", 
                  "Carrer sensing threshold for wifi STAs (dBm).",
                   ccaThresholdStaVector[5]);
    */
    cmd.AddValue ("cat2EDThreshold1", 
                  "The ED threshold to be used by Lbt category 2 algorithm (dBm). Allowed range [-100.0, 0.0]., ",
                   cat2EDThresholdVector[0]);
    cmd.AddValue ("cat2EDThreshold2", 
                  "The ED threshold to be used by Lbt category 2 algorithm (dBm). Allowed range [-100.0, 0.0]., ",
                   cat2EDThresholdVector[1]);
    cmd.AddValue ("cat2EDThreshold3", 
                  "The ED threshold to be used by Lbt category 2 algorithm (dBm). Allowed range [-100.0, 0.0]., ",
                   cat2EDThresholdVector[2]);
    cmd.AddValue ("cat2EDThreshold4", 
                  "The ED threshold to be used by Lbt category 2 algorithm (dBm). Allowed range [-100.0, 0.0]., ",
                   cat2EDThresholdVector[3]);
    cmd.AddValue ("cat2EDThreshold5", 
                  "The ED threshold to be used by Lbt category 2 algorithm (dBm). Allowed range [-100.0, 0.0]., ",
                   cat2EDThresholdVector[4]);
    cmd.AddValue ("cat2EDThreshold6", 
                  "The ED threshold to be used by Lbt category 2 algorithm (dBm). Allowed range [-100.0, 0.0]., ",
                   cat2EDThresholdVector[5]);
    cmd.AddValue ("cat3and4EDTreshold1",
                  "The ED threshold to be used by Lbt category 3 and 4 algorithm (dBm). Allowed range [-100.0, 0.0].",
                   cat3and4EDThresholdVector[0]);
    cmd.AddValue ("cat3and4EDTreshold2",
                  "The ED threshold to be used by Lbt category 3 and 4 algorithm (dBm). Allowed range [-100.0, 0.0].",
                   cat3and4EDThresholdVector[1]);
    cmd.AddValue ("cat3and4EDTreshold3",
                  "The ED threshold to be used by Lbt category 3 and 4 algorithm (dBm). Allowed range [-100.0, 0.0].",
                   cat3and4EDThresholdVector[2]);
    cmd.AddValue ("cat3and4EDTreshold4",
                  "The ED threshold to be used by Lbt category 3 and 4 algorithm (dBm). Allowed range [-100.0, 0.0].",
                   cat3and4EDThresholdVector[3]);
    cmd.AddValue ("cat3and4EDTreshold5",
                  "The ED threshold to be used by Lbt category 3 and 4 algorithm (dBm). Allowed range [-100.0, 0.0].",
                   cat3and4EDThresholdVector[4]);
    cmd.AddValue ("cat3and4EDTreshold6",
                  "The ED threshold to be used by Lbt category 3 and 4 algorithm (dBm). Allowed range [-100.0, 0.0].",
                   cat3and4EDThresholdVector[5]);
    cmd.AddValue ("lbtEDThreshold1",
                  "CCA-ED threshold for channel sensing by Lbt algorithm (dBm). Allowed range [-100.0, 0.0].",
                   lbtEDThresholdVector[0]);
    cmd.AddValue ("lbtEDThreshold2",
                  "CCA-ED threshold for channel sensing by Lbt algorithm (dBm). Allowed range [-100.0, 0.0].",
                   lbtEDThresholdVector[1]);
    cmd.AddValue ("lbtEDThreshold3",
                  "CCA-ED threshold for channel sensing by Lbt algorithm (dBm). Allowed range [-100.0, 0.0].",
                   lbtEDThresholdVector[2]);
    cmd.AddValue ("lbtEDThreshold4",
                  "CCA-ED threshold for channel sensing by Lbt algorithm (dBm). Allowed range [-100.0, 0.0].",
                   lbtEDThresholdVector[3]);
    cmd.AddValue ("lbtEDThreshold5",
                  "CCA-ED threshold for channel sensing by Lbt algorithm (dBm). Allowed range [-100.0, 0.0].",
                   lbtEDThresholdVector[4]);
    cmd.AddValue ("lbtEDThreshold6",
                  "CCA-ED threshold for channel sensing by Lbt algorithm (dBm). Allowed range [-100.0, 0.0].",
                   lbtEDThresholdVector[5]);
    cmd.AddValue ("lbtSlotTime1",
                  "The duration of a Slot by  Lbt algorithm (microseconds).",
                   lbtSlotTimeVector[0]);
    cmd.AddValue ("lbtSlotTime2",
                  "The duration of a Slot by  Lbt algorithm (microseconds).",
                   lbtSlotTimeVector[1]);
    cmd.AddValue ("lbtSlotTime3",
                  "The duration of a Slot by  Lbt algorithm (microseconds).",
                   lbtSlotTimeVector[2]);
    cmd.AddValue ("lbtSlotTime4",
                  "The duration of a Slot by  Lbt algorithm (microseconds).",
                   lbtSlotTimeVector[3]);
    cmd.AddValue ("lbtSlotTime5",
                  "The duration of a Slot by  Lbt algorithm (microseconds).",
                   lbtSlotTimeVector[4]);
    cmd.AddValue ("lbtSlotTime6",
                  "The duration of a Slot by  Lbt algorithm (microseconds).",
                   lbtSlotTimeVector[5]);              
    cmd.AddValue ("lbtDeferTime1",
                  "TimeInterval to defer during CCA by Lbt (microseconds).",
                   lbtDeferTimeVector[0]);
    cmd.AddValue ("lbtDeferTime2",
                  "TimeInterval to defer during CCA by Lbt (microseconds).",
                   lbtDeferTimeVector[1]);
    cmd.AddValue ("lbtDeferTime3",
                  "TimeInterval to defer during CCA by Lbt (microseconds).",
                   lbtDeferTimeVector[2]);
    cmd.AddValue ("lbtDeferTime4",
                  "TimeInterval to defer during CCA by Lbt (microseconds).",
                   lbtDeferTimeVector[3]);
    cmd.AddValue ("lbtDeferTime5",
                  "TimeInterval to defer during CCA by Lbt (microseconds).",
                   lbtDeferTimeVector[4]);
    cmd.AddValue ("lbtDeferTime6",
                  "TimeInterval to defer during CCA by Lbt (microseconds).",
                   lbtDeferTimeVector[5]);
    cmd.AddValue ("lbtMcot1",
                  "Duration of channel access grant by Lbt algorithm (milliseconds). Allowed range [2.0, 20.0].",
                   lbtMcotVector[0]);
    cmd.AddValue ("lbtMcot2",
                  "Duration of channel access grant by Lbt algorithm (milliseconds). Allowed range [2.0, 20.0].",
                   lbtMcotVector[1]);
    cmd.AddValue ("lbtMcot3",
                  "Duration of channel access grant by Lbt algorithm (milliseconds). Allowed range [2.0, 20.0].",
                   lbtMcotVector[2]);
    cmd.AddValue ("lbtMcot4",
                  "Duration of channel access grant by Lbt algorithm (milliseconds). Allowed range [2.0, 20.0].",
                   lbtMcotVector[3]);
    cmd.AddValue ("lbtMcot5",
                  "Duration of channel access grant by Lbt algorithm (milliseconds). Allowed range [2.0, 20.0].",
                   lbtMcotVector[4]);
    cmd.AddValue ("lbtMcot6",
                  "Duration of channel access grant by Lbt algorithm (milliseconds). Allowed range [2.0, 20.0].",
                   lbtMcotVector[5]);
    cmd.AddValue ("cat2ED1",
                  "CCA-ED threshold for channel sensing by Lbt 2 algorithm (dBm). Allowed range [-100.0, 0.0].",
                   cat2EDVector[0]);
    cmd.AddValue ("cat2ED2",
                  "CCA-ED threshold for channel sensing by Lbt 2 algorithm (dBm). Allowed range [-100.0, 0.0].",
                   cat2EDVector[1]);
    cmd.AddValue ("cat2ED3",
                  "CCA-ED threshold for channel sensing by Lbt 2 algorithm (dBm). Allowed range [-100.0, 0.0].",
                   cat2EDVector[2]);
    cmd.AddValue ("cat2ED4",
                  "CCA-ED threshold for channel sensing by Lbt 2 algorithm (dBm). Allowed range [-100.0, 0.0].",
                   cat2EDVector[3]);
    cmd.AddValue ("cat2ED5",
                  "CCA-ED threshold for channel sensing by Lbt 2 algorithm (dBm). Allowed range [-100.0, 0.0].",
                   cat2EDVector[4]);
    cmd.AddValue ("cat2ED6",
                  "CCA-ED threshold for channel sensing by Lbt 2 algorithm (dBm). Allowed range [-100.0, 0.0].",
                   cat2EDVector[5]);
    cmd.AddValue ("cat2DeferTime1",
                  "TimeInterval to defer during CCA by Lbt 2 algorithm (microseconds).",
                   cat2DeferTimeVector[0]);
    cmd.AddValue ("cat2DeferTime2",
                  "TimeInterval to defer during CCA by Lbt 2 algorithm (microseconds).",
                   cat2DeferTimeVector[1]);
    cmd.AddValue ("cat2DeferTime3",
                  "TimeInterval to defer during CCA by Lbt 2 algorithm (microseconds).",
                   cat2DeferTimeVector[2]);
    cmd.AddValue ("cat2DeferTime4",
                  "TimeInterval to defer during CCA by Lbt 2 algorithm (microseconds).",
                   cat2DeferTimeVector[3]);
    cmd.AddValue ("cat2DeferTime5",
                  "TimeInterval to defer during CCA by Lbt 2 algorithm (microseconds).",
                   cat2DeferTimeVector[4]);
    cmd.AddValue ("cat2DeferTime6",
                  "TimeInterval to defer during CCA by Lbt 2 algorithm (microseconds).",
                   cat2DeferTimeVector[5]);
    cmd.AddValue ("cat3CW1",
                  "The default fixed value of the CW in the case that Cat 3 LBT is used.",
                   cat3CWVector[0]);
    cmd.AddValue ("cat3CW2",
                  "The default fixed value of the CW in the case that Cat 3 LBT is used.",
                   cat3CWVector[1]);
    cmd.AddValue ("cat3CW3",
                  "The default fixed value of the CW in the case that Cat 3 LBT is used.",
                   cat3CWVector[2]);
    cmd.AddValue ("cat3CW4",
                  "The default fixed value of the CW in the case that Cat 3 LBT is used.",
                   cat3CWVector[3]);
    cmd.AddValue ("cat3CW5",
                  "The default fixed value of the CW in the case that Cat 3 LBT is used.",
                   cat3CWVector[4]);
    cmd.AddValue ("cat3CW6",
                  "The default fixed value of the CW in the case that Cat 3 LBT is used.",
                   cat3CWVector[5]); 
    cmd.AddValue ("cat4RetryLimit1",
                  "How many times to try to retransmit with the current CW before reseting it to the MinCw value for Cat 4 Lbt.",
                   cat4RetryLimitVector[0]);
    cmd.AddValue ("cat4RetryLimit2",
                  "How many times to try to retransmit with the current CW before reseting it to the MinCw value for Cat 4 Lbt.",
                   cat4RetryLimitVector[1]);
    cmd.AddValue ("cat4RetryLimit3",
                  "How many times to try to retransmit with the current CW before reseting it to the MinCw value for Cat 4 Lbt.",
                   cat4RetryLimitVector[2]);
    cmd.AddValue ("cat4RetryLimit4",
                  "How many times to try to retransmit with the current CW before reseting it to the MinCw value for Cat 4 Lbt.",
                   cat4RetryLimitVector[3]);
    cmd.AddValue ("cat4RetryLimit5",
                  "How many times to try to retransmit with the current CW before reseting it to the MinCw value for Cat 4 Lbt.",
                   cat4RetryLimitVector[4]);
    cmd.AddValue ("cat4RetryLimit6",
                  "How many times to try to retransmit with the current CW before reseting it to the MinCw value for Cat 4 Lbt.",
                   cat4RetryLimitVector[5]);                                            
    cmd.AddValue ("cat4MinCw1",
                  "The minimum value of the contention window used by Lbt category 4 algorithm.",
                   cat4MinCwVector[0]);
    cmd.AddValue ("cat4MinCw2",
                  "The minimum value of the contention window used by Lbt category 4 algorithm.",
                   cat4MinCwVector[1]);
    cmd.AddValue ("cat4MinCw3",
                  "The minimum value of the contention window used by Lbt category 4 algorithm.",
                   cat4MinCwVector[2]);
    cmd.AddValue ("cat4MinCw4",
                  "The minimum value of the contention window used by Lbt category 4 algorithm.",
                   cat4MinCwVector[3]);
    cmd.AddValue ("cat4MinCw5",
                  "The minimum value of the contention window used by Lbt category 4 algorithm.",
                   cat4MinCwVector[4]);
    cmd.AddValue ("cat4MinCw6",
                  "The minimum value of the contention window used by Lbt category 4 algorithm.",
                   cat4MinCwVector[5]);
    cmd.AddValue ("cat4MaxCw1",
                  "The maximum value of the contention window used by Lbt category 4 algorithm.",
                   cat4MaxCwVector[0]);
    cmd.AddValue ("cat4MaxCw2",
                  "The maximum value of the contention window used by Lbt category 4 algorithm.",
                   cat4MaxCwVector[1]);
    cmd.AddValue ("cat4MaxCw3",
                  "The maximum value of the contention window used by Lbt category 4 algorithm.",
                   cat4MaxCwVector[2]);
    cmd.AddValue ("cat4MaxCw4",
                  "The maximum value of the contention window used by Lbt category 4 algorithm.",
                   cat4MaxCwVector[3]);
    cmd.AddValue ("cat4MaxCw5",
                  "The maximum value of the contention window used by Lbt category 4 algorithm.",
                   cat4MaxCwVector[4]);
    cmd.AddValue ("cat4MaxCw6",
                  "The maximum value of the contention window used by Lbt category 4 algorithm.",
                   cat4MaxCwVector[5]);
    cmd.AddValue ("cat4CwUpdateRule1",
                  "Rule according which CW will be updated used by Lbt category 4 algorithm.",
                   cat4CwUpdateRuleVector[0]);
    cmd.AddValue ("cat4CwUpdateRule2",
                  "Rule according which CW will be updated used by Lbt category 4 algorithm.",
                   cat4CwUpdateRuleVector[1]);
    cmd.AddValue ("cat4CwUpdateRule3",
                  "Rule according which CW will be updated used by Lbt category 4 algorithm.",
                   cat4CwUpdateRuleVector[2]);
    cmd.AddValue ("cat4CwUpdateRule4",
                  "Rule according which CW will be updated used by Lbt category 4 algorithm.",
                   cat4CwUpdateRuleVector[3]);
    cmd.AddValue ("cat4CwUpdateRule5",
                  "Rule according which CW will be updated used by Lbt category 4 algorithm.",
                   cat4CwUpdateRuleVector[4]);
    cmd.AddValue ("cat4CwUpdateRule6",
                  "Rule according which CW will be updated used by Lbt category 4 algorithm.",
                   cat4CwUpdateRuleVector[5]);      
    cmd.AddValue ("onoffChangeTime1",
                  "The default fixed value of the change time in OnOffAccessManager is used.",
                   onoffChangeTimeVector[0]);
    cmd.AddValue ("onoffChangeTime2",
                  "The default fixed value of the change time in OnOffAccessManager is used.",
                   onoffChangeTimeVector[1]);
    cmd.AddValue ("onoffChangeTime3",
                  "The default fixed value of the change time in OnOffAccessManager is used.",
                   onoffChangeTimeVector[2]);
    cmd.AddValue ("onoffChangeTime4",
                  "The default fixed value of the change time in OnOffAccessManager is used.",
                   onoffChangeTimeVector[3]);
    cmd.AddValue ("onoffChangeTime5",
                  "The default fixed value of the change time in OnOffAccessManager is used.",
                   onoffChangeTimeVector[4]);
    cmd.AddValue ("onoffChangeTime6",
                  "The default fixed value of the change time in OnOffAccessManager is used.",
                   onoffChangeTimeVector[5]);
    cmd.AddValue ("nruMcs1",
                  "The mcs used for ACM in the Nru.",
                   nruMcsVector[0]);
    cmd.AddValue ("nruMcs2",
                  "The mcs used for ACM in the Nru.",
                   nruMcsVector[1]);
    cmd.AddValue ("nruMcs3",
                  "The mcs used for ACM in the Nru.",
                   nruMcsVector[2]);
    cmd.AddValue ("nruMcs4",
                  "The mcs used for ACM in the Nru.",
                   nruMcsVector[3]);
    cmd.AddValue ("nruMcs5",
                  "The mcs used for ACM in the Nru.",
                   nruMcsVector[4]);
    cmd.AddValue ("nruMcs6",
                  "The mcs used for ACM in the Nru.",
                   nruMcsVector[5]);
    cmd.AddValue ("trafficType1",
                  "The traffic type used to transport data.",
                   trafficTypeVector[0]);
    cmd.AddValue ("trafficType2",
                  "The traffic type used to transport data.",
                   trafficTypeVector[1]);
    cmd.AddValue ("trafficType3",
                  "The traffic type used to transport data.",
                   trafficTypeVector[2]);
    cmd.AddValue ("trafficType4",
                  "The traffic type used to transport data.",
                   trafficTypeVector[3]);
    cmd.AddValue ("trafficType5",
                  "The traffic type used to transport data.",
                   trafficTypeVector[4]);
    cmd.AddValue ("trafficType6",
                  "The traffic type used to transport data.",
                   trafficTypeVector[5]);
    cmd.AddValue ("udpSaturationRate1",
                  "The UDP saturation rate used.",
                   UDP_SATURATION_RATE_VECTOR[0]);
    cmd.AddValue ("udpSaturationRate2",
                  "The UDP saturation rate used.",
                   UDP_SATURATION_RATE_VECTOR[1]);
    cmd.AddValue ("udpSaturationRate3",
                  "The UDP saturation rate used.",
                   UDP_SATURATION_RATE_VECTOR[2]);
    cmd.AddValue ("udpSaturationRate4",
                  "The UDP saturation rate used.",
                   UDP_SATURATION_RATE_VECTOR[3]);
    cmd.AddValue ("udpSaturationRate5",
                  "The UDP saturation rate used.",
                   UDP_SATURATION_RATE_VECTOR[4]);
    cmd.AddValue ("udpSaturationRate6",
                  "The UDP saturation rate used.",
                   UDP_SATURATION_RATE_VECTOR[5]);
    cmd.AddValue ("packetSize1",
                  "Packet size value to transport data.",
                   packetSizeVector[0]);
    cmd.AddValue ("packetSize2",
                  "Packet size value to transport data.",
                   packetSizeVector[1]);
    cmd.AddValue ("packetSize3",
                  "Packet size value to transport data.",
                   packetSizeVector[2]);
    cmd.AddValue ("packetSize4",
                  "Packet size value to transport data.",
                   packetSizeVector[3]);
    cmd.AddValue ("packetSize5",
                  "Packet size value to transport data.",
                   packetSizeVector[4]);
    cmd.AddValue ("packetSize6",
                  "Packet size value to transport data.",
                   packetSizeVector[5]);
    cmd.AddValue ("nodeRate1",
                  "Node rate used to transport data.",
                   nodeRateVector[0]);
    cmd.AddValue ("nodeRate2",
                  "Node rate used to transport data.",
                   nodeRateVector[1]);
    cmd.AddValue ("nodeRate3",
                  "Node rate used to transport data.",
                   nodeRateVector[2]);
    cmd.AddValue ("nodeRate4",
                  "Node rate used to transport data.",
                   nodeRateVector[3]);
    cmd.AddValue ("nodeRate5",
                  "Node rate used to transport data.",
                   nodeRateVector[4]);
    cmd.AddValue ("nodeRate6",
                  "Node rate used to transport data.",
                   nodeRateVector[5]);
    cmd.AddValue ("frameRate1",
                  "Frame rate for bursty traffic to transport data.",
                   frameRateVector[0]);
    cmd.AddValue ("frameRate2",
                  "Frame rate for bursty traffic to transport data.",
                   frameRateVector[1]);
    cmd.AddValue ("frameRate3",
                  "Frame rate for bursty traffic to transport data.",
                   frameRateVector[2]);
    cmd.AddValue ("frameRate4",
                  "Frame rate for bursty traffic to transport data.",
                   frameRateVector[3]);
    cmd.AddValue ("frameRate5",
                  "Frame rate for bursty traffic to transport data.",
                   frameRateVector[4]);
    cmd.AddValue ("frameRate6",
                  "Frame rate for bursty traffic to transport data.",
                   frameRateVector[5]);               
    cmd.AddValue ("targetDataRate1",
                  "The target data rate for bursty traffic to transport data.",
                   targetDataRateVector[0]);
    cmd.AddValue ("targetDataRate2",
                  "The target data rate for bursty traffic to transport data.",
                   targetDataRateVector[1]);
    cmd.AddValue ("targetDataRate3",
                  "The target data rate for bursty traffic to transport data.",
                   targetDataRateVector[2]);
    cmd.AddValue ("targetDataRate4",
                  "The target data rate for bursty traffic to transport data.",
                   targetDataRateVector[3]);
    cmd.AddValue ("targetDataRate5",
                  "The target data rate for bursty traffic to transport data.",
                   targetDataRateVector[4]);
    cmd.AddValue ("targetDataRate6",
                  "The target data rate for bursty traffic to transport data.",
                   targetDataRateVector[5]);                       
    cmd.AddValue ("fragmentSize1",
                  "The fragment size for bursty traffic used to transport data.",
                   fragmentSizeVector[0]);
    cmd.AddValue ("fragmentSize2",
                  "The fragment size for bursty traffic used to transport data.",
                   fragmentSizeVector[1]);               
    cmd.AddValue ("fragmentSize3",
                  "The fragment size for bursty traffic used to transport data.",
                   fragmentSizeVector[2]);
    cmd.AddValue ("fragmentSize4",
                  "The fragment size for bursty traffic used to transport data.",
                   fragmentSizeVector[3]);
    cmd.AddValue ("fragmentSize5",
                  "The fragment size for bursty traffic used to transport data.",
                   fragmentSizeVector[4]);
    cmd.AddValue ("fragmentSize6",
                  "The fragment size for bursty traffic used to transport data.",
                   fragmentSizeVector[5]);            

    cmd.Parse (argc, argv);

    LogComponentEnable ("nr_ai_mac", LOG_LEVEL_ALL);

    uint32_t numApStaPairs = numWifiPairs + numNruPairs;

    DataRateValue dataRateValueVector[maxNodes];
    uint64_t bitRateVector[maxNodes];
    double intervalVector[maxNodes];
    Time udpIntervalVector[maxNodes];
    
    for (uint32_t i = 0; i < maxNodes; i++)
    {
        dataRateValueVector[i] = DataRate (UDP_SATURATION_RATE_VECTOR[i]);
        bitRateVector[i] = dataRateValueVector[i].Get().GetBitRate ();
        intervalVector[i] = static_cast<double> (packetSizeVector[i] * 8) / bitRateVector[i];
        udpIntervalVector[i] = Seconds (intervalVector[i]);
    }

    NodeContainer ueNodes[numNruPairs];
    NodeContainer gNbNodes[numNruPairs];

    std::stringstream SimTag;
    SimTag << "_" << numWifiPairs << "_" << numNruPairs << "_" << frameAggregation << "_" << simRound << "_" << gnbCamType;
    RngSeedManager::SetSeed (seed);
    RngSeedManager::SetRun (300 + simRound);
    ConfigureDefaultValues (cellScan, beamSearchAngleStep, errorModel, cat2EDThreshold, cat3and4EDThreshold, rlcModel,SimTag.str());

    //Setting wifi standard
    enum WifiStandard standard = WIFI_STANDARD_80211ax_6GHZ;
    if (wifiStandard == "11ac")
    {
        standard = WIFI_STANDARD_80211ac;
    }
    else if (wifiStandard != "11ax")
    {
        NS_ABORT_MSG ("Unsupported Wi-Fi standard");
    }

    NS_LOG_DEBUG ("Setting duplex mode");
    std::string duplexMode;
    if(Duplex == 0)
    {
        duplexMode = "FDD";
    }
    else if (Duplex == 1)
    {
        duplexMode = "TDD";
    }

    NS_LOG_DEBUG ("Create output file");
    outputDirAutomatic << "coex-ai/" << duplexMode << "/"<< gnbCamType << "_" << frameAggregation << "/" << simRound << "/";
    outputDir = "./" + outputDirAutomatic.str();
    //NodeDistributionScenario *scenario;
  
    NS_LOG_DEBUG ("Create base stations and mobile terminals");
    NodeContainer staNodes;
    staNodes.Create (numApStaPairs);
    NodeContainer apNodes;
    apNodes.Create (numApStaPairs);
    allWirelessNodes = NodeContainer (staNodes,apNodes);

    NS_LOG_DEBUG ("Create mobility");
    //Set AP and UE position array
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
    double staPositionVector [18][3];
    double apPositionVector [18][3];
    uint32_t indexAp[18];

    if (positioning == true)
    {
        std::stringstream filenamePosAp;
        std::stringstream filenameIndexAp;
        std::stringstream filenamePosSta;

        if (scenarioType == 1)
        {
            if (scenarioId == 1)
            {
                filenamePosAp << pathlossDir << "Scenario1/gNbLayoutMatrix.txt";
                filenameIndexAp << pathlossDir << "Scenario1/apIndex_" << simRound << ".txt";
                filenamePosSta << pathlossDir << "Scenario1/ueCoordinates_" << simRound << ".txt";
            }
            if (scenarioId == 2)
            {
                filenamePosAp << pathlossDir << "Scenario2/gNbLayoutMatrix.txt";
                filenameIndexAp << pathlossDir << "Scenario2/apIndex_" << simRound << ".txt";
                filenamePosSta << pathlossDir << "Scenario2/ueCoordinates_" << simRound << ".txt";
            }
            if (scenarioId == 3)
            {
                filenamePosAp << pathlossDir << "Scenario3/gNbLayoutMatrix_" << simRound << ".txt";
                filenameIndexAp << pathlossDir << "Scenario3/apIndex_" << simRound << ".txt";
                filenamePosSta << pathlossDir << "Scenario3/ueCoordinates_" << simRound << ".txt";
            }
            if (scenarioId == 4)
            {
                filenamePosAp << pathlossDir << "Scenario4/gNbLayoutMatrix.txt";
                filenameIndexAp << pathlossDir << "Scenario4/apIndex_" << simRound << ".txt";
                filenamePosSta << pathlossDir << "Scenario4/ueCoordinates_" << simRound << ".txt";
            }
        }
        if (scenarioType == 2)
        {
            if (scenarioId == 1)
            {
                filenamePosAp << pathlossDir << "onlyLOS/gNbLayoutMatrix.txt";
                filenameIndexAp << pathlossDir << "onlyLOS/apIndex_" << simRound << ".txt";
                filenamePosSta << pathlossDir << "onlyLOS/ueCoordinates_" << simRound << ".txt";
            }
            if (scenarioId == 2)
            {
                filenamePosAp << pathlossDir << "notOnlyLOS/gNbLayoutMatrix.txt";
                filenameIndexAp << pathlossDir << "notOnlyLOS/apIndex_" << simRound << ".txt";
                filenamePosSta << pathlossDir << "notOnlyLOS/ueCoordinates_" << simRound << ".txt";
            }
        }
        
        //Read pathloss from file for APs and UE
        /*
        if (!inFile.is_open ()) { NS_LOG_ERROR ("Can't open file!!!" );} 
        
        for (uint32_t i=0; i<10; i++) 
        {
        NS_ABORT_MSG_UNLESS (!inFile.eof(), "End of file!!!");
        inFile >> index_A[i];
        }
        
        for (uint32_t i=0; i<30; i++) 
        {
        NS_ABORT_MSG_UNLESS (!inFile.eof(), "End of file!!!");
        inFile >> index_B[i];
        }           
        inFile.close(); 
        
        uint32_t index_all[noAPs_A+noAPs_B];
        for (uint16_t i=0; i<noAPs_A; i++)
        {
        index_all[i] = index_A[i];
        } 
        
        for (uint16_t i=0; i<noAPs_B; i++)
        {
        index_all[noAPs_A+i] = index_B[i];
        }
        */

        std::ifstream inFile;
        
        inFile.open(filenameIndexAp.str(), std::ios_base::in);
        if (!inFile.is_open ()) 
        { 
            NS_LOG_ERROR ("Can't open file!!!" ); 
        }
        std::cout << "AP Index : ";
        for (uint32_t i = 0; i < 18; i++)
        { 
            NS_ABORT_MSG_UNLESS (!inFile.eof(), "End of file!!!");
            inFile >> indexAp[i];
            std::cout << indexAp[i] << " ";
        }  
        std::cout << std::endl;
        inFile.close();

        //Read positions from file for APs and UEs
        inFile.open(filenamePosAp.str(), std::ios_base::in);
        if (!inFile.is_open ()) 
        { 
            NS_LOG_ERROR ("Can't open file!!!" ); 
        }

        for (uint32_t i = 0; i < 18; i++)
        { 
            for (uint32_t j = 0; j < 3; j++)
                {
                    NS_ABORT_MSG_UNLESS (!inFile.eof(), "End of file!!!");
                    inFile >> apPositionVector[i][j];
                }
        }  
        inFile.close();  

        inFile.open(filenamePosSta.str(), std::ios_base::in);
        if (!inFile.is_open ()) 
        { 
            NS_LOG_ERROR ("Can't open file!!!" ); 
        }
        for (uint32_t i = 0; i < 18; i++)
        { 
            for (uint32_t j = 0; j < 3; j++)
                {
                    NS_ABORT_MSG_UNLESS (!inFile.eof(), "End of file!!!");
                    inFile >> staPositionVector[i][j];
                }
        }  
        inFile.close(); 

        for (uint32_t i = 0; i < numApStaPairs; i++)
        {
            positionAlloc->Add (Vector (staPositionVector[indexAp[i]-1][0], staPositionVector[indexAp[i]-1][1], staPositionVector[indexAp[i]-1][2]));
        };
        
        for (uint32_t i = 0; i < numApStaPairs; i++)
        {
            positionAlloc->Add (Vector (apPositionVector[indexAp[i]-1][0], apPositionVector[indexAp[i]-1][1], apPositionVector[indexAp[i]-1][2]));
        };
        
        for (uint32_t i = 0; i < numApStaPairs; i++)
        {
            std::cout << "AP " << indexAp[i] << " x : "<< apPositionVector[indexAp[i]-1][0]<<", y : "<< apPositionVector[indexAp[i]-1][1]<<", z : "<< apPositionVector[indexAp[i]-1][2] << std::endl;
            std::cout << "STA " << indexAp[i] << " x : "<< staPositionVector[indexAp[i]-1][0]<<", y : "<< staPositionVector[indexAp[i]-1][1]<<", z : "<< staPositionVector[indexAp[i]-1][2] << std::endl;
        };

    }
        
    else
    {
        Ptr<UniformRandomVariable> co = CreateObject<UniformRandomVariable> ();
        for(uint32_t i = 0; i < (numApStaPairs) *2;i++)
        {
            double xa = co->GetValue (0,1);
            double ya = co->GetValue (0, 7);
            double za = co->GetValue (0, 3);
            positionAlloc->Add (Vector (xa, ya, za));
            //positionAlloc->Add (Vector (0, 0, 0));
            // positionAlloc->Add (Vector (1, 30, 1));
            // positionAlloc->Add (Vector (0, 0, 1));
            // positionAlloc->Add (Vector (1, 0, 1));
        }
    }
    
    MobilityHelper mobility;
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator (positionAlloc);
    
    NS_LOG_DEBUG ("Set path loss configuration");
    Ptr<MatrixPropagationLossModel> propagationPathlossMatrix = CreateObject<MatrixPropagationLossModel> ();
    propagationPathlossMatrix->SetDefaultLoss (200.0);
    mobility.Install (allWirelessNodes);
    readPathLoss(pathlossDir, numApStaPairs, &staNodes, &apNodes, propagationPathlossMatrix, simRound, scenarioType, scenarioId); 
    //readPathLoss (&staNodes, &apNodes, simRound, apDensity, propagationPathlossMatrix,d3,numWifiPairs,numNruPairs); 

    NS_LOG_DEBUG ("Checking path loss matrix");
    std::cout << "Matrix pathloss between APs - APs" << std::endl;
    for (uint16_t i = 0; i < apNodes.GetN (); i++)
    {
        for (uint16_t j = 0; j < apNodes.GetN (); j++)
        {

            std::cout << propagationPathlossMatrix->GetLoss(apNodes.Get (i)->GetObject<MobilityModel> (), apNodes.Get (j)->GetObject<MobilityModel> ()) << ", ";
        }
        std::cout << std::endl;
    }

    std::cout << "Matrix pathloss between APs - STAs" << std::endl;
    for (uint16_t i = 0; i < apNodes.GetN (); i++)
    {
        for (uint16_t j = 0; j < staNodes.GetN (); j++)
        {

            std::cout << propagationPathlossMatrix->GetLoss(apNodes.Get (i)->GetObject<MobilityModel> (), staNodes.Get (j)->GetObject<MobilityModel> ()) << ", ";
        }
        std::cout << std::endl;
    }

    std::cout << "Matrix pathloss between STAs - STAs" << std::endl;
    for (uint16_t i = 0; i < staNodes.GetN (); i++)
    {
        for (uint16_t j = 0; j < staNodes.GetN (); j++)
        {

            std::cout << propagationPathlossMatrix->GetLoss(staNodes.Get (i)->GetObject<MobilityModel> (), staNodes.Get (j)->GetObject<MobilityModel> ()) << ", ";
        }
        std::cout << std::endl;
    }

    NS_LOG_DEBUG ("Setting up new nodes");
    std::cout << "Number of allWirelessNodes nodes : " << allWirelessNodes.GetN () << std::endl;

    if(enableWifi)
    {    
        for (uint32_t i = 0; i < numWifiPairs; i++)
        {
            wifiApNodes.Add (allWirelessNodes.Get (i+numApStaPairs));
            wifiStaNodes.Add (allWirelessNodes.Get(i));
        }
    }

    std::cout << "Number of WifiSta nodes : " << wifiStaNodes.GetN () << std::endl;
    std::cout << "Number of WifiAp nodes : "<< wifiApNodes.GetN () << std::endl;

    double ueNodesNum = 0;
    double gnbNodesNum = 0;
    
    if (enableNr)
    {
        for (uint32_t i = numWifiPairs; i < numApStaPairs; i++)
        {
            ueNodes[i-numWifiPairs].Add (allWirelessNodes.Get(i));
            gNbNodes[i-numWifiPairs].Add (allWirelessNodes.Get (i+numApStaPairs));

            ueNodesNum += ueNodes[i-numWifiPairs].GetN ();
            gnbNodesNum += gNbNodes[i-numWifiPairs].GetN ();
        } 
    }                   

    std::cout << "Number of UE nodes : " << ueNodesNum << std::endl;
    std::cout << "Number of gNb nodes : "<< gnbNodesNum << std::endl;

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
    
    //Set gNbCamType
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

    ss << "coex-ai-" << scenarioId << "-" << technology;
    ss << gnbCam << "-" << nodeRate << "-" << rlcModel << ".db";

    //SqliteOutputManager manager (outputDir, numWifiPairs, outputDir + ss.str (), ss.str (), seed, numNruPairs, simRound);
    //outputManager = &manager;

    //Declare nr-u and wifi helper
    //L2Setup* nr[numNruPairs];
    std::vector<std::unique_ptr<WifiSetup>> wifi;
    
    NS_LOG_DEBUG("Create channel configuration");
    //Create and configure spectrumChannel
    Ptr<SpectrumChannel> spectrumChannel;
    Ptr<MultiModelSpectrumChannel> channel = CreateObject<MultiModelSpectrumChannel>();
    Ptr<ThreeGppPropagationLossModel> propagation = CreateObject<ThreeGppRmaPropagationLossModel>();
    Ptr<ThreeGppSpectrumPropagationLossModel> spectrumPropagation = CreateObject<ThreeGppSpectrumPropagationLossModel> ();

    //Setting up propagation
    propagation->SetAttributeFailSafe("Frequency", DoubleValue(frequency));
    spectrumPropagation->SetChannelModelAttribute("Frequency", DoubleValue(frequency));

    BandwidthPartInfo::Scenario channelScenario = BandwidthPartInfo::RMa_LoS;
    Ptr<ChannelConditionModel> channelConditionModel = CreateObject<ThreeGppRmaChannelConditionModel> ();
    spectrumPropagation->SetChannelModelAttribute ("Scenario", StringValue ("RMa"));
    spectrumPropagation->SetChannelModelAttribute ("ChannelConditionModel", PointerValue (channelConditionModel));
    propagation->SetChannelConditionModel (channelConditionModel);

    channel->AddPropagationLossModel (propagation);
    channel->AddSpectrumPropagationLossModel (spectrumPropagation);

    NS_LOG_DEBUG("Create the internet and remote host");
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create (1);
    Ptr<Node> remoteHost = remoteHostContainer.Get (0);
    InternetStackHelper internet;
    internet.Install (remoteHostContainer);

    NS_LOG_DEBUG("Creating IP adresses for UEs and linking to AP");
    Ipv4InterfaceContainer ueIpIface;
    Ipv4InterfaceContainer ueIpIfaceOperatorWifi, ueIpIfaceOperatorNru[numNruPairs];
    Ipv4InterfaceContainer remoteWifiIface ,remoteNruIface;
    std::unordered_map<uint32_t, NodeContainer> networkWifiMap;
    std::unordered_map<uint32_t, Ptr<Node>> apMap;

    if(enableNr && numNruPairs!=0)
    {
        std::unique_ptr<Ipv4AddressHelper> address[numNruPairs]; 
        for (uint32_t i = 0; i < numNruPairs; i++)
        {
            address[i] = std::unique_ptr<Ipv4AddressHelper> (new Ipv4AddressHelper ());
        }   
        
        std::unordered_map<uint32_t, uint32_t> connections[numNruPairs];
        for (uint32_t ind = 0; ind < numNruPairs; ind++)
        {
            for (auto it = ueNodes[ind].Begin(); it != ueNodes[ind].End(); ++it)
            {
                Ptr<Node> closestAp = gNbNodes[ind].Get(0);
                std::cout << (*it)->GetId() << " connected with " << closestAp->GetId() << std::endl;
                connections[ind].insert (std::make_pair (ind, ind));
            }
        }   

        NS_LOG_DEBUG("Create NR simulation helpers"); 
        Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper> ();
        Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper> ();
        Ptr<NrHelper> nrHelper = CreateObject<NrHelper> ();

        Ipv4StaticRoutingHelper ipv4RoutingHelper;
        InternetStackHelper internet;
        
        for (uint32_t i = 0; i < numNruPairs; i++)
        {
            nr[i] = new NrSingleBwpSetup (gNbNodes[i], ueNodes[i], nrHelper, epcHelper, idealBeamformingHelper, channel, 
                                  propagation, spectrumPropagation, frequency, bandwidth, numerologyBwp, 
                                  totalTxPowerVector[i], ueTxPowerVector[i], connections[i], gNbCamVector[i], ueCamType, 
                                  "ns3::NrMacSchedulerTdmaPF", channelScenario, Duplex,
                                  lbtBackoffTypeVector[i],
                                  lbtEDThresholdVector[i], lbtSlotTimeVector[i], lbtDeferTimeVector[i], lbtMcotVector[i],
                                  cat2EDVector[i], cat2DeferTimeVector[i],
                                  cat3CWVector[i],
                                  cat4RetryLimitVector[i], cat4MinCwVector[i], cat4MaxCwVector[i], cat4CwUpdateRuleVector[i], cat4LinearBackoffVector[i],
                                  onoffChangeTimeVector[i], nruMcsVector[i]);
            internet.Install (nr[i]->GetUeNodes());
            ueIpIface = nr[i]-> GetEpcHelper ()->AssignUeIpv4Address (nr[i]->GetUeDev());
            auto ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (nr[i]->GetUeNodes().Get (0)->GetObject<Ipv4> ());
            ueStaticRouting->SetDefaultRoute (nr[i]-> GetEpcHelper ()->GetUeDefaultGatewayAddress (), 1);    
            nr[i]-> GetNrHelper ()->AttachToEnb (nr[i]->GetUeDev ().Get (0), nr[i]->GetGnbDev ().Get (0));    
            nr[i]->AssignIpv4ToStations (address[i]); 
            ueIpIfaceOperatorNru[i].Add (ueIpIface);
        }

        Ptr<Node> pgw = epcHelper->GetPgwNode ();
        // connect a remoteHost to pgw. Setup routing too
        PointToPointHelper p2ph;
        p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("1Gb/s")));
        p2ph.SetDeviceAttribute ("Mtu", UintegerValue (2500));
        p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.000)));

        std::cout << "Connecting remote hosts to EPC with a 1 Gb/s link, MTU 2500 B, delay 0 s" << std::endl;
        
        NetDeviceContainer internetDevices;
        for (auto it = remoteHostContainer.Begin (); it != remoteHostContainer.End (); ++it)
        {
        internetDevices.Add (p2ph.Install (pgw, *it));
        }

        std::string base = "1.0.0.0";
        Ipv4AddressHelper ipv4h;
        ipv4h.SetBase (base.c_str (), "255.255.0.0");
        Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
        remoteNruIface = internetIpIfaces;

        for (uint32_t i = 0; i < numNruPairs; i++)
        {
            //nr[i]->SetSinrCallback (MakeCallback (&OutputManager::SinrStore, &manager));
            //nr[i]->SetMacTxDataFailedCb (MakeCallback (&OutputManager::MacDataTxFailed, &manager));
            //nr[i]->SetChannelOccupancyCallback (MakeCallback (&NrOccupancy));
            ueIpIfaceOperatorNru[i].Add (ueIpIface);
        }      
    }

    if(enableWifi && numNruPairs == 0)
    {
        std::cout << "TESTING BUG" << std::endl;
        std::cout << "TESTING SPECTRUM CHANNEL POINTER" << std::endl;
        spectrumChannel = CreateObject<MultiModelSpectrumChannel> ();
        std::cout << spectrumChannel << std::endl;
    }

    if(enableWifi && numNruPairs == 0)
    {
        spectrumChannel->AddPropagationLossModel(propagationPathlossMatrix);
    }
    
    if (enableWifi)
    {
        NS_LOG_DEBUG("Creating Wifi helpers"); 
        int32_t indWifi = 0;
        for (auto it = wifiStaNodes.Begin(); it != wifiStaNodes.End(); ++it)
            {
                Ptr<Node> closestAp = wifiApNodes.Get(indWifi);
                uint32_t closestApId = closestAp->GetId();
                std::cout << (*it)->GetId() << " connected with " << closestAp->GetId() << std::endl;

                auto v = networkWifiMap.find (closestApId);
                if (v == networkWifiMap.end ())
                {
                    v = networkWifiMap.emplace (std::make_pair (closestApId, NodeContainer ())).first;
                }
                v->second.Add (*it);

                if (apMap.find (closestApId) == apMap.end())
                {
                    apMap.insert (std::make_pair (closestApId, closestAp));
                }
                indWifi++;
            }

        uint32_t startingClass = 1;
        for (const auto & v : networkWifiMap) 
        {
            std::stringstream base, ssid, remoteAddress;
            base << "10.0." << startingClass << ".0";
            remoteAddress << "2." <<startingClass << ".0.0";
            ssid << "coex" << startingClass;
            startingClass++;
            std::unique_ptr<Ipv4AddressHelper> address = std::unique_ptr<Ipv4AddressHelper> (new Ipv4AddressHelper ());
            address->SetBase (Ipv4Address (base.str().c_str ()), "255.255.255.0");
            auto it = apMap.find (v.first);
            wifi.push_back (std::unique_ptr<WifiSetup> (new WifiSetup (NodeContainer (it->second), v.second,
                                    channel, propagation, spectrumPropagation,
                                    frequency, bandwidth, totalTxPower, totalTxPower, ccaThresholdAp, ccaThresholdSta,
                                    standard, "primero",frameAggregation)));
            ueIpIfaceOperatorWifi.Add(wifi.back()->AssignIpv4ToUe (address));
            wifi.back()->AssignIpv4ToStations (address);
            remoteWifiIface = wifi.back()->ConnectToRemotes (remoteHostContainer, remoteAddress.str().c_str());
            //wifi.back()->SetSinrCallback (MakeCallback (&OutputManager::SinrStore, &manager));
            //wifi.back()->SetMacTxDataFailedCb (MakeCallback (&OutputManager::MacDataTxFailed, &manager));
            //wifi.back()->SetChannelOccupancyCallback (MakeCallback (&WifiOccupancy));
            //wifi.back()->SetChannelRTACallback (MakeCallback (&OutputManager::ChannelRequestTime, &manager));
        }
    }

    if (enableNr && numNruPairs > 0)
    {
        NS_LOG_DEBUG("Create NR net device");
        std::cout << "TESTING BUG" << std::endl;
        std::cout << "TESTING SPECTRUM CHANNEL POINTER" << std::endl;
        Ptr<NrGnbNetDevice> nrGnbNetDevice = DynamicCast<NrGnbNetDevice> (nr[0]->GetGnbDev ().Get (0));
        spectrumChannel = nrGnbNetDevice->GetPhy (0)->GetSpectrumPhy ()->GetSpectrumChannel ();
        std::cout << spectrumChannel << std::endl;
        spectrumChannel->AddPropagationLossModel (propagationPathlossMatrix);
    }

    if (enableWifi && numNruPairs == 0)
    {
        NS_LOG_DEBUG("Create Wifi net device");
        Ptr<WifiNetDevice> wifiNetDevice = DynamicCast<WifiNetDevice>(wifi.back()->GetGnbDev().Get(0));
        spectrumChannel = DynamicCast<SpectrumChannel>(wifiNetDevice->GetPhy()->GetChannel());
        std::cout << spectrumChannel << std::endl;
        spectrumChannel->AddPropagationLossModel (propagationPathlossMatrix);
    }

    for (uint32_t i = 0; i < numApStaPairs; i++)
    {
        if (i < numWifiPairs) 
        {
            g_nodeIndex_popA.push_back(i);
        }
        else 
        {
            g_nodeIndex_popB.push_back(i);
        }
    }

    //Setting up routing
    uint32_t ifaceId = 1;
    if (enableNr && numNruPairs != 0)
    {
        Ipv4StaticRoutingHelper ipv4RoutingHelper;
        for (auto it = remoteHostContainer.Begin (); it != remoteHostContainer.End (); ++it)
        {
            Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting ((*it)->GetObject<Ipv4> ());
            remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), ifaceId);
        }
        ifaceId++;
    }

    if (enableWifi)
    {
        Ipv4StaticRoutingHelper ipv4RoutingHelper;
        for (auto it = remoteHostContainer.Begin (); it != remoteHostContainer.End (); ++it)
        {
            for (uint32_t startingClass = 1; startingClass <= networkWifiMap.size (); startingClass++)
            {
                std::stringstream base;
                base << "10.0." << startingClass<< ".0";
                Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting ((*it)->GetObject<Ipv4> ());
                remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address (base.str().c_str()), Ipv4Mask ("255.255.255.0"), ifaceId);
                ifaceId++;
            }
        }
    }

    NodeContainer wifiStaNodesAlt[numWifiPairs];
    NodeContainer wifiApNodesAlt[numWifiPairs];
    Ipv4InterfaceContainer ueIpIfaceOperatorWifiAlt[numWifiPairs];
    for (uint32_t i = 0; i < numWifiPairs; i++)
    {
        wifiStaNodesAlt[i].Add (wifiStaNodes.Get(i));
        wifiApNodesAlt[i].Add (wifiApNodes.Get (i));
        ueIpIfaceOperatorWifiAlt[i].Add(ueIpIfaceOperatorWifi.Get (i));
    }   

    ApplicationContainer clientAppsWifi, clientAppsNru, serverAppsNru, serverAppsWifi;
    ApplicationContainer pingApps;
    uint32_t udpLambda = 2000;
    double ftpLambda = 5;
    uint32_t ftpFileSize = 512000;
    uint16_t dlPortLowLat = 1236;

    Time clientAppStartTime = MilliSeconds(500);
    Time serverAppStartTime = MilliSeconds(500);
    Time simulationTime = MilliSeconds(1500);

    std::string transportProtocol = "ns3::UdpSocketFactory";

    uint32_t maxFileSize = 5e6;

    //Testing new traffic
    /*
    //Upcoming traffics

    //XR Traffic
    uint16_t dlPortArStart = 1121;
    uint16_t dlPortArStop = 1124;
    uint16_t dlPortVrStart = 1131;
    uint16_t dlPortCgStart = 1141;

    EpsBearer vrBearer(EpsBearer::NGBR_LOW_LAT_EMBB);

    Ptr<EpcTft> vrTft = Create<EpcTft>();
    EpcTft::PacketFilter dlpfVr;
    dlpfVr.localPortStart = dlPortVrStart;
    dlpfVr.localPortEnd = dlPortVrStart;
    vrTft->Add(dlpfVr);

    for (uint32_t i = 0; i < wifiStaNodes.GetN(); ++i)
    {
        ConfigureXrApp(wifiStaNodes,
                       i,
                       ueIpIfaceOperatorWifi,
                       VR_DL1,
                       dlPortVrStart,
                       transportProtocol,
                       remoteHostContainer,
                       ueVrNetDev,
                       nrHelper,
                       vrBearer,
                       vrTft,
                       1,
                       arTfts,
                       serverApps,
                       clientApps,
                       pingApps);
    }

    //NGMN_HTTP
    if (enableWifi)
    {
        NodeContainer httpUeContainer;

        for (uint32_t i = 0; i < wifiStaNodes.GetN(); i++)
        {
            httpUeContainer.Add(wifiStaNodes.Get(i));
        }

        // 1. Create HTTP client applications
        ThreeGppHttpClientHelper clientHelper(remoteWifiIface.GetAddress(1));
        // Install HTTP clients on UEs
        ApplicationContainer clientApps = clientHelper.Install(httpUeContainer);

        // 2. Create HTTP server applications
        ThreeGppHttpServerHelper serverHelper(remoteWifiIface.GetAddress(1));
        // Install HTTP server on a remote host node
        ApplicationContainer serverApps = serverHelper.Install(remoteHostContainer.Get(0));
        Ptr<ThreeGppHttpServer> httpServer = serverApps.Get(0)->GetObject<ThreeGppHttpServer>();
        
        // 3. Setup HTTP variables for the server according to NGMN white paper
        PointerValue ptrVal;
        httpServer->GetAttribute("Variables", ptrVal);
        Ptr<ThreeGppHttpVariables> httpParameters = ptrVal.Get<ThreeGppHttpVariables>();
        httpParameters->SetMainObjectSizeMean(10710);        // according to NGMN white paper
        httpParameters->SetMainObjectSizeStdDev(25032);      // according to NGMN white paper
        httpParameters->SetEmbeddedObjectSizeMean(7758);     // according to NGMN white paper
        httpParameters->SetEmbeddedObjectSizeStdDev(126168); /// according to NGMN white paper
        httpParameters->SetNumOfEmbeddedObjectsMax(55);      // according to NGMN white paper
        httpParameters->SetNumOfEmbeddedObjectsScale(2);     // according to NGMN white paper
        httpParameters->SetNumOfEmbeddedObjectsShape(1.1);   // according to NGMN white paper
        httpParameters->SetReadingTimeMean(Seconds(30));     // according to NGMN white paper
        httpParameters->SetParsingTimeMean(Seconds(0.13));   // according to NGMN white paper

        for (uint32_t i = 0; i < ueIpIfaceOperatorWifi.GetN(); i++)
        {
            Ipv4Address ipAddress = ueIpIfaceOperatorWifi.GetAddress(i, 0);
            V4PingHelper ping(ipAddress);
            pingApps.Add(ping.Install(remoteHostContainer));    
        }
    }    
    
    if (enableNr && numNruPairs !=0)
    {
        for (uint32_t i = numWifiPairs; i < numApStaPairs; i++)
        {
            TrafficGeneratorHelper ftpHelper(transportProtocol, Address(), TrafficGeneratorNgmnFtpMulti::GetTypeId());
            ftpHelper.SetAttribute("PacketSize", UintegerValue(1448));
            ftpHelper.SetAttribute("MaxFileSize", UintegerValue(5e6));

            for (uint32_t index = 0; index < ueNodes[i-numWifiPairs].GetN(); index++)
            {
                Ipv4Address ipAddress = ueIpIfaceOperatorNru[index].GetAddress(0, 0);
                AddressValue ueAddress(InetSocketAddress(ipAddress, operatorPortNru));
                ftpHelper.SetAttribute("Remote", ueAddress);
                clientAppsNru.Add(ftpHelper.Install(remoteHostContainer));
                // Seed the ARP cache by pinging early in the simulation
                // This is a workaround until a static ARP capability is provided
                V4PingHelper ping(ipAddress);
                pingApps.Add(ping.Install(remoteHostContainer));
            }

            // configure FTP servers
            InetSocketAddress localAddress(Ipv4Address::GetAny(), operatorPortNru);
            PacketSinkHelper packetSinkHelper(transportProtocol, localAddress);

            for (uint32_t index = 0; index < ueNodes[i-numWifiPairs].GetN(); index++)
            {
                serverAppsNru.Add(packetSinkHelper.Install(ueNodes[i-numWifiPairs].Get(index)));
            }
        } 
    } 
    */

    //Proper syntax
    //Setting up udp server for operator A
    /*
    if (enableWifi)
    {   
        for (uint32_t i = 0; i < numWifiPairs; i++)
        {
            if (trafficTypeVector[i] == "UDP_SAT")
            {
                serverAppsWifi.Add (setUdpServer (wifiStaNodesAlt[i], operatorPortWifi));
                clientAppsWifi.Add(setUdpClient (wifiStaNodesAlt[i], remoteHostContainer, ueIpIfaceOperatorWifiAlt[i], operatorPortWifi, udpIntervalVector[i], appStartTime, packetSizeVector[i]));
            }
            if (trafficTypeVector[i] == "UDP_CBR")
            {
                serverAppsWifi.Add (setUdpPoissonServer (wifiStaNodesAlt[i], operatorPortWifi));
                clientAppsWifi.Add (setUdpPoissonClient (wifiStaNodesAlt[i], remoteHostContainer, remoteHost, ueIpIfaceOperatorWifiAlt[i], operatorPortWifi, udpLambda, packetSizeVector[i]));
            }
            if (trafficTypeVector[i] == "OnOff")
            {
                serverAppsWifi.Add (setOnOffServer (wifiStaNodesAlt[i], operatorPortWifi));
                clientAppsWifi.Add(setOnOffClient (wifiStaNodesAlt[i], remoteHostContainer, ueIpIfaceOperatorWifiAlt[i], operatorPortWifi,  appStartTime, packetSizeVector[i], nodeRateVector[i]));
            }
            if (trafficTypeVector[i] == "TCP")
            {
                serverAppsWifi.Add (setTcpServer (wifiStaNodesAlt[i], operatorPortWifi, remoteWifiIface));
                clientAppsWifi.Add(setTcpClient (wifiStaNodesAlt[i], remoteHostContainer, ueIpIfaceOperatorWifiAlt[i], operatorPortWifi,  appStartTime, packetSizeVector[i], nodeRateVector[i]));
            }
            if (trafficTypeVector[i] == "Burst")
            {
                serverAppsWifi.Add(setBurstServer (wifiStaNodesAlt[i], operatorPortWifi, ueIpIfaceOperatorWifiAlt[i], fragmentSizeVector[i], frameRateVector[i], targetDataRateVector[i], vrAppName, remoteHostContainer));
                clientAppsWifi.Add(setBurstClient (remoteHostContainer, remoteWifiIface, wifiStaNodesAlt[i], operatorPortWifi));
            }
            if (trafficTypeVector[i] == "FTP_3GPP_M1")
            {
                ApplicationContainer ftpServerAppsWifi[numWifiPairs];
                ApplicationContainer ftpClientAppsWifi[numWifiPairs];
                Ptr<ThreeGppFtpM1Helper> ftpHelperWifi[numWifiPairs];
                ftpHelperWifi[i] = CreateObject<ThreeGppFtpM1Helper>(&ftpServerAppsWifi[i],
                                                      &ftpClientAppsWifi[i],
                                                      &wifiStaNodesAlt[i],
                                                      &remoteHostContainer,
                                                      &ueIpIfaceOperatorWifiAlt[i]);
                ftpHelperWifi[i]->Configure(operatorPortWifi,
                                    serverAppStartTime,
                                    clientAppStartTime,
                                    simulationTime,
                                    ftpLambda,
                                    ftpFileSize);
                ftpHelperWifi[i]->Start();
                clientAppsWifi.Add(ftpClientAppsWifi[i]);
                serverAppsWifi.Add(ftpServerAppsWifi[i]);
            }
            if (trafficTypeVector[i] == "NGMN_FTP")
            {
                serverAppsWifi.Add (setNgmnFtpServer (wifiStaNodesAlt[i], operatorPortWifi, transportProtocol));
                clientAppsWifi.Add (setNgmnFtpClient (wifiStaNodesAlt[i], remoteHostContainer, ueIpIfaceOperatorWifiAlt[i], operatorPortWifi, maxFileSize, packetSizeVector[i], transportProtocol));
            }
            if (trafficTypeVector[i] == "NGMN_VIDEO")
            {
                serverAppsWifi.Add (setNgmnVideoServer (wifiStaNodesAlt[i], operatorPortWifi, transportProtocol));
                clientAppsWifi.Add (setNgmnVideoClient (wifiStaNodesAlt[i], remoteHostContainer, ueIpIfaceOperatorWifiAlt[i], operatorPortWifi, udpIntervalVector[i], packetSizeVector[i], transportProtocol));
            }
            if (trafficTypeVector[i] == "NGMN_GAMING")
            {
                serverAppsWifi.Add (setNgmnGamingServer (wifiStaNodesAlt[i], operatorPortWifi, transportProtocol));
                clientAppsWifi.Add (setNgmnGamingClient (wifiStaNodesAlt[i], remoteHostContainer, ueIpIfaceOperatorWifiAlt[i], operatorPortWifi, transportProtocol));
            }
            if (trafficTypeVector[i] == "NGMN_VOIP")
            {
                serverAppsWifi.Add (setNgmnVoipServer (wifiStaNodesAlt[i], operatorPortWifi, transportProtocol));
                clientAppsWifi.Add (setNgmnVoipClient (wifiStaNodesAlt[i], remoteHostContainer, ueIpIfaceOperatorWifiAlt[i], operatorPortWifi, transportProtocol));
            }
        }  
    }

    if (enableNr && numNruPairs !=0)
    {
        for (uint32_t i = numWifiPairs; i < numApStaPairs; i++)
        {
            if (trafficTypeVector[i] == "UDP_SAT")
            {
                serverAppsNru.Add (setUdpServer (ueNodes[i-numWifiPairs], operatorPortNru));
                clientAppsWifi.Add(setUdpClient (ueNodes[i-numWifiPairs], remoteHostContainer, ueIpIfaceOperatorNru[i-numWifiPairs], operatorPortNru, udpIntervalVector[i], appStartTime, packetSizeVector[i]));
            }
            if (trafficTypeVector[i] == "UDP_CBR")
            {
                serverAppsNru.Add (setUdpPoissonServer (ueNodes[i-numWifiPairs], operatorPortNru));
                clientAppsNru.Add (setUdpPoissonClient (ueNodes[i-numWifiPairs], remoteHostContainer, remoteHost, ueIpIfaceOperatorNru[i-numWifiPairs], operatorPortNru, udpLambda, packetSizeVector[i]));
            }
            if (trafficTypeVector[i] == "OnOff")
            {
                serverAppsNru.Add (setOnOffServer (ueNodes[i-numWifiPairs], operatorPortNru));
                clientAppsNru.Add(setOnOffClient (ueNodes[i-numWifiPairs], remoteHostContainer, ueIpIfaceOperatorNru[i-numWifiPairs], operatorPortNru,  appStartTime, packetSizeVector[i], nodeRateVector[i]));
            }
            if (trafficTypeVector[i] == "TCP")
            {
                serverAppsNru.Add (setTcpServer (ueNodes[i-numWifiPairs], operatorPortNru, remoteNruIface));
                clientAppsNru.Add(setTcpClient (ueNodes[i-numWifiPairs], remoteHostContainer, ueIpIfaceOperatorNru[i-numWifiPairs], operatorPortNru,  appStartTime, packetSizeVector[i], nodeRateVector[i]));
            }
            if (trafficTypeVector[i] == "Burst")
            {
                serverAppsNru.Add(setBurstServer (ueNodes[i-numWifiPairs], operatorPortNru, ueIpIfaceOperatorNru[i-numWifiPairs], fragmentSizeVector[i], frameRateVector[i], targetDataRateVector[i], vrAppName, remoteHostContainer));
                clientAppsNru.Add(setBurstClient (remoteHostContainer, remoteNruIface, ueNodes[i-numWifiPairs], operatorPortNru));
            }
            if (trafficTypeVector[i] == "FTP_3GPP_M1")
            {
                ApplicationContainer ftpServerAppsNru[numNruPairs];
                ApplicationContainer ftpClientAppsNru[numNruPairs];
                Ptr<ThreeGppFtpM1Helper> ftpHelperNru[numNruPairs];
                ftpHelperNru[i-numWifiPairs] = CreateObject<ThreeGppFtpM1Helper>(&ftpServerAppsNru[i-numWifiPairs],
                                                        &ftpClientAppsNru[i-numWifiPairs],
                                                        &ueNodes[i-numWifiPairs],
                                                        &remoteHostContainer,
                                                        &ueIpIfaceOperatorNru[i-numWifiPairs]);
                ftpHelperNru[i-numWifiPairs]->Configure(operatorPortNru,
                                clientAppStartTime,
                                clientAppStartTime,
                                simulationTime,
                                ftpLambda,
                                ftpFileSize);
                ftpHelperNru[i-numWifiPairs]->Start();
                clientAppsNru.Add(ftpClientAppsNru[i-numWifiPairs]);
                serverAppsNru.Add(ftpServerAppsNru[i-numWifiPairs]);
            }
            if (trafficTypeVector[i] == "NGMN_FTP")
            {
                serverAppsNru.Add (setNgmnFtpServer (ueNodes[i-numWifiPairs], operatorPortNru, transportProtocol));
                clientAppsNru.Add (setNgmnFtpClient (ueNodes[i-numWifiPairs], remoteHostContainer, ueIpIfaceOperatorNru[i-numWifiPairs], operatorPortNru, maxFileSize, packetSizeVector[i], transportProtocol));
            }
            if (trafficTypeVector[i] == "NGMN_VIDEO")
            {
                serverAppsNru.Add (setNgmnVideoServer (ueNodes[i-numWifiPairs], operatorPortNru, transportProtocol));
                clientAppsNru.Add (setNgmnVideoClient (ueNodes[i-numWifiPairs], remoteHostContainer, ueIpIfaceOperatorNru[i-numWifiPairs], operatorPortNru, udpIntervalVector[i], packetSizeVector[i], transportProtocol));
            }
            if (trafficTypeVector[i] == "NGMN_GAMING")
            {
                serverAppsNru.Add (setNgmnGamingServer (ueNodes[i-numWifiPairs], operatorPortNru, transportProtocol));
                clientAppsNru.Add (setNgmnGamingClient (ueNodes[i-numWifiPairs], remoteHostContainer, ueIpIfaceOperatorNru[i-numWifiPairs], operatorPortNru, transportProtocol));
            }
            if (trafficTypeVector[i] == "NGMN_VOIP")
            {
                serverAppsNru.Add (setNgmnVoipServer (ueNodes[i-numWifiPairs], operatorPortNru, transportProtocol));
                clientAppsNru.Add (setNgmnVoipClient (ueNodes[i-numWifiPairs], remoteHostContainer, ueIpIfaceOperatorNru[i-numWifiPairs], operatorPortNru, transportProtocol));
            }
        } 
    }
    */

    for (uint32_t i = 0; i < numNruPairs/2; i++)
    {
        serverAppsNru.Add (setUdpPoissonServer (ueNodes[i*2-numWifiPairs], operatorPortNru));
        clientAppsNru.Add (setUdpPoissonClientAi (ueNodes[i*2-numWifiPairs], remoteHostContainer, remoteHost, ueIpIfaceOperatorNru[i*2-numWifiPairs], operatorPortNru, udpLambda, packetSizeVector[i*2], appStartTime));
        serverAppsNru.Add(setBurstServerAi (ueNodes[i*2+1-numWifiPairs], operatorPortNru, ueIpIfaceOperatorNru[i*2+1-numWifiPairs], fragmentSizeVector[i*2+1], frameRateVector[i*2+1], targetDataRateVector[i*2+1], remoteHostContainer, appStartTime));
        clientAppsNru.Add(setBurstClient (remoteHostContainer, remoteNruIface, ueNodes[i*2+1-numWifiPairs], operatorPortNru));
    }
    
    pingApps.Start(Seconds(0.300));
    pingApps.Stop(Seconds(0.500));

    PopulateArpCache ();
    Packet::EnablePrinting ();
    
    /*
    std::string newCamType = "ns3::NrAlwaysOnAccessManager";
    Ptr<NrHelper> nrHelper = nr[0]->GetNrHelper();
    Ptr<Node> nodeChange = gNbNodes[0].Get (0);
    BandwidthPartInfoPtrVector newBwp = nr[0]->GetBwpVector ();
    //nr[0]->GetNrHelper()->ChangeGnbCam (gNbNodes[0].Get (0), nr[0]->GetBwpVector (), newCamType);
    */
    //Simulator::Schedule (Seconds (1), &Ipv4::SetDown, ipv4test1, ipv4ifIndex1);

    //Simulator::Schedule (Seconds (1), &NrHelper::ChangeGnbCam, nrHelper, nodeChange, newBwp, newCamType);

    std::cout << "UE" << std::endl;
    // PrintIpAddress (scenario->GetUes ());
    // PrintRoutingTable (scenario->GetUes ());
    // std::cout << "GNB" << std::endl;
    // PrintIpAddress (scenario->GetGnbs ());
    // PrintRoutingTable (scenario->GetGnbs ());
    // std::cout << "REMOTE" << std::endl;
    PrintIpAddress (remoteHostContainer);
    PrintRoutingTable (remoteHostContainer);

    Simulator::Schedule (MicroSeconds (100), &TimePasses);

    // Flow monitor
    //FlowMonitorHelper flowHelper;
    //auto monitor = flowHelper.InstallAll ();

    FlowMonitorHelper flowHelper;
    NodeContainer endpointNodes;
    endpointNodes.Add (remoteHost);
    for (uint32_t i = 0; i < numNruPairs; i++)
    {
        endpointNodes.Add (ueNodes[i]);
    } 
    endpointNodes.Add (wifiStaNodes);

    Ptr<FlowMonitor> monitor = flowHelper.Install (endpointNodes);
    monitor->Stop (Seconds(simTime));
    //Ptr<FlowMonitor> monitor2 = flowHelper.Install (endpointNodes2);

    //13.11.2023
    //monitor->SetAttribute("DelayBinWidth", DoubleValue(0.001));
    //monitor->SetAttribute("JitterBinWidth", DoubleValue(0.001));
    //monitor->SetAttribute("PacketSizeBinWidth", DoubleValue(20));

    std::ofstream myfile_e2e;
    std::stringstream eestre;
    eestre<<outputDirAutomatic.str()<<"e2e"<<d3<<"_"<<numWifiPairs<<"_"<<numNruPairs;
    myfile_e2e.open (eestre.str());
    myfile_e2e <<"technology"<<"\t,"<< "thMbps"<<"\t,"<<"txBytes"<<"\t,"<<"rxBytes"<<"\t,"<<"delay"<<"\t,"<<"jitter"<<"\t,"<<"addr"<< '\n';
    
    NodeContainer newGnbNodes;
    NodeContainer newUeNodes;
    Ipv4InterfaceContainer newIpIface;

    uint32_t ipv4ifIndex = 1;

    for (uint16_t i = 0; i < numNruPairs; i++)
    {
        newGnbNodes.Add(gNbNodes[i].Get (0)); 
        newUeNodes.Add(ueNodes[i].Get (0)); 
        newIpIface.Add(ueIpIfaceOperatorNru[i].Get (0)); 
    }
    
    for (uint16_t i = 0; i < numNruPairs; i++)
    {
        nr[i]->ChangeGnbTxPower(-200);
        newGnbNodes.Get (i)->GetObject<Ipv4> ()->SetDown (ipv4ifIndex);  
    }


    Simulator::Schedule (g_timeStep, &GiveThroughputAlt, &flowHelper, monitor, numNruPairs);
    Simulator::Schedule (g_timeStep, &PrintProgressAlt);
    Simulator::Schedule (g_timeStep, &ChangeMacTypeAlt);
    Simulator::Schedule (g_timeStep, &ChangeTrafficType, serverAppsNru, clientAppsNru);
    Simulator::Schedule (g_timeStep, &ScheduleNextStateRead);
    Simulator::Schedule (g_timeStep, &UpdateTrafficType, newGnbNodes);
    
    Simulator::Stop (Seconds (simTime));

    Simulator::Run ();

    NS_LOG_DEBUG("Printing output data");
    std::ofstream layoutFile, outputFile;
    std::string outputFileName, layoutFileName;
    std::string outputDirectory = "/home/inets/Desktop/A/workspace-nr/ns-3-dev/nru-results/factory-coexistance/";
    layoutFileName = outputDirectory + "apLayouts_simRound_" + std::to_string(simRound) + ".txt";
    outputFileName = outputDirectory + "nru_wifi_coexistance_" +  gnbCam + "_simRound_" + std::to_string(simRound) + "_numWifiPairs_" + std::to_string(numWifiPairs) + "_numNruPairs_" + std::to_string(numNruPairs) + ".txt";

    layoutFile.open(layoutFileName);

    layoutFile << "STA/AP \t Index \t x (m) \t y (m) \t z (m)\n";

    for (uint32_t i = 0; i < numApStaPairs; i++)
    {
        layoutFile << "AP\t" << indexAp[i] << "\t" << apPositionVector[indexAp[i]-1][0] << "\t" << apPositionVector[indexAp[i]-1][1] << "\t" << apPositionVector[indexAp[i]-1][2] << "\n";
        layoutFile << "STA\t" << indexAp[i] << "\t" << staPositionVector[indexAp[i]-1][0] << "\t" << staPositionVector[indexAp[i]-1][1] << "\t" << staPositionVector[indexAp[i]-1][2] << "\n";
    };

    layoutFile.close();

    outputFile.open(outputFileName);

    outputFile << "Flow \t Technology \t Throughput (Mbps) \t Delay (microseconds) \t Jitter (microseconds)\n";

    //13.11.2023
    monitor->CheckForLostPackets();

    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowHelper.GetClassifier ());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();

    double averageFlowThroughput = 0.0;
    double averageFlowDelay = 0.0;
    double delayValues[stats.size()];
    uint64_t cont = 0;

    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
      {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
	    double throughputMbps;

        bool isNr = ((t.sourceAddress.Get () >> 24) & 0xff) == 1;
        bool isWigig = ((t.sourceAddress.Get () >> 24) & 0xff) != 1;

        std::cout << "Flow " << i->first << " (" << t.sourceAddress << ":" << t.sourcePort << " -> " << t.destinationAddress << ":" << t.destinationPort << ")" << std::endl;
        outputFile << i->first << " (" << t.sourceAddress << ":" << t.sourcePort << " -> " << t.destinationAddress << ":" << t.destinationPort << ") \t";
        
        std::cout << "  Tx Packets: " << i->second.txPackets << "\n";
        std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
        std::cout << "  TxOffered:  "
                  << i->second.txBytes * 8.0 / (simTime - appStartTime) / 1000.0 /
                         1000.0
                  << " Mbps\n";
        std::cout << "  Rx Packets: " << i->second.rxPackets << "\n";
        std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";

        if (i->second.rxPackets > 0)
        {
            if (isNr || isWigig)
            {
                std::string technology = isNr ? "nr" : "wifi";
                std::cout << "  Technology : "<< technology << std::endl;
                outputFile << technology << " \t";
                if (isNr)
                {
                    throughputMbps = i->second.rxBytes * 8.0 / (simTime - appStartTime) / 1e6;
                    std::cout << "  Throughput : "<< throughputMbps << " Mbps" << std::endl;
                    outputFile << throughputMbps << "\t";
                }
                
                if (isWigig)
                {
                    throughputMbps = i->second.rxBytes * 8.0 / (simTime - appStartTime - 0.01) / 1e6;
                    std::cout << "  Throughput : "<< throughputMbps << " Mbps" << std::endl;
                    outputFile << throughputMbps << "\t";
                }
                
                double delay = 0.0;
                double jitter = 0.0;

                delay = i->second.delaySum.GetMicroSeconds () / i->second.rxPackets;
                std::cout << "  Mean delay : "<< delay << " microseconds" << std::endl;
                outputFile << delay << "\t";
                //jitter = i->second.jitterSum.GetMicroSeconds () / (i->second.rxPackets - 1);
                //std::cout << "  Mean jitter : "<< jitter << " microseconds" << std::endl;
                //outputFile << jitter << "\t";
                std::cout << "  Last packet delay: " << i->second.lastDelay.As(Time::MS) << "\n";

                std::stringstream addr;
                t.destinationAddress.Print (addr);

                //manager.StoreE2EStatsFor (technology, throughputMbps, i->second.txBytes, i->second.rxBytes, delay, jitter, addr.str ());
                myfile_e2e <<technology<<"\t,"<< throughputMbps <<"\t,"<<i->second.txBytes<<"\t,"<<i->second.rxBytes<<"\t,"<<delay<<"\t,"<<jitter<<"\t,"<<addr.str ()<< '\n';
                

                averageFlowThroughput += i->second.rxBytes * 8.0 / (simTime - appStartTime) / 1000 / 1000;
                averageFlowDelay += 1000 * i->second.delaySum.GetSeconds() / i->second.rxPackets;
                delayValues[cont] = 1000 * i->second.delaySum.GetSeconds() / i->second.rxPackets;
                cont++;
            }
            outputFile << "\n";
        }
      }

    if (enableNr && numNruPairs != 0)
    {
        //manager.StoreChannelOccupancyRateFor ("nr", m_nrOccupancy.GetSeconds () / (simTime - udpAppStartTime));
    }

    if (enableWifi)
    {
        //.StoreChannelOccupancyRateFor ("wifi", m_wifiOccupancy.GetSeconds () / (simTime - udpAppStartTime));
    }

    //manager.Close ();
    std::sort(delayValues, delayValues + stats.size());

    outputFile.close();

    /*
    std::ofstream resultSinr;
    std::stringstream snrstr;
    snrstr << outputDirAutomatic.str() <<"WiFiSinr_" <<d3<< numWifiPairs << "_" << numNruPairs;
    resultSinr.open(snrstr.str(), std::ios_base::out);
    if (!resultSinr.is_open ())
    {
        NS_LOG_ERROR ("Can't open file!!!" );
    } 
    for (uint32_t j=0; j<sinrForDataPack.size(); j++)
    {
        resultSinr<< sinrForDataPack[j] <<", "<< rxNodeForSinr[j] <<", "<< ipAddrForSinr[j] <<", "<<MCS_WIFI[j]<<", "<<numaggrpacket[j]<<"\n";
    }
    resultSinr.close();
    */

    double FiftyTileFlowDelay = delayValues[stats.size() / 2];

    std::cout << "Summary of flows " << std::endl;
    std::cout << "  Mean flow throughput : "<< averageFlowThroughput / stats.size () << " Mbps" <<std::endl;
    std::cout << "  Mean flow delay : " << averageFlowDelay / stats.size()  << " ms" <<std::endl;
    std::cout << "  Median flow delay : " << FiftyTileFlowDelay  << " ms" <<std::endl;

    Simulator::Destroy ();
    
    return 0;
}

