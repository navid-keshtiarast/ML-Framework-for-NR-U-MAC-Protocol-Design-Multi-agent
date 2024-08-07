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
#include "wifi-mac-env.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("wifi-ai-mac");

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

  // Config::SetDefault ("ns3::ApWifiMac::EnableBeaconJitter", BooleanValue (false));

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

  Config::SetDefault ("ns3::FlowMonitor::MaxPerHopDelay", TimeValue(Seconds(1000000000)));
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

void readPathLoss (std::string pathlossDir, uint32_t numApStaPairs, NodeContainer* staNodes, NodeContainer* apNodes, Ptr<MatrixPropagationLossModel> pathloss, uint32_t simRound, uint32_t scenarioType, uint32_t scenarioId)
{
  uint32_t apDensity;
  if(scenarioId == 1 || scenarioId == 2 || scenarioId == 4)
  {
    apDensity = 18;
  }
  else if (scenarioId == 0)
  {
    apDensity = 3;
  }
  else
  {
    apDensity = 6;
  }

  uint32_t indexAp[apDensity];
  double pathlossApAp[apDensity][apDensity];
  double pathlossApSta [apDensity][apDensity];
  double pathlossStaSta [apDensity][apDensity];

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
                filenameApAp << pathlossDir << "Scenario3/pathlossMatrixGnbGnb.txt";
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
            if (scenarioId == 5)
            {
                filenameApAp << pathlossDir << "Scenario5/pathlossMatrixGnbGnb.txt";
                filenameApSta << pathlossDir << "Scenario5/pathlossMatrixGnbUe_" << simRound << ".txt";
                filenameIndexAp << pathlossDir << "Scenario5/apIndex_" << simRound << ".txt";
                filenameStaSta << pathlossDir << "Scenario5/pathlossMatrixUeUe_" << simRound << ".txt";
            }
            if (scenarioId == 6)
            {
                filenameApAp << pathlossDir << "Scenario6/pathlossMatrixGnbGnb.txt";
                filenameApSta << pathlossDir << "Scenario6/pathlossMatrixGnbUe_" << simRound << ".txt";
                filenameIndexAp << pathlossDir << "Scenario6/apIndex_" << simRound << ".txt";
                filenameStaSta << pathlossDir << "Scenario6/pathlossMatrixUeUe_" << simRound << ".txt";
            }
            if (scenarioId == 0)
            {
                filenameApAp << pathlossDir << "Tim/pathlossMatrixGnbGnb.txt";
                filenameApSta << pathlossDir << "Tim/pathlossMatrixGnbUe.txt";
                filenameIndexAp << pathlossDir << "Tim/apIndex.txt";
                filenameStaSta << pathlossDir << "Tim/pathlossMatrixUeUe.txt";
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
    for (uint32_t i = 0; i < apDensity; i++)
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
    for (uint32_t i = 0; i < apDensity; i++)
    { 
        for (uint32_t j = 0; j < apDensity; j++)
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
    for (uint32_t i = 0; i < apDensity; i++)
    { 
        for (uint32_t j = 0; j < apDensity; j++)
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
    for (uint32_t i = 0; i < apDensity; i++)
    { 
        for (uint32_t j = 0; j < apDensity; j++)
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

//Configurations
bool g_connect = false;
bool g_customEpisode = false;
uint32_t g_numAps = 4;
Ptr<OpenGymInterface> g_openGymInterface;
Ptr<WifiMacEnv> g_env;
static OutputManager *outputManager;
uint32_t maxNodes = 4;
std::vector<bool> g_firstIterationUdp(maxNodes);
std::vector<bool> g_firstIterationBurst(maxNodes);

Time g_timeStep = MilliSeconds(500);
std::vector<L2Setup*> nr(maxNodes);
std::vector<std::unique_ptr<WifiSetup>> wifi(maxNodes);
uint32_t g_nruPairs;
uint32_t g_wifiPairs;

ApplicationContainer g_nruServerNodes;
ApplicationContainer g_nruClientNodes;
ApplicationContainer g_wifiServerNodes;
ApplicationContainer g_wifiClientNodes;

//Rewards
std::vector<std::pair<uint32_t, double>> g_throughput(maxNodes);
std::vector<std::pair<uint32_t, double>> g_delay(maxNodes);
std::vector<std::pair<uint32_t, double>> g_jitter(maxNodes);
std::vector<double> g_throughputDiff(maxNodes);
std::vector<double> g_jitterDiff(maxNodes);
std::vector<double> g_delayDiff(maxNodes);

std::vector<bool> g_nrIsOccupying (maxNodes);
std::vector<bool> g_wifiIsOccupying (maxNodes);
std::vector<Time> g_nrOccupancy (maxNodes);
std::vector<Time> g_wifiOccupancy (maxNodes);
std::vector<Time> g_nrOccupancyOld (maxNodes);
std::vector<Time> g_wifiOccupancyOld (maxNodes);
std::vector<double> g_airTime (maxNodes);

static void
ResetWifiOccupancy (uint32_t nodeId)
{      
    //std::cout << g_wifiPairs << std::endl;
    if(nodeId >= g_wifiPairs)
    {
        g_wifiIsOccupying[nodeId-g_wifiPairs] = false;
    }
}

static void
WifiOccupancy (uint32_t nodeId, const Time & time)
{
    if(nodeId >= g_wifiPairs)
    {
        g_wifiOccupancy[nodeId-g_wifiPairs] += time;
        g_wifiIsOccupying[nodeId-g_wifiPairs] = true;   
    }
    Simulator::Schedule (time, &ResetWifiOccupancy, nodeId);
}

//States
std::vector<uint32_t> g_trafficType(maxNodes);
std::vector<uint32_t> g_udpLambda(maxNodes);
std::vector<uint32_t> g_packetSize(maxNodes);
std::vector<double> g_ueRxPower(maxNodes);

//Actions
std::vector<uint32_t> g_mcot(maxNodes);
std::vector<uint32_t> g_amsdu(maxNodes);
std::vector<uint32_t> g_ampdu(maxNodes);
std::vector<double> g_txPower(maxNodes);
std::vector<uint32_t> g_mcs(maxNodes);
std::vector<double> g_edThreshold(maxNodes);
std::vector<uint32_t> g_backoffType(maxNodes);
std::vector<uint32_t> g_minCw(maxNodes);
std::vector<uint32_t> g_aifsn(maxNodes);
std::vector<double> g_slotTime(maxNodes);
std::vector<double> g_deferTime(maxNodes);

void 
CreateEnv()
{
    // std::cout << "Debug CreateEnv()" << std::endl;
    Ptr<WifiMacTimeStepEnv> env;
    env = CreateObject<WifiMacTimeStepEnv>(g_numAps, g_trafficType[0], g_udpLambda, g_packetSize[0]);
    g_env = env;
}

void
GiveWifiThroughputAlt (FlowMonitorHelper *flowHelper, Ptr<FlowMonitor> flowMonitor, uint32_t numWifiPairs)
{
    // std::cout << "Debug GiveWifiThroughputAlt()" << std::endl;
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
    std::vector<std::pair<uint32_t, double>> newThroughput(numWifiPairs);
    std::vector<std::pair<uint32_t, double>> newJitter(numWifiPairs);
    std::vector<std::pair<uint32_t, double>> newDelay(numWifiPairs);

    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
        
        // std::cout << "Flow " << i->first << " (" << t.sourceAddress << ":" << t.sourcePort << " -> " << t.destinationAddress << ":" << t.destinationPort << ")" << std::endl;
            
        //     std::cout << "  Tx Packets: " << i->second.txPackets << "\n";
        //     std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
        //     std::cout << "  Tx Offered:  "
        //             << i->second.txBytes * 8.0 / 1000.0 /
        //                     1000.0
        //             << " Mbps\n";
        //     std::cout << "  Rx Packets: " << i->second.rxPackets << "\n";
        //     std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
        //     std::cout << "  Lost Packets: " << i->second.lostPackets << "\n";
            
        double throughputMbps = 0;
        double jitter = 0;
        double delay = 0;
        if (i->second.rxPackets > 0)
        {
        throughputMbps = i->second.rxBytes * 8.0 / g_timeStep.GetSeconds() / 1e6;
        // std::cout << "  Throughput : "<< throughputMbps << " mbps" << std::endl;
        }
        if (i->second.rxPackets > 1)
        {
        delay = i->second.delaySum.GetMilliSeconds () / i->second.rxPackets;
        // std::cout << "  Mean delay : "<< delay << " microseconds" << std::endl;
        
        newThroughput[vecNum] = std::make_pair(t.destinationAddress.Get (), throughputMbps);
        newDelay[vecNum] = std::make_pair(t.destinationAddress.Get (), delay);
        }
        vecNum++;
    } 
    
    double throughputDiff;
    double delayDiff;
    double jitterDiff;

    for (uint32_t k = 0; k < maxNodes; k++)
    {
        for (uint32_t j = 0; j < newThroughput.size(); j++)
        {
            uint32_t throughputIp = 167772417 + k*256;
            if (newThroughput[j].first == throughputIp)
            {
                if (g_firstIterationUdp[k])
                {
                    throughputDiff = newThroughput[j].second;
                    delayDiff = newDelay[j].second;
                    jitterDiff = newJitter[j].second;
                    g_throughputDiff[k] = throughputDiff;
                    g_delayDiff[k] = delayDiff;
                    g_jitterDiff[k] = jitterDiff;
                    // std::cout << "Throughput diff agent " << k << " : " << g_throughputDiff[k] << std::endl;
                    // std::cout << "Delay diff agent " << k << " : " << g_delayDiff[k] << std::endl;
                    // std::cout << "Jitter diff agent " << k << " : " << g_jitterDiff[k] << std::endl;
                }
                else 
                {
                    for (uint32_t i = 0; i < g_throughput.size(); i++)
                    {
                        if (g_throughput[i].first == throughputIp)
                        {
                            throughputDiff = newThroughput[j].second - g_throughput[i].second;
                            delayDiff = newDelay[j].second - g_delay[i].second;
                            if (delayDiff < 0)
                            {
                                delayDiff = newDelay[j].second;
                            }
                            jitterDiff = newJitter[j].second - g_jitter[i].second;
                            g_throughputDiff[k] = throughputDiff;
                            g_delayDiff[k] = delayDiff;
                            g_jitterDiff[k] = jitterDiff;
                            // std::cout << "Throughput diff agent " << k << " : " << g_throughputDiff[k] << std::endl;
                            // std::cout << "Delay diff agent " << k << " : " << g_delayDiff[k] << std::endl;
                            // std::cout << "Jitter diff agent " << k << " : " << g_jitterDiff[k] << std::endl;
                        }
                    }
                }
                g_firstIterationUdp[k] = false;
            }
        }
    } 

    for (uint32_t i = 0; i < g_throughput.size(); i++)
    {    
        g_env->GiveThroughput (g_throughputDiff[i],i);
        g_env->GiveDelay (g_delayDiff[i],i);
        g_throughputDiff[i] = 0; //reset
        g_delayDiff[i] = 0;
        g_jitterDiff[i] = 0;
    }
    g_throughput = newThroughput;
    g_delay = newDelay;
    g_jitter = newJitter;

    Simulator::Schedule (g_timeStep, &GiveWifiThroughputAlt, flowHelper, flowMonitor, numWifiPairs);
}

void
GiveWifiAirTime ()
{
    // std::cout << "Debug GiveWifiAirTime()" << std::endl;
    for (uint32_t i = 0; i < g_airTime.size(); i++)
    {
        g_airTime[i] = g_wifiOccupancy[i].GetMilliSeconds() - g_wifiOccupancyOld[i].GetMilliSeconds();
        g_env->GiveAirTime (g_airTime[i],i);
        g_airTime[i] = 0; //reset
    }
    g_wifiOccupancyOld = g_wifiOccupancy;

    Simulator::Schedule (g_timeStep, &GiveWifiAirTime);
}

void
ChangeWifiMacTypeAlt()
{
//   std::cout << "Debug ChangeWifiMacTypeAlt()" << std::endl;
  if (!g_connect)
  {
    g_connect = true;
    CreateEnv();
  }
  for (uint16_t i = 0; i < g_aifsn.size(); i++)
  {
    // std::cout << "ChangeWifiMacType() new Mac parameters with iteration = i " << i << ": " << std::endl;
    double newTxPower = g_env->ChangeTxPower(g_txPower[i],i);
    uint32_t newMinCw = g_env->ChangeMinCw(g_minCw[i],i);
    uint32_t newAmpdu = g_env->ChangeAmpdu(g_ampdu[i],i);
    uint32_t newMcs = g_env->ChangeMcs(g_mcs[i],i);
    double newEDThreshold = g_env->ChangeEDThreshold(g_edThreshold[i],i);
    uint32_t newAifsn = g_env->ChangeAifsn(g_aifsn[i],i);
    g_txPower[i] = newTxPower;
    g_minCw[i] = newMinCw;
    g_ampdu[i] = newAmpdu;
    g_mcs[i] = newMcs;
    g_edThreshold[i] = newEDThreshold;
    g_aifsn[i] = newAifsn;
  }
  
  Simulator::Schedule (g_timeStep, &ChangeWifiMacTypeAlt);
}

void
ChangeTrafficType()
{
  if (!g_connect)
  {
    g_connect = true;
    CreateEnv();
  }
  for (uint16_t i = 0; i < g_trafficType.size(); i++)
  {
      g_env->ChangeTrafficType(g_trafficType[i],i);
      g_env->ChangeUdpLambda(g_udpLambda[i],i);
      g_env->ChangePacketSize(g_packetSize[i],i);
  }
}

void
UpdateWifiMacParameters (NodeContainer wifiApNodes)
{
    // std::cout << "Debug UpdateWifiMacParameters()" << std::endl;
    // std::cout << wifi.size() << std::endl;
    uint32_t ipv4ifIndex = 1;
    for (uint32_t i = 0; i < g_aifsn.size(); i++)
    {
        wifi[i]->ChangeAmpdu(g_ampdu[i]*1000);
        wifi[i]->GetTxOp()->SetMinCw(g_minCw[i]);
        wifi[i]->GetTxOp()->SetAifsn(g_aifsn[i]);
        wifi[i]->ChangeGnbTxPower(g_txPower[i]);
        wifi[i]->ChangeEdThreshold(g_edThreshold[i]);
        wifi[i]->ChangeMcs(g_mcs[i]);
    }
    Simulator::Schedule (g_timeStep, &UpdateWifiMacParameters, wifiApNodes);
}

void
GiveRxPower (NodeContainer gNbNodes, NodeContainer ueNodes, Ptr<MatrixPropagationLossModel> propagationPathlossMatrix)
{
    // std::cout << "Debug GiveRxPower()" << std::endl;
    double pathloss = 0.0;
    double staRxPower = 0.0;

    for (uint16_t i = 0; i < g_ueRxPower.size (); i++)
    {
        for (uint16_t j = 0; j < g_ueRxPower.size (); j++)
        {
            if (i ==  j) 
            {
                pathloss = propagationPathlossMatrix->GetLoss(gNbNodes.Get (i)->GetObject<MobilityModel> (), ueNodes.Get (j)->GetObject<MobilityModel> ());
            }
            else if (i != j)
            {   
                pathloss = propagationPathlossMatrix->GetLoss(gNbNodes.Get (i)->GetObject<MobilityModel> (), gNbNodes.Get (j)->GetObject<MobilityModel> ());
            }
            g_ueRxPower[j] = g_txPower[i] - pathloss;
            g_env->GiveRxPower (g_ueRxPower[j], i, j);
        }
    }

    Simulator::Schedule (g_timeStep, &GiveRxPower, gNbNodes, ueNodes, propagationPathlossMatrix);
}

void ScheduleNextStateRead ()
{
    // std::cout << "Debug ScheduleNextStateRead()" << std::endl;
    g_env->ScheduleNextStateRead();
}

int
main (int argc, char *argv[])
{
    for (uint32_t i = 0; i < maxNodes; i++)
    {
        g_nrIsOccupying[i] = false;
        g_wifiIsOccupying[i] = false;
    }    

    //Scenario parameters
    uint32_t numAgents = 4;
    bool customEpisode = false;
    std::string envNumber = "";
    uint32_t timeStep = 500;
    bool isEvaluation = false;
    std::string outputDirectory = "";

    uint32_t numWifiPairs = numAgents;
    uint32_t numNruPairs = 0;
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
    uint32_t udpLambda = 1000;
    uint32_t packetSize = 1500; // bytes
    double interval = static_cast<double> (packetSize * 8) / bitRate;
    Time udpInterval = Seconds (interval);
    uint16_t operatorPortWifi = 1234;
    uint16_t operatorPortNru = 1235;

    double frameRate = 30;
    std::string targetDataRate = "40Mbps";
    std::string vrAppName = "VirusPopper";
    uint32_t fragmentSize = 1500;

    std::string trafficType = "UDP_CBR";

    //Distance parameters
    double d1 = 1.0; // meters
    double d2 = 1.0; // meters
    double d3 = 1.0; // meters

    //Simulation parameters
    double simTime = 2.0; // seconds
    double ueX = 1.0; // meters
    std::string simTag = "default";
    std::string outputDir = "./";
    bool enableNr = false;
    bool enableWifi = true;
    bool doubleTechnology = false;
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
    
    int Duplex = 0;
    uint8_t nruMcs = -1;

    double ccaThresholdAp = -62.0; //dBm Wifi
    double ccaThresholdSta = -62.0; //dBm Wifi
    std::string wifiStandard = "11ax";
    int frameAggregation = 0; //0 no agg. 1 MPDU max,2 MSDU max
    uint32_t wifiMpdu = 0;
    uint32_t wifiMsdu = 0;
    uint32_t wifiMinCw = 15;
    uint32_t wifiAifsn = 2;
    uint32_t wifiBackoffType = 3;
    uint32_t wifiSlotTime = 9;
    uint32_t wifiMcs = 0;

    std::string gnbCamType = "ns3::NrCat4LbtAccessManager";
    double lbtEDThreshold = -79.0;
    double lbtSlotTime = 9.0; //Microseconds
    double lbtDeferTime = 16.0; //MicroSeconds
    double lbtMcot = 5.0; //MilliSeconds
    std::string lbtBackoffType = "Binary";
    double cat2EDThreshold = -72.0; // dBm
    double cat3and4EDThreshold = -72.0; // dBm
    double cat2ED = -69.0;
    double cat2DeferTime = 10; //MicroSeconds
    uint32_t cat3CW = 15;
    uint32_t cat4RetryLimit = 0;
    uint32_t cat4MinCw = 15;
    uint32_t cat4MaxCw = 1023;
    uint32_t cat4CwUpdateRule = NrCat4LbtAccessManager::ANY_NACK; //alternatives : NrCat4LbtAccessManager::ALL_NACKS, NrCat4LbtAccessManager::NACKS_10_PERCENT, NrCat4LbtAccessManager::NACKS_80_PERCENT 
    uint32_t cat4LinearBackoff = 3;
    double onoffChangeTime = 10.0;

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
    uint32_t udpLambdaVector[maxNodes];
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
        udpLambdaVector[i] = udpLambda;
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
    cmd.AddValue ("numAgents", "number of DRL agents", numAgents);

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
    // cmd.AddValue ("totalTxPower5",
    //               "Total TX power (dBm) that will be proportionally assigned to"
    //               " bandwidth parts depending on each BWP bandwidth ",
    //               totalTxPowerVector[4]);
    // cmd.AddValue ("totalTxPower6",
    //               "Total TX power (dBm) that will be proportionally assigned to"
    //               " bandwidth parts depending on each BWP bandwidth ",
    //               totalTxPowerVector[5]);
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
    // cmd.AddValue ("ueTxPower5",
    //               "TX power (dBm) for UEs",
    //                ueTxPowerVector[4]);
    // cmd.AddValue ("ueTxPower6",
    //               "TX power (dBm) for UEs",
    //                ueTxPowerVector[5]);              
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
    // cmd.AddValue ("gnbCamType5", "The gNB CAM of gNb", gNbCamVector[4]);
    // cmd.AddValue ("gnbCamType6", "The gNB CAM of gNb", gNbCamVector[5]);
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
    // cmd.AddValue ("cat2EDThreshold5", 
    //               "The ED threshold to be used by Lbt category 2 algorithm (dBm). Allowed range [-100.0, 0.0]., ",
    //                cat2EDThresholdVector[4]);
    // cmd.AddValue ("cat2EDThreshold6", 
    //               "The ED threshold to be used by Lbt category 2 algorithm (dBm). Allowed range [-100.0, 0.0]., ",
    //                cat2EDThresholdVector[5]);
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
    // cmd.AddValue ("cat3and4EDTreshold5",
    //               "The ED threshold to be used by Lbt category 3 and 4 algorithm (dBm). Allowed range [-100.0, 0.0].",
    //                cat3and4EDThresholdVector[4]);
    // cmd.AddValue ("cat3and4EDTreshold6",
    //               "The ED threshold to be used by Lbt category 3 and 4 algorithm (dBm). Allowed range [-100.0, 0.0].",
    //                cat3and4EDThresholdVector[5]);
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
    // cmd.AddValue ("lbtEDThreshold5",
    //               "CCA-ED threshold for channel sensing by Lbt algorithm (dBm). Allowed range [-100.0, 0.0].",
    //                lbtEDThresholdVector[4]);
    // cmd.AddValue ("lbtEDThreshold6",
    //               "CCA-ED threshold for channel sensing by Lbt algorithm (dBm). Allowed range [-100.0, 0.0].",
    //                lbtEDThresholdVector[5]);
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
    // cmd.AddValue ("lbtSlotTime5",
    //               "The duration of a Slot by  Lbt algorithm (microseconds).",
    //                lbtSlotTimeVector[4]);
    // cmd.AddValue ("lbtSlotTime6",
    //               "The duration of a Slot by  Lbt algorithm (microseconds).",
    //                lbtSlotTimeVector[5]);              
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
    // cmd.AddValue ("lbtDeferTime5",
    //               "TimeInterval to defer during CCA by Lbt (microseconds).",
    //                lbtDeferTimeVector[4]);
    // cmd.AddValue ("lbtDeferTime6",
    //               "TimeInterval to defer during CCA by Lbt (microseconds).",
    //                lbtDeferTimeVector[5]);
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
    // cmd.AddValue ("lbtMcot5",
    //               "Duration of channel access grant by Lbt algorithm (milliseconds). Allowed range [2.0, 20.0].",
    //                lbtMcotVector[4]);
    // cmd.AddValue ("lbtMcot6",
    //               "Duration of channel access grant by Lbt algorithm (milliseconds). Allowed range [2.0, 20.0].",
    //                lbtMcotVector[5]);
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
    // cmd.AddValue ("cat2ED5",
    //               "CCA-ED threshold for channel sensing by Lbt 2 algorithm (dBm). Allowed range [-100.0, 0.0].",
    //                cat2EDVector[4]);
    // cmd.AddValue ("cat2ED6",
    //               "CCA-ED threshold for channel sensing by Lbt 2 algorithm (dBm). Allowed range [-100.0, 0.0].",
    //                cat2EDVector[5]);
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
    // cmd.AddValue ("cat2DeferTime5",
    //               "TimeInterval to defer during CCA by Lbt 2 algorithm (microseconds).",
    //                cat2DeferTimeVector[4]);
    // cmd.AddValue ("cat2DeferTime6",
    //               "TimeInterval to defer during CCA by Lbt 2 algorithm (microseconds).",
    //                cat2DeferTimeVector[5]);
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
    // cmd.AddValue ("cat3CW5",
    //               "The default fixed value of the CW in the case that Cat 3 LBT is used.",
    //                cat3CWVector[4]);
    // cmd.AddValue ("cat3CW6",
    //               "The default fixed value of the CW in the case that Cat 3 LBT is used.",
    //                cat3CWVector[5]); 
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
    // cmd.AddValue ("cat4RetryLimit5",
    //               "How many times to try to retransmit with the current CW before reseting it to the MinCw value for Cat 4 Lbt.",
    //                cat4RetryLimitVector[4]);
    // cmd.AddValue ("cat4RetryLimit6",
    //               "How many times to try to retransmit with the current CW before reseting it to the MinCw value for Cat 4 Lbt.",
    //                cat4RetryLimitVector[5]);                                            
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
    // cmd.AddValue ("cat4MinCw5",
    //               "The minimum value of the contention window used by Lbt category 4 algorithm.",
    //                cat4MinCwVector[4]);
    // cmd.AddValue ("cat4MinCw6",
    //               "The minimum value of the contention window used by Lbt category 4 algorithm.",
    //                cat4MinCwVector[5]);
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
    // cmd.AddValue ("cat4MaxCw5",
    //               "The maximum value of the contention window used by Lbt category 4 algorithm.",
    //                cat4MaxCwVector[4]);
    // cmd.AddValue ("cat4MaxCw6",
    //               "The maximum value of the contention window used by Lbt category 4 algorithm.",
    //                cat4MaxCwVector[5]);
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
    // cmd.AddValue ("cat4CwUpdateRule5",
    //               "Rule according which CW will be updated used by Lbt category 4 algorithm.",
    //                cat4CwUpdateRuleVector[4]);
    // cmd.AddValue ("cat4CwUpdateRule6",
    //               "Rule according which CW will be updated used by Lbt category 4 algorithm.",
    //                cat4CwUpdateRuleVector[5]);      
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
    // cmd.AddValue ("onoffChangeTime5",
    //               "The default fixed value of the change time in OnOffAccessManager is used.",
    //                onoffChangeTimeVector[4]);
    // cmd.AddValue ("onoffChangeTime6",
    //               "The default fixed value of the change time in OnOffAccessManager is used.",
    //                onoffChangeTimeVector[5]);
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
    // cmd.AddValue ("nruMcs5",
    //               "The mcs used for ACM in the Nru.",
    //                nruMcsVector[4]);
    // cmd.AddValue ("nruMcs6",
    //               "The mcs used for ACM in the Nru.",
    //                nruMcsVector[5]);
    cmd.AddValue ("trafficType",
                  "The traffic type used to transport data.",
                   trafficType);
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
    // cmd.AddValue ("trafficType5",
    //               "The traffic type used to transport data.",
    //                trafficTypeVector[4]);
    // cmd.AddValue ("trafficType6",
    //               "The traffic type used to transport data.",
    //                trafficTypeVector[5]);
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
    // cmd.AddValue ("udpSaturationRate5",
    //               "The UDP saturation rate used.",
    //                UDP_SATURATION_RATE_VECTOR[4]);
    // cmd.AddValue ("udpSaturationRate6",
    //               "The UDP saturation rate used.",
    //                UDP_SATURATION_RATE_VECTOR[5]);
    cmd.AddValue ("udpLambda",
                  "Lambda for udp interval.",
                   udpLambda);
    cmd.AddValue ("udpLambda1",
                  "Lambda for udp interval.",
                   udpLambdaVector[0]);
    cmd.AddValue ("udpLambda2",
                  "Lambda for udp interval.",
                   udpLambdaVector[1]);
    cmd.AddValue ("udpLambda3",
                  "Lambda for udp interval.",
                   udpLambdaVector[2]);
    cmd.AddValue ("udpLambda4",
                  "Lambda for udp interval.",
                   udpLambdaVector[3]);
    // cmd.AddValue ("udpLambda5",
    //               "Lambda for udp interval.",
    //                udpLambdaVector[4]);
    // cmd.AddValue ("udpLambda6",
    //               "Lambda for udp interval.",
    //                udpLambdaVector[5]);
    cmd.AddValue ("packetSize",
                  "Packet size value to transport data.",
                   packetSize);
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
    // cmd.AddValue ("packetSize5",
    //               "Packet size value to transport data.",
    //                packetSizeVector[4]);
    // cmd.AddValue ("packetSize6",
    //               "Packet size value to transport data.",
    //                packetSizeVector[5]);
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
    // cmd.AddValue ("nodeRate5",
    //               "Node rate used to transport data.",
    //                nodeRateVector[4]);
    // cmd.AddValue ("nodeRate6",
    //               "Node rate used to transport data.",
    //                nodeRateVector[5]);
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
    // cmd.AddValue ("frameRate5",
    //               "Frame rate for bursty traffic to transport data.",
    //                frameRateVector[4]);
    // cmd.AddValue ("frameRate6",
    //               "Frame rate for bursty traffic to transport data.",
    //                frameRateVector[5]);               
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
    // cmd.AddValue ("targetDataRate5",
    //               "The target data rate for bursty traffic to transport data.",
    //                targetDataRateVector[4]);
    // cmd.AddValue ("targetDataRate6",
    //               "The target data rate for bursty traffic to transport data.",
    //                targetDataRateVector[5]);
    cmd.AddValue ("fragmentSize",
                  "The fragment size for bursty traffic used to transport data.",
                   fragmentSize);                       
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
    // cmd.AddValue ("fragmentSize5",
    //               "The fragment size for bursty traffic used to transport data.",
    //                fragmentSizeVector[4]);
    // cmd.AddValue ("fragmentSize6",
    //               "The fragment size for bursty traffic used to transport data.",
    //                fragmentSizeVector[5]);  
    cmd.AddValue ("customEpisode",
                  "Using custom parameters for episode.",
                   customEpisode);  
    cmd.AddValue ("isEvaluation",
                  "Is this an evaluation.",
                   isEvaluation);     
    cmd.AddValue ("envNumber",
                  "Serial number of the running environment",
                   envNumber);  
    cmd.AddValue ("timeStep",
                  "Interval of one time step in the environment",
                   timeStep);                         

    cmd.Parse (argc, argv);                    

    LogComponentEnable ("wifi-ai-mac", LOG_LEVEL_ALL);

    uint32_t apDensity;
    if(scenarioId == 1 || scenarioId == 2 || scenarioId == 4)
    {
        apDensity = 18;
    }
    else if(scenarioId == 0)
    {
        apDensity = 3;
    }
    else
    {
        apDensity = 6;
    }

    g_openGymInterface = OpenGymInterface::Get(envNumber);

    // numNruPairs = numAgents;
    numWifiPairs = numAgents;
    g_nruPairs = numNruPairs;
    g_wifiPairs = numWifiPairs;

    g_customEpisode = customEpisode;
    g_timeStep =  MilliSeconds(timeStep);

    nr.resize(numNruPairs);
    wifi.resize(numWifiPairs);
    g_throughput.resize(numAgents);
    g_delay.resize(numAgents);
    g_jitter.resize(numAgents);
    g_throughputDiff.resize(numAgents);
    g_jitterDiff.resize(numAgents);
    g_delayDiff.resize(numAgents);

    g_nrIsOccupying.resize(numNruPairs);
    g_wifiIsOccupying.resize(numWifiPairs);
    g_nrOccupancy.resize(numNruPairs);
    g_wifiOccupancy.resize(numWifiPairs);
    g_nrOccupancyOld.resize(numNruPairs);
    g_wifiOccupancyOld.resize(numWifiPairs);
    g_airTime.resize(numAgents);
    g_trafficType.resize(numAgents);
    g_udpLambda.resize(numAgents);
    g_packetSize.resize(numAgents);
    g_ueRxPower.resize(numAgents);

    g_mcot.resize(numNruPairs);
    g_ampdu.resize(numWifiPairs);
    g_amsdu.resize(numWifiPairs);
    g_txPower.resize(numAgents);
    g_mcs.resize(numAgents);
    g_edThreshold.resize(numAgents);
    g_backoffType.resize(numAgents);
    g_minCw.resize(numAgents);
    g_slotTime.resize(numAgents);
    g_deferTime.resize(numAgents);
    g_aifsn.resize(numWifiPairs);

    g_numAps = numAgents;
    for (uint32_t i = 0; i < maxNodes; i++)
    {
        g_firstIterationUdp[i] = true;
        g_firstIterationBurst[i] = true;
    }
    
    for (uint32_t i = 0; i < g_edThreshold.size(); i++)
    {
        g_edThreshold[i] = -62;
        g_udpLambda[i] = udpLambdaVector[i];
        g_packetSize[i] = packetSize;
        if (trafficType == "UDP_CBR")
        {
            g_trafficType[i] = 0;
        }
        else if (trafficType == "BURST")
        {
            g_trafficType[i] = 1;
        }
    }
    
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

        double tempDataRate = fragmentSize*8*udpLambdaVector[i]/1e6;
        targetDataRateVector[i] = std::to_string(tempDataRate) + "Mbps";
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
  
    NS_LOG_DEBUG ("Create base stations and mobile terminals");
    NodeContainer staNodes;
    staNodes.Create (numApStaPairs);
    NodeContainer apNodes;
    apNodes.Create (numApStaPairs);
    allWirelessNodes = NodeContainer (staNodes,apNodes);

    NS_LOG_DEBUG ("Create mobility");
    //Set AP and UE position array
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
    double staPositionVector [apDensity][3];
    double apPositionVector [apDensity][3];
    uint32_t indexAp[apDensity];

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
                filenamePosAp << pathlossDir << "Scenario3/gNbLayoutMatrix.txt";
                filenameIndexAp << pathlossDir << "Scenario3/apIndex_" << simRound << ".txt";
                filenamePosSta << pathlossDir << "Scenario3/ueCoordinates_" << simRound << ".txt";
            }
            if (scenarioId == 4)
            {
                filenamePosAp << pathlossDir << "Scenario4/gNbLayoutMatrix.txt";
                filenameIndexAp << pathlossDir << "Scenario4/apIndex_" << simRound << ".txt";
                filenamePosSta << pathlossDir << "Scenario4/ueCoordinates_" << simRound << ".txt";
            }
            if (scenarioId == 5)
            {
                filenamePosAp << pathlossDir << "Scenario5/gNbLayoutMatrix.txt";
                filenameIndexAp << pathlossDir << "Scenario5/apIndex_" << simRound << ".txt";
                filenamePosSta << pathlossDir << "Scenario5/ueCoordinates_" << simRound << ".txt";
            }
            if (scenarioId == 6)
            {
                filenamePosAp << pathlossDir << "Scenario6/gNbLayoutMatrix.txt";
                filenameIndexAp << pathlossDir << "Scenario6/apIndex_" << simRound << ".txt";
                filenamePosSta << pathlossDir << "Scenario6/ueCoordinates_" << simRound << ".txt";
            }
            if (scenarioId == 0)
            {
                filenamePosAp << pathlossDir << "Tim/gNbLayoutMatrix.txt";
                filenameIndexAp << pathlossDir << "Tim/apIndex.txt";
                filenamePosSta << pathlossDir << "Tim/ueCoordinates.txt";
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

        std::ifstream inFile;
        
        inFile.open(filenameIndexAp.str(), std::ios_base::in);
        if (!inFile.is_open ()) 
        { 
            NS_LOG_ERROR ("Can't open file!!!" ); 
        }
        std::cout << "AP Index : ";
        for (uint32_t i = 0; i < apDensity; i++)
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

        for (uint32_t i = 0; i < apDensity; i++)
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
        for (uint32_t i = 0; i < apDensity; i++)
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

    Packet::EnablePrinting ();

    //SqliteOutputManager manager (outputDir, numWifiPairs, outputDir + ss.str (), ss.str (), seed, numNruPairs, simRound);
    //outputManager = &manager;
    
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

    if(enableWifi && numNruPairs == 0)
    {
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
        uint32_t indexWifiAp = 0;
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
            wifi[indexWifiAp] = std::unique_ptr<WifiSetup> (new WifiSetup (NodeContainer (it->second), v.second,
                                    channel, propagation, spectrumPropagation,
                                    frequency, bandwidth, totalTxPower, totalTxPower, ccaThresholdAp, ccaThresholdSta,
                                    standard, "primero",frameAggregation, wifiMpdu, wifiMsdu,
                                    wifiMinCw, wifiAifsn, wifiBackoffType,
                                    wifiSlotTime,
                                    wifiMcs));
            ueIpIfaceOperatorWifi.Add(wifi[indexWifiAp]->AssignIpv4ToUe (address));
            wifi[indexWifiAp]->AssignIpv4ToStations (address);
            remoteWifiIface = wifi[indexWifiAp]->ConnectToRemotes (remoteHostContainer, remoteAddress.str().c_str());
            //wifi.back()->SetSinrCallback (MakeCallback (&OutputManager::SinrStore, &manager));
            //wifi.back()->SetMacTxDataFailedCb (MakeCallback (&OutputManager::MacDataTxFailed, &manager));
            wifi[indexWifiAp]->SetChannelOccupancyCallback (MakeCallback (&WifiOccupancy));
            //wifi.back()->SetChannelRTACallback (MakeCallback (&OutputManager::ChannelRequestTime, &manager));
            indexWifiAp++;
        }
    }

    if (enableWifi && numNruPairs == 0)
    {
        NS_LOG_DEBUG("Create Wifi net device");
        Ptr<WifiNetDevice> wifiNetDevice = DynamicCast<WifiNetDevice>(wifi[0]->GetGnbDev().Get(0));
        spectrumChannel = DynamicCast<SpectrumChannel>(wifiNetDevice->GetPhy()->GetChannel());
        std::cout << spectrumChannel << std::endl;
        spectrumChannel->AddPropagationLossModel (propagationPathlossMatrix);
    }

    //Setting up routing
    uint32_t ifaceId = 1;
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

    if (enableWifi)
    { 
        for (uint32_t i = 0; i < numWifiPairs; i++)
        {
            if(trafficType == "UDP_CBR")
            {
                serverAppsWifi.Add (setUdpPoissonServer (wifiStaNodesAlt[i], operatorPortWifi));
                clientAppsWifi.Add (setUdpPoissonClientAi (wifiStaNodesAlt[i], remoteHostContainer, remoteHost, ueIpIfaceOperatorWifiAlt[i], operatorPortWifi, udpLambdaVector[i], packetSizeVector[i], appStartTime));
            }
            else if (trafficType == "BURST")
            {
                serverAppsWifi.Add(setBurstServerAi (wifiStaNodesAlt[i], operatorPortWifi, ueIpIfaceOperatorWifiAlt[i], fragmentSizeVector[i], frameRateVector[i], targetDataRateVector[i], remoteHostContainer, appStartTime));
                clientAppsWifi.Add(setBurstClient (remoteHostContainer, remoteWifiIface, wifiStaNodesAlt[i], operatorPortWifi));
            }
        }
    }
    
    pingApps.Start(Seconds(0.300));
    pingApps.Stop(Seconds(0.500));

    PopulateArpCache ();
    Packet::EnablePrinting ();

    std::cout << "UE" << std::endl;
    PrintIpAddress (remoteHostContainer);
    PrintRoutingTable (remoteHostContainer);

    // Simulator::Schedule (MicroSeconds (100), &TimePasses);

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

    // ChangeTrafficType();
    Simulator::Schedule (MilliSeconds(0), &GiveWifiThroughputAlt, &flowHelper, monitor, numWifiPairs);
    Simulator::Schedule (MilliSeconds(0), &GiveRxPower, wifiApNodes, wifiStaNodes, propagationPathlossMatrix);
    Simulator::Schedule (MilliSeconds(0), &GiveWifiAirTime);
    Simulator::Schedule (MilliSeconds(0), &ChangeWifiMacTypeAlt);
    Simulator::Schedule (MilliSeconds(0), &ScheduleNextStateRead);
    Simulator::Schedule (MilliSeconds(0), &UpdateWifiMacParameters, wifiApNodes);
    
    Simulator::Stop (Seconds (simTime));

    Simulator::Run ();

    time_t t = time (nullptr);
    struct tm tm = *localtime (&t);
    std::ofstream outputFile;
    std::string outputFileName = outputDirectory + "ns3eval_results_" + envNumber + "_" + std::to_string(simRound) + "_" + std::to_string(tm.tm_hour) + "h, " + std::to_string(tm.tm_min) + "m, " + std::to_string(tm.tm_sec) + "s.txt";
    
    if(isEvaluation)
    {
        outputFile.open(outputFileName);
        std::string endMessage = "\nStarting new episode " + envNumber + " with --numAgents=" + std::to_string(numAgents) + " --simTime=" + std::to_string(simTime) +  " --simRound=" + std::to_string(simRound) + " --trafficType=" + trafficType + " --packetSize=" + std::to_string(packetSize) + " --udpLambda=" + std::to_string(udpLambdaVector[0]) + "," + std::to_string(udpLambdaVector[1]) + "," + std::to_string(udpLambdaVector[2]) + "," + std::to_string(udpLambdaVector[3]) + "," + std::to_string(udpLambdaVector[4]) + "," + std::to_string(udpLambdaVector[5]) + "\n";
        outputFile << endMessage;
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
            outputFile << "Flow " << i->first << " (" << t.sourceAddress << ":" << t.sourcePort << " -> " << t.destinationAddress << ":" << t.destinationPort << ") \n";
            std::cout << "  Tx Packets: " << i->second.txPackets << "\n";
            outputFile << " Tx Packets: " << i->second.txPackets << "\n";
            std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
            outputFile << "  Tx Bytes:   " << i->second.txBytes << "\n";
            std::cout << "  TxOffered:  "
                    << i->second.txBytes * 8.0 / (simTime - appStartTime) / 1000.0 /
                            1000.0
                    << " Mbps\n";
            outputFile << "  TxOffered:  "
                    << i->second.txBytes * 8.0 / (simTime - appStartTime) / 1000.0 /
                            1000.0
                    << " Mbps\n";
            std::cout << "  Rx Packets: " << i->second.rxPackets << "\n";
            outputFile << "  Rx Packets: " << i->second.rxPackets << "\n";
            std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
            outputFile << "  Rx Bytes:   " << i->second.rxBytes << "\n";
            //std::cout << "  Lost Packets: " << i->second.lostPackets << "\n";
            //outputFile << "  Lost Packets: " << i->second.lostPackets << "\n";

            if (i->second.rxPackets > 0)
            {
                if (isNr || isWigig)
                {
                    std::string technology = isNr ? "nr" : "wifi";
                    std::cout << "  Technology : "<< technology << std::endl;
                    outputFile << "  Technology : " << technology << " \n";
                    if (isNr)
                    {
                        throughputMbps = i->second.rxBytes * 8.0 / (simTime - appStartTime) / 1e6;
                        std::cout << "  Throughput : "<< throughputMbps << " Mbps" << std::endl;
                        outputFile << "  Throughput : " << throughputMbps << " Mbps" << "\n";
                    }
                    
                    if (isWigig)
                    {
                        throughputMbps = i->second.rxBytes * 8.0 / (simTime - appStartTime - 0.01) / 1e6;
                        std::cout << "  Throughput : "<< throughputMbps << " Mbps" << std::endl;
                        outputFile << "  Throughput : " << throughputMbps << " Mbps" << "\n";
                    }
                    
                    double delay = 0.0;
                    double jitter = 0.0;

                    delay = i->second.delaySum.GetMicroSeconds () / i->second.rxPackets;
                    std::cout << "  Mean delay : " << delay << " microseconds" << std::endl;
                    outputFile << "  Mean delay : " << delay << "\n";
                    jitter = i->second.jitterSum.GetMicroSeconds () / (i->second.rxPackets - 1);
                    std::cout << "  Mean jitter : " << jitter << " microseconds" << std::endl;
                    outputFile << "  Mean jitter : " << jitter << "\n";
                    std::cout << "  Last packet delay: " << i->second.lastDelay.As(Time::MS) << "\n";
                    outputFile << "  Last packet delay: " << i->second.lastDelay.As(Time::MS) << "\n";
                    
                    averageFlowThroughput += i->second.rxBytes * 8.0 / (simTime - appStartTime) / 1000 / 1000;
                    averageFlowDelay += 1000 * i->second.delaySum.GetSeconds() / i->second.rxPackets;
                    delayValues[cont] = 1000 * i->second.delaySum.GetSeconds() / i->second.rxPackets;
                    cont++;
                }
                outputFile << "\n";
            }
            outputFile.flush();
        }

        //manager.Close ();
        std::sort(delayValues, delayValues + stats.size());

        double FiftyTileFlowDelay = delayValues[stats.size() / 2];

        std::cout << "Summary of flows " << std::endl;
        outputFile << "Summary of flows " << "\n";
        std::cout << "  Mean flow throughput : "<< averageFlowThroughput / stats.size () << " Mbps" <<std::endl;
        outputFile << "  Mean flow throughput : " << averageFlowThroughput / stats.size () << " Mbps" << "\n";
        std::cout << "  Mean flow delay : " << averageFlowDelay / stats.size()  << " ms" << std::endl;
        outputFile << "  Mean flow delay : " << averageFlowDelay / stats.size()  << " ms" << "\n";
        std::cout << "  Median flow delay : " << FiftyTileFlowDelay  << " ms" <<std::endl;
        outputFile << "  Median flow delay : " << FiftyTileFlowDelay << " ms" << "\n";
        
        double averageAirTime = 0;
        if(enableWifi)
        {
            for (uint32_t i = 0; i < g_wifiOccupancy.size(); i++)
            {
                averageAirTime = averageAirTime + g_wifiOccupancy[i].GetMilliSeconds();
            }
        }
        averageAirTime = averageAirTime/numAgents;
        std::cout << "  Mean airtime : " << averageAirTime  << " ms" <<std::endl;
        outputFile << "  Mean airtime : " << averageAirTime << " ms" << "\n";
        outputFile.flush();
    }
    
    outputFile.close();

    std::cout << "Simulation ending...." << std::endl;
    g_openGymInterface->NotifySimulationEnd();
    
    std::cout << "Destroying simulation...." << std::endl;
    Simulator::Destroy ();
    
    return 0;
}
