#include "ns3/lte-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/config-store.h"
//#include "ns3/gtk-config-store.h"
#include "ns3/flow-monitor-module.h"

using namespace ns3;


int
main (int argc, char *argv[])
{
  
  uint32_t maxBytes = 0;
  double distance = 500.0;
  double simTime = 5.0;
  // Command line arguments
  CommandLine cmd;
  cmd.Parse(argc, argv);

  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<EpcHelper>  epcHelper = CreateObject<EpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);
  lteHelper->SetSchedulerType("ns3::RrFfMacScheduler");
  lteHelper->SetPathlossModelType("ns3::FriisPropagationLossModel");
  //lteHelper->GetRlcStats ()->SetAttribute("EpochDuration", TimeValue());
  lteHelper->SetAttribute("EpsBearerToRlcMapping", EnumValue (LteHelper::RLC_AM_ALWAYS));
  

  ConfigStore inputConfig;
  inputConfig.ConfigureDefaults();
  // parse again so you can override default values from the command line
  cmd.Parse(argc, argv);

  Ptr<Node> pgw = epcHelper->GetPgwNode ();

   // Create a single RemoteHost
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

 

 // Create the Internet
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.01)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  // interface 0 is localhost, 1 is the p2p device
  
  //specify routes so that the remote host can reach LTE UE
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);


  NodeContainer ueNodes;
  NodeContainer enbNodes;
  enbNodes.Create(1);
  ueNodes.Create(1);

  // Install Mobility Model
    
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector(distance, 0, 0));
  MobilityHelper mobilityUe;
  mobilityUe.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobilityUe.SetPositionAllocator(positionAlloc);
  mobilityUe.Install(ueNodes);


  MobilityHelper mobilityEnb;
  mobilityEnb.SetMobilityModel("ns3::ConstantPositionMobilityModel"); 
  mobilityEnb.Install(enbNodes);
 

  // Install LTE Devices to the nodes
  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbNodes);
  NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice (ueNodes);

  lteHelper->Attach (ueLteDevs, enbLteDevs.Get (0));
      

  // Install the IP stack on the UEs
  internet.Install (ueNodes);
 

 
  // Assign IP address to UE
 
   Ptr<Node> ueNode = ueNodes.Get (0);
   Ptr<NetDevice> ueLteDevice = ueLteDevs.Get (0);
   Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueLteDevice));
    // Set the default gateway for the UE
   Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
   ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);


   
  lteHelper->ActivateEpsBearer (ueLteDevs, EpsBearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT), EpcTft::Default ());



// SINK!!!!!!
// Create a PacketSinkApplication and install it on ue
  uint16_t sinkPort = 8080;
  PacketSinkHelper packetSinkHelper ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), sinkPort));
  ApplicationContainer sinkApps = packetSinkHelper.Install (ueNodes);
  sinkApps.Start (Seconds (1.));
  sinkApps.Stop (Seconds (simTime));

//SEND!!!!!
  
  
  BulkSendHelper source ("ns3::TcpSocketFactory", InetSocketAddress (ueIpIface.GetAddress (0), sinkPort));
  source.SetAttribute ("MaxBytes", UintegerValue (maxBytes));
  ApplicationContainer sourceApps = source.Install (remoteHost);
  sourceApps.Start (Seconds (2.0));
  sourceApps.Stop (Seconds (simTime));


  p2ph.EnablePcapAll("my-lena-simple-epc-tcp");
  lteHelper->EnableTraces ();
  internet.EnablePcapIpv4 ("my-lena-simple-epc-tcp", remoteHost);


  Simulator::Stop(Seconds(simTime));

//FLOW MONITOR
  
  Ptr <FlowMonitor> flowmon;
  FlowMonitorHelper flowmonhelper;
  flowmon = flowmonhelper.Install (remoteHost);
  flowmon = flowmonhelper.Install (ueNodes);




  Simulator::Run();

  flowmon -> SerializeToXmlFile ("results.xml", false, false);

  /*GtkConfigStore config;
  config.ConfigureAttributes();*/

  Simulator::Destroy();
  return 0;

}

