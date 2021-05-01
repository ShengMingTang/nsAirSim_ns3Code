// std includes
#include <vector>
#include <ctime>
// ns3 includes
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"

#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/lte-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/lte-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-probe.h"
// zmq includes
#include <zmq.hpp>
// json lib
// #include "<nlohmann/json.hpp>"
// custom includes
#include "gcsApp.h"
#include "uavApp.h"
#include "congApp.h"
#include "AirSimSync.h"
#include "netMeasure.h"

using namespace std;
using namespace ns3;

static int nRbs = 6; // see https://i.imgur.com/q55uR8T.png
AirSimSync::NetConfig config;

NS_LOG_COMPONENT_DEFINE ("NS_AIRSIM");

int main(int argc, char *argv[])
{
  // local vars
  zmq::context_t context(1);
  float dist;


  srand (static_cast <unsigned> (time(0)));
  LogComponentEnable("NS_AIRSIM", LOG_LEVEL_INFO);
  // LogComponentEnable("GcsApp", LOG_LEVEL_INFO);
  // LogComponentEnable("UavApp", LOG_LEVEL_INFO);
  // LogComponentEnable("CongApp", LOG_LEVEL_INFO);
  // LogComponentEnable("AIRSIM_SYNC", LOG_LEVEL_INFO);
  
  CommandLine cmd (__FILE__);
  cmd.AddValue("dist", "dist", dist);

  cmd.Parse (argc, argv);
  
  NS_LOG_INFO("Read Net config");

  AirSimSync sync(context);
  sync.readNetConfigFromAirSim(config);
  NS_LOG_INFO(config);

  Time::SetResolution(Time::NS);
  // GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));
  
  // ==========================================================================
  // Config TCP socket
  // https://www.nsnam.org/doxygen/classns3_1_1_tcp_socket.html
  Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(config.segmentSize));
  Config::SetDefault("ns3::TcpSocket::SndBufSize", UintegerValue(429496729));
  Config::SetDefault("ns3::TcpSocket::RcvBufSize", UintegerValue(429496729));

  
  // Packet level settings
  ns3::Packet::EnablePrinting();

  // ==========================================================================
  // Node containers
  NodeContainer uavNodes;
  uavNodes.Create(config.uavsName.size());
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create(1);
  NodeContainer enbNodes;
  enbNodes.Create(config.initPostEnb.size());
  NodeContainer congNodes;
  congNodes.Create(config.numOfCong);

  Ptr<Node> pgw;
  Ptr<Node> sgw;

  NS_LOG_INFO("Creating Topology");

  // ==========================================================================
  NS_LOG_INFO("Create UAV nodes");
  // allocate initial position of each uav
  Ptr<ListPositionAllocator> initPosUavAlloc = CreateObject<ListPositionAllocator>();
  for(uint32_t i = 0; i < uavNodes.GetN(); i++){
    initPosUavAlloc->Add(Vector(0, 0, 0));
  }
  // mobility setup
  // Initial position is left to AirSim, update in the start of an application
  MobilityHelper mobilityUav;
  mobilityUav.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobilityUav.SetPositionAllocator(initPosUavAlloc);
  mobilityUav.Install(uavNodes); // allocate corresponding indexed initial position
  // Internet stack
  InternetStackHelper internetUav;
  internetUav.Install(uavNodes);
  
  // ==========================================================================
  NS_LOG_INFO("Build LTE Topology");
  Config::SetDefault ("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue (false));
  Config::SetDefault ("ns3::LteSpectrumPhy::DataErrorModelEnabled", BooleanValue (true));
  Config::SetDefault ("ns3::PfFfMacScheduler::HarqEnabled", BooleanValue (false));
  Config::SetDefault ("ns3::PfFfMacScheduler::CqiTimerThreshold", UintegerValue (10));
  Config::SetDefault ("ns3::LteEnbRrc::EpsBearerToRlcMapping",EnumValue(LteEnbRrc::RLC_AM_ALWAYS));
  Config::SetDefault ("ns3::LteEnbNetDevice::UlBandwidth", UintegerValue(nRbs));
  Config::SetDefault ("ns3::LteEnbNetDevice::DlBandwidth", UintegerValue(nRbs));
  Config::SetDefault ("ns3::LteUePhy::EnableUplinkPowerControl", BooleanValue (false));
  Config::SetDefault ("ns3::LteUePhy::TxPower", DoubleValue(0));

  NS_LOG_INFO("Setup LTE helper");
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
  lteHelper->SetHandoverAlgorithmType ("ns3::NoOpHandoverAlgorithm"); // disable automatic handover
  lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::FriisPropagationLossModel"));

  NS_LOG_INFO("Setup EPC helper");
  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
  lteHelper->SetEpcHelper(epcHelper);
  // sgw/pgw node @@ where is sgw?
  pgw = epcHelper->GetPgwNode();
  sgw = epcHelper->GetSgwNode();

  // ==========================================================================
  // Create a single remoteHost (later be installed with pgw)
  // This remote host will be in GCS
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internetRemoteHost;
  internetRemoteHost.Install (remoteHostContainer);
  
  // EPC
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("1Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.00001)));
  
  NetDeviceContainer pgwRemoteDevices = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (pgwRemoteDevices);
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);

  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4>());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  NS_LOG_INFO("Create eNb");
  Ptr<ListPositionAllocator> positionAllocEnb = CreateObject<ListPositionAllocator> ();
  for(int i = 0; i< config.initPostEnb.size(); i++){
    positionAllocEnb->Add(Vector(0, 0, 0));
  }
  // mobility
  MobilityHelper mobilityLte;
  mobilityLte.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilityLte.SetPositionAllocator(positionAllocEnb);
  mobilityLte.Install (enbNodes);
  // internet stack
  NetDeviceContainer enbDevices;
  enbDevices = lteHelper->InstallEnbDevice(enbNodes);

  NS_LOG_INFO("Create UE");
  NetDeviceContainer ueDevices;
  for(int i = 0; i < uavNodes.GetN(); i++){
    ueDevices.Add(lteHelper->InstallUeDevice(uavNodes.Get(i)));
  }
  // assign ipv4 interfaces
  Ipv4InterfaceContainer ueIpIfaces;
  for(int i = 0; i< uavNodes.GetN(); i++){
    Ptr<Node> ue = uavNodes.Get(i);
    Ptr<NetDevice> ueLteDevice = ueDevices.Get(i);
    Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueLteDevice));
    ueIpIfaces.Add(ueIpIface);
    // set the default gateway for the UE
    Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ue->GetObject<Ipv4> ());
    ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

    // Replaced by AttachToClosestEnb()
    // lteHelper->Attach(ueLteDevice, enbDevices.Get(0));
    
    // @@ what does this mean ?
    lteHelper->ActivateDedicatedEpsBearer (ueLteDevice, EpsBearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT), EpcTft::Default ());
  }
  lteHelper->AttachToClosestEnb(ueDevices, enbDevices);

  // ==========================================================================
  // add application to GCS
  Ptr<Node> gcsNode = remoteHostContainer.Get(0);
  Address gcsSinkAddress(InetSocketAddress (remoteHostAddr, GCS_PORT_START));
  Ptr<Socket> gcsTcpSocket = Socket::CreateSocket(gcsNode, TcpSocketFactory::GetTypeId());
  // uav
  std::map<std::string, Ptr<ConstantPositionMobilityModel> > uavsMobility;
  std::vector<Address> uavsAddress;
  Ptr<GcsApp> gcsApp = CreateObject<GcsApp>();
  std::vector< Ptr<UavApp> > uavsApp;
  
  // add application to uavNodes
  for(int i = 0; i < uavNodes.GetN(); i++){  
    // GCS -> UAV (sink)
    uint16_t uavPort = UAV_PORT_START;
    Ptr<Node> uav = uavNodes.Get(i);
    Ipv4Address uavAddress = ueIpIfaces.GetAddress(i);
    Ptr<Socket> uavTcpSocket = Socket::CreateSocket(uavNodes.Get(i), TcpSocketFactory::GetTypeId());
    Address uavMyAddress(InetSocketAddress(uavAddress, uavPort));
    Ptr<UavApp> app = CreateObject<UavApp>();
    
    uavNodes.Get(i)->AddApplication(app);
    app->Setup(context, uavTcpSocket, uavMyAddress, gcsSinkAddress,
      AIRSIM2NS_PORT_START + i, NS2AIRSIM_PORT_START + i, config.uavsName[i]
    );
    app->SetStartTime(Seconds(UAV_APP_START_TIME));
    app->SetStopTime(Simulator::GetMaximumSimulationTime());

    uavsMobility[config.uavsName[i]] = uavNodes.Get(i)->GetObject<ConstantPositionMobilityModel>();
    
    uavsAddress.push_back(InetSocketAddress(uavAddress, uavPort));
    uavsApp.push_back(app);
  }

  gcsNode->AddApplication(gcsApp);
  gcsApp->Setup(context, gcsTcpSocket, InetSocketAddress(Ipv4Address::GetAny(), GCS_PORT_START), 
    uavsMobility,
    AIRSIM2NS_GCS_PORT , NS2AIRSIM_GCS_PORT
  );
  gcsApp->SetStartTime(Seconds(GCS_APP_START_TIME));
  gcsApp->SetStopTime(Simulator::GetMaximumSimulationTime());

  // ==========================================================================
  NS_LOG_INFO("Create Congestion nodes");
  // Mobility
  MobilityHelper mobilityCong;
  mobilityCong.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilityCong.SetPositionAllocator("ns3::UniformDiscPositionAllocator",
					"rho", DoubleValue(config.congRho),
					"X", DoubleValue(config.congX),
					"Y", DoubleValue(config.congY));
  mobilityCong.Install(congNodes);
  // Internet stack
  InternetStackHelper internetCong;
  internetCong.Install(congNodes);
  // NetDevice
  NetDeviceContainer congUeDevices;
  for (int i = 0; i < congNodes.GetN (); i++){
    congUeDevices.Add(lteHelper->InstallUeDevice (congNodes.Get(i)));
  }
  // assign ipv4 interfaces
  Ipv4InterfaceContainer congUeIpfaces;
  for (uint32_t i = 0; i < congNodes.GetN (); i++){
    Ipv4AddressHelper ipv4;
    Ptr<Node> congue = congNodes.Get (i);
    Ptr<NetDevice> congUeLteDevice = congUeDevices.Get (i);
    Ipv4InterfaceContainer congueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (congUeLteDevice));
    congUeIpfaces.Add(congueIpIface);

    // set the default gateway for the UE
    Ptr<Ipv4StaticRouting> congueStaticRouting = ipv4RoutingHelper.GetStaticRouting (congue->GetObject<Ipv4> ());
    congueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

    // Replaced by AttachToClosestEnb()
    // lteHelper->Attach(congUeLteDevice, enbDevices.Get(0));
    
    lteHelper->ActivateDedicatedEpsBearer (congUeLteDevice, EpsBearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT), EpcTft::Default ());
  }
  lteHelper->AttachToClosestEnb(congUeDevices, enbDevices);
  
  std::vector< Ptr<CongApp> > congsApp;
  // Add application to cong node
  for(int i = 0; i < congNodes.GetN(); i++){  
    uint16_t congPort = CONG_PORT_START; // use the same port as uav does
    Ptr<Node> cong = congNodes.Get(i);
    Ipv4Address congAddress = congUeIpfaces.GetAddress(i);
    Ptr<Socket> congTcpSocket = Socket::CreateSocket(congNodes.Get(i), TcpSocketFactory::GetTypeId());
    Address congMyAddress(InetSocketAddress(Ipv4Address::GetAny(), congPort));
    Ptr<CongApp> app = CreateObject<CongApp>();
    
    congNodes.Get(i)->AddApplication(app);
    app->Setup(congTcpSocket, congMyAddress, gcsSinkAddress,
      config.congRate, "cong" + to_string(i)
    );
    app->SetStartTime(Seconds(CONG_APP_START_TIME));
    app->SetStopTime(Simulator::GetMaximumSimulationTime());

    congsApp.push_back(app);
  }

  // ==========================================================================
  // measure
  enableThroughPutMeasure("nsAirSim_throughput.csv", gcsNode, uavNodes, config.uavsName);
  enableMobilityMeasure("nsAirSim_mobility.csv", uavNodes, config.uavsName);

  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.Install(gcsNode);

  // ==========================================================================
  // Run
  sync.startAirSim();
  Simulator::ScheduleNow(&AirSimSync::takeTurn, &sync, gcsApp, uavsApp);
  Simulator::Stop(Seconds(1.99));
  Simulator::Run();
  
  // ==========================================================================
  // post
  // std::cout << ueIpIfaces.GetN() << endl;
  // for(int i = 0; i < ueIpIfaces.GetN(); i++){
      // NS_LOG_INFO("UAV address = "  << ueIpIfaces.GetAddress(i));
  // }
  // NS_LOG_INFO("GCS address = "  << remoteHostAddr);
  
  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats ();
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i){
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
    std::cout << "dist=" << dist << " source=" << t.sourceAddress << ", dest=" << t.destinationAddress << ", throughput= "<< i->second.txBytes * 8.0 / (i->second.timeLastTxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds()+0.001) / 1000 / 1000  << " Mbps"  << endl;

    // sender
    // if(t.sourceAddress == ueIpIfaces.GetAddress(0)){
    // }
  }

  // ==========================================================================
  Simulator::Destroy();
  return 0;
}