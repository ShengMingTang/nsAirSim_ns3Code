// std includes
#include <vector>
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
// zmq includes
#include <zmq.hpp>
// json lib
// #include "<nlohmann/json.hpp>"
// custom includes
#include "gcsApp.h"
#include "uavApp.h"
#include "congApp.h"
#include "AirSimSync.h"

using namespace std;
using namespace ns3;


// modified by AirSimSync.cc
int nzmqIOthread = 3;
int numOfCong;
float congRate;
float congX, congY, congRho;
std::vector< std::vector<float> > initPostEnb;
int segmentSize;
std::vector<string> uavsName;

zmq::context_t context(nzmqIOthread);

NS_LOG_COMPONENT_DEFINE ("NS_AIRSIM");
int main(int argc, char *argv[])
{
  LogComponentEnable("NS_AIRSIM", LOG_LEVEL_INFO);
  LogComponentEnable("GcsApp", LOG_LEVEL_INFO);
  LogComponentEnable("UavApp", LOG_LEVEL_INFO);
  LogComponentEnable("CongApp", LOG_LEVEL_INFO);
  LogComponentEnable("AIRSIM_SYNC", LOG_LEVEL_INFO);
  
  // temp
  std::stringstream ss;
  
  initNsAirSim();

  NS_LOG_INFO("Read Net config");
  readNetConfigFromAirSim();
  NS_LOG_INFO("nzmqIOthread " << nzmqIOthread << " seg size:" << segmentSize << " numOfCong:" << numOfCong << " congRate:" << congRate);
  NS_LOG_INFO("congX:" << congX << " congY:" << congY << " congRho:" << congRho);
  for(auto it:uavsName){
    ss << it << ",";
  }
  NS_LOG_INFO("UAV names(" << uavsName.size() << "):"<< ss.str());
  std::stringstream().swap(ss);
  NS_LOG_INFO("Enb pos:");
  for(int i = 0; i < initPostEnb.size(); i++){
    NS_LOG_INFO(i << "(" << initPostEnb[i][0] << ", " << initPostEnb[i][1] << ", " << initPostEnb[i][2] << ")");
  }


  Time::SetResolution(Time::NS);
  // GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));
  Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(segmentSize));
  
  // Packet level settings
  ns3::Packet::EnablePrinting();

  NS_LOG_INFO("Creating Topology");

  NS_LOG_INFO("Create UAV nodes");
  NodeContainer uavNodes;
  uavNodes.Create(uavsName.size());
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
  
  NS_LOG_INFO("Build LTE Topology");
  // @@Todo Configure Default LTE Parameters
  Config::SetDefault ("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue (false));
  Config::SetDefault ("ns3::LteSpectrumPhy::DataErrorModelEnabled", BooleanValue (true));
  Config::SetDefault ("ns3::PfFfMacScheduler::HarqEnabled", BooleanValue (false));
  Config::SetDefault ("ns3::PfFfMacScheduler::CqiTimerThreshold", UintegerValue (10));
  Config::SetDefault ("ns3::LteEnbRrc::EpsBearerToRlcMapping",EnumValue(LteEnbRrc::RLC_AM_ALWAYS));
  Config::SetDefault ("ns3::LteEnbNetDevice::UlBandwidth", UintegerValue(100));
  Config::SetDefault ("ns3::LteEnbNetDevice::DlBandwidth", UintegerValue(100));
  Config::SetDefault ("ns3::LteUePhy::EnableUplinkPowerControl", BooleanValue (false));

  NS_LOG_INFO("Setup LTE helper");
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
  lteHelper->SetHandoverAlgorithmType ("ns3::NoOpHandoverAlgorithm"); // disable automatic handover
  lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::FriisPropagationLossModel"));

  NS_LOG_INFO("Setup EPC helper");
  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
  lteHelper->SetEpcHelper(epcHelper);
  // sgw/pgw node @@ where is sgw?
  Ptr<Node> pgw = epcHelper->GetPgwNode();

  // Create a single remoteHost (later be installed with pgw)
  // This remote host will be in GCS
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create(1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internetRemoteHost;
  internetRemoteHost.Install (remoteHostContainer);
  
  // EPC
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
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
  NodeContainer enbNodes;
  enbNodes.Create(initPostEnb.size());
  Ptr<ListPositionAllocator> positionAllocEnb = CreateObject<ListPositionAllocator> ();
  for(int i = 0; i< initPostEnb.size(); i++){
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

  // add application to GCS
  Ptr<Node> gcsNode = remoteHostContainer.Get(0);
  Address gcsSinkAddress(InetSocketAddress (remoteHostAddr, GCS_PORT_START));
  Ptr<Socket> gcsTcpSocket = Socket::CreateSocket(gcsNode, TcpSocketFactory::GetTypeId());
  // uav
  std::map<std::string, Ptr<ConstantPositionMobilityModel> > uavsMobility;
  // std::vector<Ptr<ConstantPositionMobilityModel> > uavsMobility;
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
    Address uavMyAddress(InetSocketAddress(Ipv4Address::GetAny(), uavPort));
    Ptr<UavApp> app = CreateObject<UavApp>();
    
    uavNodes.Get(i)->AddApplication(app);
    app->Setup(uavTcpSocket, uavMyAddress, gcsSinkAddress,
      AIRSIM2NS_PORT_START + i, NS2AIRSIM_PORT_START + i, uavsName[i]
    );
    app->SetStartTime(Seconds(UAV_APP_START_TIME));

    uavsMobility[uavsName[i]] = uavNodes.Get(i)->GetObject<ConstantPositionMobilityModel>();
    
    uavsAddress.push_back(InetSocketAddress(uavAddress, uavPort));
    uavsApp.push_back(app);
  }

  gcsNode->AddApplication(gcsApp);
  gcsApp->Setup(gcsTcpSocket, InetSocketAddress(Ipv4Address::GetAny(), GCS_PORT_START), 
    uavsMobility,
    AIRSIM2NS_GCS_PORT , NS2AIRSIM_GCS_PORT
  );
  gcsApp->SetStartTime(Seconds(GCS_APP_START_TIME));

  /* Todo Create congestion node */
  NS_LOG_INFO("Create Congestion nodes");
  NodeContainer congNodes;
  congNodes.Create(numOfCong);
  // Mobility
  MobilityHelper mobilityCong;
  mobilityCong.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  // @@
  mobilityCong.SetPositionAllocator("ns3::UniformDiscPositionAllocator",
					"rho", DoubleValue(congRho),
					"X", DoubleValue(congX),
					"Y", DoubleValue(congY));
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
      congRate, "cong" + to_string(i)
    );
    app->SetStartTime(Seconds(CONG_APP_START_TIME));

    congsApp.push_back(app);
  }


  Simulator::ScheduleNow(&nsAirSimTakeTurn, gcsApp, uavsApp);
  nsAirSimSimBegin();
  Simulator::Run();
  Simulator::Destroy();
  std::cout << "[NS] over" << endl;
  return 0;
}