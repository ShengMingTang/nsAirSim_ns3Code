// std includes
#include <cstdlib>
// ns3 includes
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/stats-module.h"
// custom includes
#include "congApp.h"

using namespace std;
using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("CongApp");

CongApp::CongApp(): m_event()
{
    // Todo
}

CongApp::~CongApp()
{
    // Todo
}

TypeId CongApp::GetTypeId(void)
{
    static TypeId tid = TypeId("CongApp")
        .SetParent<Application>()
        .SetGroupName("ns3_AirSim")
        .AddConstructor<CongApp>()
    ;
    return tid;
}

/* Init ns stuff, rPC client connection and zmq socket init, connect */
void CongApp::Setup(Ptr<Socket> socket, Address myAddress, Address peerAddress,
    float congRate, std::string name
)
{
    m_socket = socket;
    m_address = myAddress;
    m_peerAddress = peerAddress;
    m_congRate = congRate;
    m_name = name;
}

/* Bind ns sockets and logging*/
void CongApp::StartApplication(void)
{
    m_running = true;

    // ns socket routines
    m_socket->Bind();
    if(m_socket->Connect(m_peerAddress) != 0){
        NS_FATAL_ERROR("Cong connect error");
    };
    
    m_socket->SetRecvCallback(
        MakeCallback(&CongApp::recvCallback, this)
    );

    scheduleTx();

    NS_LOG_INFO("[Cong " << m_name << " starts]");
}
void CongApp::StopApplication(void)
{
    m_running = false;

    if(m_event.IsRunning()){
        Simulator::Cancel(m_event);
    }
    if(m_socket){
        m_socket->Close();
    }

    NS_LOG_INFO("[Cong " << m_name << " stopped]");
}
/* <sim_time> <payload> */
void CongApp::scheduleTx(void)
{
    float now = Simulator::Now().GetSeconds();
    // r [0, 1]
    float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    r = rand() % 2 ? r : -r;

    Ptr<Packet> packet = Create<Packet>(m_congRate*1024);

    Time tNext(Seconds(max((float)1e-3, 1/m_congRate + r)));
    m_event = Simulator::Schedule(tNext, &CongApp::Tx, this, m_socket, packet);
    NS_LOG_INFO("time: " << now << ", [" << m_name << " send]");
}
/* <from-address> <payload> then forward to application code */
void CongApp::recvCallback(Ptr<Socket> socket)
{
    float now = Simulator::Now().GetSeconds();
    Ptr<Packet> packet;
    Address from;

    packet = socket->RecvFrom(from);
    // packet = socket->Recv();

    NS_LOG_INFO("time: " << now << ", [" << m_name << " recv]");
}