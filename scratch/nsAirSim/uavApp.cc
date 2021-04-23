// std includes
#include <fstream>
#include <sstream>
#include <limits>
// ns3 includes
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/stats-module.h"
// AirSim includes
#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
// zmq includes
#include <zmq.hpp>
// custom includes
#include "uavApp.h"
#include "AirSimSync.h"

// extern
extern zmq::context_t context;

using namespace std;
using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("UavApp");

UavApp::UavApp(): m_event()
{
    // Todo
}

UavApp::~UavApp()
{
    // Todo
}

TypeId UavApp::GetTypeId(void)
{
    static TypeId tid = TypeId("UavApp")
        .SetParent<Application>()
        .SetGroupName("ns3_AirSim")
        .AddConstructor<UavApp>()
    ;
    return tid;
}

/* Init ns stuff, rPC client connection and zmq socket init, connect */
void UavApp::Setup(Ptr<Socket> socket, Address myAddress, Address peerAddress,
    int zmqRecvPort, int zmqSendPort, std::string name
)
{
    m_name = name;
    m_socket = socket;
    m_address = myAddress;
    m_peerAddress = peerAddress;

    m_zmqSocketSend = zmq::socket_t(context, ZMQ_PUSH);
    m_zmqSocketSend.bind("tcp://*:" + to_string(zmqSendPort));
    m_zmqSocketRecv = zmq::socket_t(context, ZMQ_PULL);
    m_zmqSocketRecv.connect("tcp://localhost:" + to_string(zmqRecvPort));
}

/* Bind ns sockets and logging*/
void UavApp::StartApplication(void)
{
    m_running = true;

    // ns socket routines
    m_socket->Bind();
    if(m_socket->Connect(m_peerAddress) != 0){
        NS_FATAL_ERROR("UAV connect error");
    };
    
    /* @@ We may leave the job to application */
    // send my name
    std::string s = "name " + m_name;
    Ptr<Packet> packet = Create<Packet>((const uint8_t*)(s.c_str()), s.size());
    if(m_socket->Send(packet) == -1){
        NS_FATAL_ERROR(m_name << "sends my name Error");
    }
    
    m_socket->SetRecvCallback(
        MakeCallback(&UavApp::recvCallback, this)
    );

    NS_LOG_INFO("[ " << m_name << " starts]");
}
void UavApp::StopApplication(void)
{
    m_running = false;

    if(m_event.IsRunning()){
        Simulator::Cancel(m_event);
    }
    if(m_socket){
        m_socket->Close();
    }

    NS_LOG_INFO("[ " << m_name << " stopped]");
}
/* <sim_time> <payload> */
void UavApp::scheduleTx(void)
{
    zmq::message_t message;
    float now = Simulator::Now().GetSeconds();
    zmq::recv_result_t res;
    while((res = m_zmqSocketRecv.recv(message, zmq::recv_flags::dontwait)) && res.has_value() && res.value() != -1){ // EAGAIN
        std::string smessage(static_cast<char*>(message.data()), message.size());
        std::stringstream ss(smessage);
        float simTime;
        std::string payload;

        ss >> simTime >> payload;

        Ptr<Packet> packet = Create<Packet>((const uint8_t*)(payload.c_str()), payload.length()+1);
        // m_socket->Send(packet);
        Time tNext(Seconds(simTime - now));
        Simulator::Schedule(tNext, &UavApp::Tx, this, m_socket, packet);
        NS_LOG_INFO("time: " << simTime << ", [" << m_name << " send]" << payload);

        message.rebuild();
    }
}
/* <from-address> <payload> then forward to application code */
void UavApp::recvCallback(Ptr<Socket> socket)
{
    Ptr<Packet> packet;
    Address from;
    char buff[300] = {0};
    float now = Simulator::Now().GetSeconds();

    packet = socket->RecvFrom(from);
    // packet = socket->Recv();
    
    std::stringstream ss;
    std::string s;
    packet->CopyData((uint8_t *)buff, sizeof(buff)-1);
    ss << from << " " << buff;
    s = ss.str();
    zmq::message_t message(s.begin(), s.end());
    NS_LOG_INFO("time: " << now << ", [" << m_name << " recv] content: " << buff);
    m_zmqSocketSend.send(message, zmq::send_flags::none);
}