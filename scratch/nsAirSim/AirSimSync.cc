// standard includes
#include <sstream>
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
// custom includes
#include "AirSimSync.h"
// externs
// extern int nzmqIOthread;
// extern int numOfCong;
// extern float congRate;
// extern float congX, congY, congRho;
// extern std::vector< std::vector<float> > initPostEnb;
// extern int segmentSize;
// extern std::vector<string> uavsName;
// static float updateGranularity;
// static EventId event;
using namespace std;

NS_LOG_COMPONENT_DEFINE("AIRSIM_SYNC");

std::ostream& operator<<(ostream & os, const AirSimSync::NetConfig &config)
{
    os << "nzmqIOthread " << config.nzmqIOthread << " seg size:" << config.segmentSize << " numOfCong:" << config.numOfCong << " congRate:" << config.congRate  << endl;
    os << "congX:" << config.congX << " congY:" << config.congY << " congRho:" << config.congRho  << endl;
    os << "UAV names(" << config.uavsName.size() << "):" << endl;
    for(auto it:config.uavsName){
        os << it << ",";
    }
    os << endl;

    os << "Enb pos:"  << endl;
    for(int i = 0; i < config.initPostEnb.size(); i++){
        os << i << "(" << config.initPostEnb[i][0] << ", " << config.initPostEnb[i][1] << ", " << config.initPostEnb[i][2] << ")"  << endl;
    }
    return os;
}

AirSimSync::AirSimSync(zmq::context_t &context): event()
{
    zmqRecvSocket = zmq::socket_t(context, ZMQ_PULL);
    zmqRecvSocket.connect("tcp://localhost:" + to_string(AIRSIM2NS_CTRL_PORT));
    zmqSendSocket = zmq::socket_t(context, ZMQ_PUSH);
    zmqSendSocket.bind("tcp://*:" + to_string(NS2AIRSIM_CTRL_PORT));
}
AirSimSync::~AirSimSync()
{
    ;
}

void AirSimSync::readNetConfigFromAirSim(AirSimSync::NetConfig &config)
{
    zmq::message_t message;
    zmqRecvSocket.recv(message, zmq::recv_flags::none);
    std::string s(static_cast<char*>(message.data()), message.size());
    std::istringstream ss(s);
    int numOfUav, numOfEnb;
    
    NS_LOG_INFO("Config: " << (const char*)message.data());
    ss >> config.nzmqIOthread >> config.segmentSize >> config.updateGranularity >> config.numOfCong >> config.congRate >> config.congX >> config.congY >> config.congRho;
    ss >> numOfUav;
    config.uavsName = std::vector<std::string>(numOfUav);
    for(int i = 0; i < numOfUav; i++){
        ss >> config.uavsName[i];
    }
    ss >> numOfEnb;
    config.initPostEnb = std::vector< std::vector<float> >(numOfEnb, std::vector<float>(3));
    for(int i = 0; i < numOfEnb; i++){
        ss >> config.initPostEnb[i][0] >> config.initPostEnb[i][1] >> config.initPostEnb[i][2];
    }

    zmqRecvSocket.setsockopt(ZMQ_RCVTIMEO, MAX_RECV_TIMEO);
    updateGranularity = config.updateGranularity;
}
void AirSimSync::startAirSim()
{
    zmq::message_t ntf(1);
    // notify AirSim
    zmqSendSocket.send(ntf, zmq::send_flags::none);
}
void AirSimSync::takeTurn(Ptr<GcsApp> &gcsApp, std::vector< Ptr<UavApp> > &uavsApp)
{
    float now = Simulator::Now().GetSeconds();
    zmq::message_t message;
    zmq::recv_result_t res;
    zmq::message_t ntf(1);
    
    // NS_LOG_INFO("Time: " << now);

    // notify AirSim
    zmqSendSocket.send(ntf, zmq::send_flags::dontwait);
    
    // AirSim's turn at time t
    res = zmqRecvSocket.recv(message, zmq::recv_flags::none); // block until AirSim sends any (nofitied by AirSim)
    
    std::string s(static_cast<char*>(message.data()), message.size());
    if((!res.has_value() || res.value() < 0) || (s == "End")){
        zmq::message_t ntf("bye");
        if(event.IsRunning()){
            NS_LOG_INFO("AirSimSync cancel");
            Simulator::Cancel(event);
        }
        zmqSendSocket.send(ntf, zmq::send_flags::none);
        zmqRecvSocket.close();
        zmqSendSocket.close();
        NS_LOG_INFO("AirSimSync stopped");
        Simulator::Stop(Seconds(0));
        return;
    }
    
    // ns' turn at time t, AirSim at time t + 1
    // packet send
    gcsApp->mobilityUpdateDirect();
    if(gcsApp){
        gcsApp->scheduleTx();
    }
    for(int i = 0; i < uavsApp.size(); i++){
        uavsApp[i]->scheduleTx();
    }

    // will fire at time t + 1
    Time tNext(Seconds(updateGranularity));
    event = Simulator::Schedule(tNext, &AirSimSync::takeTurn, this, gcsApp, uavsApp);
}
