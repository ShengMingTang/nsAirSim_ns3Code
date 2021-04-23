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
extern int nzmqIOthread;
extern int numOfCong;
extern float congRate;
extern float congX, congY, congRho;
extern std::vector< std::vector<float> > initPostEnb;
extern int segmentSize;
extern std::vector<string> uavsName;
extern zmq::context_t context;

using namespace std;


static zmq::socket_t zmqRecvSocket, zmqSendSocket;
static float updateGranularity;

NS_LOG_COMPONENT_DEFINE("AIRSIM_SYNC");

void initNsAirSim()
{
    zmqRecvSocket = zmq::socket_t(context, ZMQ_PULL);
    zmqRecvSocket.connect("tcp://localhost:" + to_string(AIRSIM2NS_CTRL_PORT));
    zmqSendSocket = zmq::socket_t(context, ZMQ_PUSH);
    zmqSendSocket.bind("tcp://*:" + to_string(NS2AIRSIM_CTRL_PORT));
}

/*
ns airsim
# <- : nzmqIOthread segmentSize updateGranularity numOfCong congRate [congX congY congRho] numOfUav [name1 ]+ numOfEnb [px py pz ]+
*/
void readNetConfigFromAirSim()
{
    zmq::message_t message;
    zmqRecvSocket.recv(message, zmq::recv_flags::none);
    std::string s(static_cast<char*>(message.data()), message.size());
    std::istringstream ss(s);
    int numOfUav, numOfEnb;
    
    ss >> nzmqIOthread >> segmentSize >> updateGranularity >> numOfCong >> congRate >> congX >> congY >> congRho;
    ss >> numOfUav;
    uavsName = std::vector<std::string>(numOfUav);
    for(int i = 0; i < numOfUav; i++){
        ss >> uavsName[i];
    }
    ss >> numOfEnb;
    initPostEnb = std::vector< std::vector<float> >(numOfEnb, std::vector<float>(3));
    for(int i = 0; i < numOfEnb; i++){
        ss >> initPostEnb[i][0] >> initPostEnb[i][1] >> initPostEnb[i][2];
    }
}

void nsAirSimSimBegin()
{
    zmq::message_t ntf(1);
    // notify AirSim
    zmqSendSocket.send(ntf, zmq::send_flags::none);
}

void nsAirSimTakeTurn(Ptr<GcsApp> &gcsApp, std::vector< Ptr<UavApp> > &uavsApp)
{
    float now = Simulator::Now().GetSeconds();
    zmq::message_t message;
    NS_LOG_INFO("Time: " << now);

    zmq::message_t ntf(1);
    // notify AirSim
    zmqSendSocket.send(ntf, zmq::send_flags::dontwait);

    // AirSim's turn at time t
    zmqRecvSocket.recv(message, zmq::recv_flags::none); // block until AirSim sends any (nofitied by AirSim)

    std::string s(static_cast<char*>(message.data()), message.size());
    if(s == "End"){
        zmq::message_t message("End");
        zmqSendSocket.send(message, zmq::send_flags::none);
        Simulator::Stop(Seconds (0.0));
    }
    
    // ns' turn at time t, AirSim at time t + 1
    // packet send
    // gcsApp->mobilityUpdateDirect();
    gcsApp->scheduleTx();
    for(int i = 0; i < uavsApp.size(); i++){
        uavsApp[i]->scheduleTx();
    }

    // will fire at time t + 1
    Time tNext(Seconds(updateGranularity));
    Simulator::Schedule(tNext, &nsAirSimTakeTurn, gcsApp, uavsApp);
}