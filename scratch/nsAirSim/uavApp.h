#ifndef INCLUDE_UAVAPP_H
#define INCLUDE_UAVAPP_H

// custom includes
#include <queue>
#include <map>
// ns3 includes
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/stats-module.h"
// zmq includes
#include <zmq.hpp>

using namespace std;
using namespace ns3;

class UavApp: public Application
{
public:
    UavApp();
    virtual ~UavApp();

    /**
    * Register this type.
    * \return The TypeId.
    */
    static TypeId GetTypeId(void);
    void Setup(Ptr<Socket> socket, Address myAddress, Address peerAddress,
        int zmqRecvPort, int zmqSendPort, std::string name
    );

    void scheduleTx(void);
private:
    virtual void StartApplication (void);
    virtual void StopApplication (void);

    void Tx(Ptr<Socket> socket, Ptr<Packet> packet) {socket->Send(packet);}

    void recvCallback(Ptr<Socket> socket);

    bool m_running;
    // ns stuff
    Ptr<Socket>     m_socket;
    Address         m_address;
    Address         m_peerAddress;
    EventId         m_event;

    // custom application member
    string m_name;
    zmq::socket_t m_zmqSocketSend;
    zmq::socket_t m_zmqSocketRecv;
};

#endif