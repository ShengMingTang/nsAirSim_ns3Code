#ifndef INCLUDE_AIRSIMSYNC_H
#define INCLUDE_AIRSIMSYNC_H
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
#include "gcsApp.h"
#include "uavApp.h"
// externs
extern zmq::context_t context;


#define NS2AIRSIM_PORT_START (5000)
#define AIRSIM2NS_PORT_START (6000)
#define NS2AIRSIM_GCS_PORT (4999)
#define AIRSIM2NS_GCS_PORT (4998)

#define UAV_PORT_START (3000)
#define GCS_PORT_START (4000)
#define CONG_PORT_START (UAV_PORT_START)

#define NS2AIRSIM_CTRL_PORT (8000)
#define AIRSIM2NS_CTRL_PORT (8001)

#define GCS_APP_START_TIME (0.1)
#define UAV_APP_START_TIME (0.2)
#define CONG_APP_START_TIME (UAV_APP_START_TIME)

using namespace std;

void initNsAirSim();
void readNetConfigFromAirSim();

void nsAirSimSimBegin();
void nsAirSimTakeTurn(Ptr<GcsApp> &gcsApp, std::vector< Ptr<UavApp> > &duavsApp);

#endif