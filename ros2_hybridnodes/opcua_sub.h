#ifndef OPCUA_SUB_H
#define OPCUA_SUB_H

#include "opcuaSub_ros2Pub_node.h"

#include <open62541/plugin/log_stdout.h>
#include <open62541/plugin/pubsub.h>
#include <open62541/plugin/pubsub_udp.h>
#include <open62541/server.h>
#include <open62541/server_config_default.h>
#include <open62541/plugin/ua_pubsub_networkmessage.h> //manually placed

#include <signal.h>

#ifdef UA_ENABLE_PUBSUB_ETH_UADP
#include <open62541/plugin/pubsub_ethernet.h>
#endif


/* The only exposed function in the OPC UA subscriber.
   Through a internal loop it update the global variable with the 
   subscribed value */
int opcuaSubscriberRun();


#endif //header-guard