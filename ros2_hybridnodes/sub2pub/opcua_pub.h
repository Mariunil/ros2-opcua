#ifndef ONEFILE_H
#define ONEFILE_H

#include "ros2Sub_opcuaPub_node.h"

#include <signal.h>
#include <pthread.h>

//opcua library (amalgamated)
#include "open62541.h"

//starts a publishing server, publishing the global variable for node.h
int opcuaPublisherRun();


#endif //header-guard