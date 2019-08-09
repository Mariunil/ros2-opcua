/*
CLIENT/SERVER DEMO EXAMPLE: Simple OPC UA server sending fake "sensor" data

The OPC-UA server sets up a "Piece counter" node containing fake sensor data. 
The data models a sensor counting passings. The server utilizes a seperate thread 
to monitor "the sensor", which in reality is a loop incrementing a variable on a node
hosted by the server.

*/


#include <open62541/plugin/log_stdout.h>
#include <open62541/server.h>
#include <open62541/server_config_default.h>

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <signal.h>



// Define the sampling time for the sensor
#define SLEEP_TIME_MILLIS 50
// Define the ID of the node externally as it will be needed inside the thread
#define COUNTER_NODE_ID 20305


// Global variable to keep the number of counted objects
int32_t numberOfParts = 0;

// To allow for ctrl-c triggered stop
UA_Boolean running = true;
static void stopHandler(int sig) {
    UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "Received ctrl-c");
    running = false;
}


static void addCounterSensorVariable(UA_Server* server) {	

	/* Here we are setting a specific ID for the node but the library
	   can do it if we don't specify it */
	UA_NodeId counterNodeId = UA_NODEID_NUMERIC(1, COUNTER_NODE_ID);	

	// We specify the name of the OPC UA node
	UA_QualifiedName counterName = UA_QUALIFIEDNAME(1, "Piece Counter[pieces]");	

	UA_VariableAttributes attr = UA_VariableAttributes_default;
	attr.description = UA_LOCALIZEDTEXT("en_US","Piece Counter (units:pieces)");
	attr.displayName = UA_LOCALIZEDTEXT("en_US","Piece Counter");
    attr.dataType = UA_TYPES[UA_TYPES_INT32].typeId;	

    // Set the initial value to 0
	UA_Int32 counterValue = 0;
    UA_Variant_setScalarCopy(&attr.value, &counterValue, &UA_TYPES[UA_TYPES_INT32]);	

    // Include the variable to the server under the root object folder
	UA_Server_addVariableNode(server, counterNodeId,
		UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
		UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
		counterName, UA_NODEID_NULL, attr, NULL, NULL);
}




// Thread used to monitor the "sensor"
void* mainSensor(void* ptr){	

	UA_Server* server = ptr;	
	int utime = SLEEP_TIME_MILLIS*15000;

	while (running == 1){
					
		// Update the counter 
		numberOfParts += 1;
		printf("\nCounter updated: %i parts\n", numberOfParts);

		// Update the OPC-UA node
		UA_Variant value;
		UA_Int32 myInteger = (UA_Int32) numberOfParts;
		UA_Variant_setScalarCopy(&value, &myInteger, &UA_TYPES[UA_TYPES_INT32]);
		UA_Server_writeValue(server, UA_NODEID_NUMERIC(1,COUNTER_NODE_ID), value);					

		// "Sampling time" of the "sensor"
		usleep(utime);
	}
}



int main(void) {   

	int ret;
	pthread_t threadSensor;

    signal(SIGINT, stopHandler);
    signal(SIGTERM, stopHandler);    

	// Create a new server with default configuration
	UA_Server* server = UA_Server_new(); 
    UA_ServerConfig_setDefault(UA_Server_getConfig(server));

    // Add the variable from the fake sensor
    addCounterSensorVariable(server); 

    // Launch the thread. The OPC-UA server is passed as parameter as the 
    // value of the node needs to be updated.
	if(pthread_create( &threadSensor, NULL, mainSensor, server)) {
		fprintf(stderr,"Error - pthread_create(): %d\n",ret);
		exit(EXIT_FAILURE);
	}

    /* This line runs the server in a loop while the running variable is true. 
	   It's important that initializations and other things done in our 
	   code are before this function call. */   
    UA_StatusCode retval = UA_Server_run(server, &running);    
    
	// When the server stops running we free the resources
    UA_Server_delete(server);


    return (int)retval;
}