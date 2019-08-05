/*

PUB/SUB DEMO EXAMPLE: 

The PubSub publish example demonstrate the simplest way to publish
informations from the information model over UDP multicast using the UADP
encoding.

*/

#include <open62541/plugin/log_stdout.h>
#include <open62541/plugin/pubsub_ethernet.h>
#include <open62541/plugin/pubsub_udp.h>
#include <open62541/server.h>
#include <open62541/server_config_default.h>

#include <signal.h>

UA_NodeId connectionIdent, publishedDataSetIdent, writerGroupIdent;


static void addPubSubConnection(UA_Server* server, UA_String* transportProfile,
                                UA_NetworkAddressUrlDataType* networkAddressUrl){


    /* Create a new ConnectionConfig. The addPubSubConnection function takes the
       config and create a new connection. The Connection identifier is
       copied to the NodeId parameter. */
    UA_PubSubConnectionConfig connectionConfig;
    memset(&connectionConfig, 0, sizeof(connectionConfig));
    connectionConfig.name = UA_STRING("UADP Connection 1");
    connectionConfig.transportProfileUri = *transportProfile;
    connectionConfig.enabled = UA_TRUE;

    UA_Variant_setScalar(&connectionConfig.address, networkAddressUrl,
                         &UA_TYPES[UA_TYPES_NETWORKADDRESSURLDATATYPE]);

    connectionConfig.publisherId.numeric = UA_UInt32_random();

    /* Create a new concrete connection and add the connection
       to the current PubSub configuration. */
    UA_Server_addPubSubConnection(server, &connectionConfig, &connectionIdent);
}



/* The PublishedDataSet (PDS) and PubSubConnection are the toplevel entities and
   can exist alone. The PDS contains the collection of the published fields. All
   other PubSub elements are directly or indirectly linked with the PDS or
   connection. */
static void addPublishedDataSet(UA_Server* server) {

    /* The PublishedDataSetConfig contains all necessary public
       informations for the creation of a new PublishedDataSet */
    UA_PublishedDataSetConfig publishedDataSetConfig;
    memset(&publishedDataSetConfig, 0, sizeof(UA_PublishedDataSetConfig));
    publishedDataSetConfig.publishedDataSetType = UA_PUBSUB_DATASET_PUBLISHEDITEMS;
    publishedDataSetConfig.name = UA_STRING("Demo PDS");

    /* Create new PublishedDataSet based on the PublishedDataSetConfig. */
    UA_Server_addPublishedDataSet(server, &publishedDataSetConfig, &publishedDataSetIdent);
}




/* The DataSetField (DSF) is part of the PDS and describes exactly one 
   published field. */
static void addDataSetField(UA_Server* server) {

    /* Add a field to the previous created PublishedDataSet */
    UA_NodeId dataSetFieldIdent;
    UA_DataSetFieldConfig dataSetFieldConfig;
    memset(&dataSetFieldConfig, 0, sizeof(UA_DataSetFieldConfig));
    dataSetFieldConfig.dataSetFieldType = UA_PUBSUB_DATASETFIELD_VARIABLE;
    dataSetFieldConfig.field.variable.fieldNameAlias = UA_STRING("Server localtime");
    dataSetFieldConfig.field.variable.promotedField = UA_FALSE;
    dataSetFieldConfig.field.variable.publishParameters.publishedVariable =
    
    UA_NODEID_NUMERIC(0, UA_NS0ID_SERVER_SERVERSTATUS_CURRENTTIME);
    dataSetFieldConfig.field.variable.publishParameters.attributeId = UA_ATTRIBUTEID_VALUE;
    UA_Server_addDataSetField(server, publishedDataSetIdent,
                              &dataSetFieldConfig, &dataSetFieldIdent);
}


/*The WriterGroup (WG) is part of the connection and contains the primary 
  configuration parameters for the message creation.*/
static void addWriterGroup(UA_Server* server) {

    /* Now we create a new WriterGroupConfig and add the group to the existing
       PubSubConnection. */
	UA_WriterGroupConfig writerGroupConfig;
	memset(&writerGroupConfig, 0, sizeof(UA_WriterGroupConfig));
	writerGroupConfig.name = UA_STRING("Demo WriterGroup");
	writerGroupConfig.publishingInterval = 100;
	writerGroupConfig.enabled = UA_FALSE;
	writerGroupConfig.writerGroupId = 100;
	writerGroupConfig.encodingMimeType = UA_PUBSUB_ENCODING_UADP;

	/* The configuration flags for the messages are encapsulated inside the
	   message- and transport settings extension objects. These extension
	   objects are defined by the standard. e.g. UadpWriterGroupMessageDataType */
	UA_Server_addWriterGroup(server, connectionIdent, &writerGroupConfig, 
                                                      &writerGroupIdent);
}



/*A DataSetWriter (DSW) is the glue between the WG and the PDS. The DSW is linked to 
  exactly one PDS and contains additional informations for the message generation.*/
static void addDataSetWriter(UA_Server* server) {

    /* We need now a DataSetWriter within the WriterGroup. This means we must
       create a new DataSetWriterConfig and add call the addWriterGroup function. */
	UA_NodeId dataSetWriterIdent;
	UA_DataSetWriterConfig dataSetWriterConfig;

	memset(&dataSetWriterConfig, 0, sizeof(UA_DataSetWriterConfig));
	dataSetWriterConfig.name = UA_STRING("Demo DataSetWriter");
	dataSetWriterConfig.dataSetWriterId = 62541;
	dataSetWriterConfig.keyFrameCount = 10;

	UA_Server_addDataSetWriter(server, writerGroupIdent, publishedDataSetIdent,
	 							   &dataSetWriterConfig, &dataSetWriterIdent);

}


//To allow for ctrl-c triggered stop
UA_Boolean running = true;
static void stopHandler(int sign) {
	UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "received ctrl-c");
	running = false;
}



/*starts the publisher server*/
static int run(UA_String* transportProfile, 
			   UA_NetworkAddressUrlDataType* networkAddressUrl) {

	// ctrl-c handler
	signal(SIGINT, stopHandler);
	signal(SIGTERM, stopHandler);

	// Server established, default configuration
	UA_Server* server = UA_Server_new();
	UA_ServerConfig* config = UA_Server_getConfig(server);
	UA_ServerConfig_setDefault(config);

  /* Add the PubSubTransportLayer implementation to the server config.
     The PubSubTransportLayer is a factory to create new connections
     on runtime. The UA_PubSubTransportLayer is used for all kinds of
     concrete connections e.g. UDP, MQTT, AMQP...*/
	config->pubsubTransportLayers =
	(UA_PubSubTransportLayer*) UA_calloc(2, sizeof(UA_PubSubTransportLayer));

	if(!config->pubsubTransportLayers) {
		UA_Server_delete(server);
		return EXIT_FAILURE;
	}

  /* It is possible to use multiple PubSubTransportLayers on runtime. The correct factory
     is selected on runtime by the standard defined PubSub TransportProfileUri's. */
	config->pubsubTransportLayers[0] = UA_PubSubTransportLayerUDPMP();
	config->pubsubTransportLayersSize++;


#ifdef UA_ENABLE_PUBSUB_ETH_UADP
    //config->pubsubTransportLayers[1] = UA_PubSubTransportLayerEthernet();
    //config->pubsubTransportLayersSize++;
#endif


     /* Publisher API calls */
    addPubSubConnection(server, transportProfile, networkAddressUrl);
    addPublishedDataSet(server);
    addDataSetField(server);
    addWriterGroup(server);
    addDataSetWriter(server);

    // This line runs the server in a loop while the running variable is true.
    UA_StatusCode retval = UA_Server_run(server, &running);
	  // When the server stops running we free the resources
    UA_Server_delete(server);

    return retval == UA_STATUSCODE_GOOD ? EXIT_SUCCESS : EXIT_FAILURE;
}



static void usage(char *progname) {
    printf("usage: %s <uri> [device]\n", progname);
}


int main(int argc, char **argv) {

    /* The address and interface is part of the standard
       defined UA_NetworkAddressUrlDataType. */
    UA_String transportProfile =
        UA_STRING("http://opcfoundation.org/UA-Profile/Transport/pubsub-udp-uadp");
    UA_NetworkAddressUrlDataType networkAddressUrl =
        {UA_STRING_NULL , UA_STRING("opc.udp://224.0.0.22:4840/")};

    if (argc > 1) {
        if (strcmp(argv[1], "-h") == 0) {
            usage(argv[0]);
            return EXIT_SUCCESS;
        } else if (strncmp(argv[1], "opc.udp://", 10) == 0) {
            networkAddressUrl.url = UA_STRING(argv[1]);
        } else if (strncmp(argv[1], "opc.eth://", 10) == 0) {
            transportProfile =
                UA_STRING("http://opcfoundation.org/UA-Profile/Transport/pubsub-eth-uadp");
            if (argc < 3) {
                printf("Error: UADP/ETH needs an interface name\n");
                return EXIT_FAILURE;
            }
            networkAddressUrl.networkInterface = UA_STRING(argv[2]);
            networkAddressUrl.url = UA_STRING(argv[1]);
        } else {
            printf("Error: unknown URI\n");
            return EXIT_FAILURE;
        }
    }

    return run(&transportProfile, &networkAddressUrl);
}