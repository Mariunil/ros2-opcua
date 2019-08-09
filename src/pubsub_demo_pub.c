/*

PUB/SUB DEMO EXAMPLE: Publisher

The PubSub publish demo demonstrate the simplest way to publish
informations from the information model over UDP multicast using the UADP
encoding.

The publisher uses high level Publisher-API

*/

#include <open62541/plugin/log_stdout.h>
#include <open62541/plugin/pubsub_ethernet.h>
#include <open62541/plugin/pubsub_udp.h> 
#include <open62541/server.h>
#include <open62541/server_config_default.h>
#include <signal.h>

static UA_NodeId folderId;
static UA_NodeId ds1Int32Id;
static UA_Int32 ds1Int32Val = 24;
#define Publisher_ID 2042
// Define the sampling time for the sensor
#define SLEEP_TIME_MILLIS 50
// Define the ID of the node externally as it will be needed inside the thread
#define COUNTER_NODE_ID 20305

UA_NodeId connectionId, publishedDataSetId, writerGroupId;

//To allow for ctrl-c triggered stop
UA_Boolean running = true;
static void stopHandler(int sign) {
  UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "received ctrl-c");
  running = false;
}

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

    connectionConfig.publisherId.numeric = Publisher_ID;

    /* Add the connection to the current PubSub configuration. */
    UA_Server_addPubSubConnection(server, &connectionConfig, &connectionId);
}



/* The PublishedDataSet (PDS) and PubSubConnection are the toplevel entities and
   can exist alone. The PDS contains the collection of the published fields. All
   other PubSub elements are directly or indirectly linked with the PDS or
   connection. */
static void addPublishedDataSet(UA_Server* server) {

    /* The PublishedDataSetConfig contains all necessary public
       informations for the creation of a new PublishedDataSet */

    /* Create new PublishedDataSet based on the PublishedDataSetConfig. */
    UA_PublishedDataSetConfig publishedDataSetConfig;
    memset(&publishedDataSetConfig, 0, sizeof(UA_PublishedDataSetConfig));
    publishedDataSetConfig.publishedDataSetType = UA_PUBSUB_DATASET_PUBLISHEDITEMS;
    publishedDataSetConfig.name = UA_STRING("TempSensor");

    UA_Server_addPublishedDataSet(server, &publishedDataSetConfig, &publishedDataSetId);
}




/* The DataSetField is part of the PublishedDataSet and describes exactly one 
   published field. The fields are whats being published */
static void addDataSetField(UA_Server* server) {

  /*-----------------den gamle---------------------------------------------
    /* Objects are used to represent systems, system components, 
     real-world objects and software objects. */
  

    UA_ObjectAttributes oAttr = UA_ObjectAttributes_default;
    oAttr.displayName = UA_LOCALIZEDTEXT("en-US", "Publisher 1");
    UA_Server_addObjectNode(server, UA_NODEID_NULL,
      UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
      UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
      UA_QUALIFIEDNAME(1, "Publisher 1"), 
      UA_NODEID_NUMERIC(0, UA_NS0ID_BASEOBJECTTYPE), oAttr, NULL, &folderId); 
    
    UA_NodeId_init(&ds1Int32Id);
    UA_VariableAttributes int32Attr = UA_VariableAttributes_default;
    int32Attr.valueRank = -1;
    UA_NodeId_copy(&UA_TYPES[UA_TYPES_INT32].typeId, &int32Attr.dataType);
    int32Attr.accessLevel = UA_ACCESSLEVELMASK_READ ^ UA_ACCESSLEVELMASK_WRITE;
    UA_Variant_setScalar(&int32Attr.value, &ds1Int32Val, &UA_TYPES[UA_TYPES_INT32]);
    int32Attr.displayName = UA_LOCALIZEDTEXT("en-US", "Int32");
    UA_Server_addVariableNode(server, UA_NODEID_STRING(1, "Publisher1.Int32"), folderId,
        UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
        UA_QUALIFIEDNAME(1, "Int32"),
        UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE), int32Attr, NULL, &ds1Int32Id);
    
    if (!UA_NodeId_equal(&publishedDataSetId, &UA_NODEID_NULL)){
        UA_DataSetFieldConfig int32Config;
        memset(&int32Config, 0, sizeof(UA_DataSetFieldConfig));
        int32Config.field.variable.fieldNameAlias = UA_STRING("Int32");
        int32Config.field.variable.promotedField = false;
        int32Config.field.variable.publishParameters.publishedVariable = ds1Int32Id;
        int32Config.field.variable.publishParameters.attributeId = UA_ATTRIBUTEID_VALUE;

        UA_NodeId f1;
        UA_Server_addDataSetField(server, publishedDataSetId, &int32Config, &f1);
    }

   /* 
   --------------------------------------------------------------

    //        Fugnerer toveis
    //-----------------------------------//
    /Here we are setting a specific ID for the node but the library
    //can do it if we don't specify it 
    UA_NodeId counterNodeId = UA_NODEID_NUMERIC(1, COUNTER_NODE_ID);  
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

    // Add a field to the previous created PublishedDataSet 
    UA_NodeId dataSetFieldId;
    UA_DataSetFieldConfig dataSetFieldConfig;
    memset(&dataSetFieldConfig, 0, sizeof(UA_DataSetFieldConfig));
    dataSetFieldConfig.dataSetFieldType = UA_PUBSUB_DATASETFIELD_VARIABLE;
    dataSetFieldConfig.field.variable.fieldNameAlias = UA_STRING("Server localtime");
    dataSetFieldConfig.field.variable.promotedField = UA_FALSE;
    dataSetFieldConfig.field.variable.publishParameters.publishedVariable =
    counterNodeId; //noden laget ovenfor etter cs_demo_server oppskrift
    dataSetFieldConfig.field.variable.publishParameters.attributeId = UA_ATTRIBUTEID_VALUE;
    UA_Server_addDataSetField(server, publishedDataSetId,
                              &dataSetFieldConfig, &dataSetFieldId);

    //-------------------------///
    */
}


/* The WriterGroup (WG) is part of the connection and contains the primary 
   configuration parameters for the message creation. */
static void addWriterGroup(UA_Server* server) {

    /* Now we create a new WriterGroupConfig and add the group to the existing
     PubSubConnection. */

    /* Create a new WriterGroup and configure parameters like the publish interval. */
    UA_WriterGroupConfig writerGroupConfig;
    memset(&writerGroupConfig, 0, sizeof(UA_WriterGroupConfig));
    writerGroupConfig.name = UA_STRING("WriterGroup");
    writerGroupConfig.publishingInterval = 100;
    writerGroupConfig.enabled = UA_FALSE;
    writerGroupConfig.writerGroupId = 100;
    writerGroupConfig.encodingMimeType = UA_PUBSUB_ENCODING_UADP;
    // writerGroupConfig.maxEncapsulatedDataSetMessageCount = 3; iop has it

    /* The configuration flags for the messages are encapsulated inside the
       message- and transport settings extension objects. These extension
       objects are defined by the standard. e.g. UadpWriterGroupMessageDataType */

    /* Add the new WriterGroup to an existing Connection. */
    UA_Server_addWriterGroup(server, connectionId, &writerGroupConfig, 
                                                      &writerGroupId);
}



/*A DataSetWriter (DSW) is the glue between the WG and the PDS. The DSW is linked to 
  exactly one PDS and contains additional informations for the message generation.*/
static void addDataSetWriter(UA_Server* server) {

    /* Create a new Writer and connect it with an existing PublishedDataSet */
    UA_NodeId dataSetWriterIdent;
    UA_DataSetWriterConfig dataSetWriterConfig;

    memset(&dataSetWriterConfig, 0, sizeof(UA_DataSetWriterConfig));
    dataSetWriterConfig.name = UA_STRING("DataSetWriter");
    dataSetWriterConfig.dataSetWriterId = Publisher_ID;

    /* The creation of delta messages is configured in the following line. Value
     0 -> no delta messages are created. */
    dataSetWriterConfig.keyFrameCount = 10;

    UA_Server_addDataSetWriter(server, writerGroupId, publishedDataSetId,
     						  &dataSetWriterConfig, &dataSetWriterIdent);

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
    config->pubsubTransportLayers[1] = UA_PubSubTransportLayerEthernet();
    config->pubsubTransportLayersSize++;
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

    /*
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
    */

    return run(&transportProfile, &networkAddressUrl);
}