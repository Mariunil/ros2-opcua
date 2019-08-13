#include "opcua_sub.h"


//To allow for ctrl-c triggered stop
UA_Boolean running = true;
static void stopHandler(int sign) {
    UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "received ctrl-c"); 
    running = false;
}

/* should be replaced with high level API calls similar to opcua_pub */
static UA_StatusCode subscriberListen(UA_PubSubChannel* psc) {
   
    UA_ByteString buffer;
    UA_StatusCode retval = UA_ByteString_allocBuffer(&buffer, 512);

    if(retval != UA_STATUSCODE_GOOD) {
        UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_SERVER,
                     "Message buffer allocation failed!");
        return retval;
    }

    /* Receive the message. Blocks for 100ms */
    retval = psc->receive(psc, &buffer, NULL, 100);
    /* workaround, if retval isnt good stop and get next message */
    if(retval != UA_STATUSCODE_GOOD || buffer.length == 0) {
        UA_ByteString_clear(&buffer);
        return UA_STATUSCODE_GOOD;
    }

    /* Decode the message */
    /*UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND,
                "Message length: %lu", (unsigned long) buffer.length);*/
    UA_NetworkMessage networkMessage;
    memset(&networkMessage, 0, sizeof(UA_NetworkMessage));
    size_t currentPosition = 0;
    // message transferred from buffer to networkMessage
    UA_NetworkMessage_decodeBinary(&buffer, &currentPosition, &networkMessage);
    UA_ByteString_clear(&buffer);

    /* Is this the correct message type? */
    if(networkMessage.networkMessageType != UA_NETWORKMESSAGE_DATASET)
        goto cleanup;

    /* At least one DataSetMessage in the NetworkMessage? */
    if(networkMessage.payloadHeaderEnabled &&
       networkMessage.payloadHeader.dataSetPayloadHeader.count < 1)
        goto cleanup;

    /* Is this a KeyFrame-DataSetMessage? */
    //for(size_t j = 0; j < networkMessage.payloadHeader.dataSetPayloadHeader.count; j++) {
    
        UA_DataSetMessage *dsm = &networkMessage.payload.dataSetPayload.dataSetMessages[0];
        if(dsm->header.dataSetMessageType != UA_DATASETMESSAGE_DATAKEYFRAME)
            printf("\nif\n");
            //continue;
      
        /* Loop over the fields and print well-known content types */
        for(int i = 0; i < dsm->data.keyFrameData.fieldCount; i++) {
            
            const UA_DataType* currentType = dsm->data.keyFrameData.dataSetFields[i].value.type;
            
            if(currentType == &UA_TYPES[UA_TYPES_DOUBLE]) {
               UA_Double value = *(UA_Double*)dsm->data.keyFrameData.dataSetFields[i].value.data;
               globalVal = value; // value from subscription placed in global variable
               /*UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND,
                           "Message content: [TempRead] \t %f [C]", value);*/

            } 
           
        } 
    //}

    cleanup:
    UA_NetworkMessage_clear(&networkMessage);
    return retval;
}

int run() {

    //ctrl-c handler
    signal(SIGINT, stopHandler);
    signal(SIGTERM, stopHandler);

    //transportlayer to be used
    UA_PubSubTransportLayer udpLayer = UA_PubSubTransportLayerUDPMP();

    UA_PubSubConnectionConfig connectionConfig;
    memset(&connectionConfig, 0, sizeof(connectionConfig));
    connectionConfig.name = UA_STRING("UADP Connection 1");

    connectionConfig.transportProfileUri =
        UA_STRING("http://opcfoundation.org/UA-Profile/Transport/pubsub-udp-uadp");
    connectionConfig.enabled = UA_TRUE;

    UA_NetworkAddressUrlDataType networkAddressUrl =
        {UA_STRING_NULL , UA_STRING("opc.udp://224.0.0.22:4840/")};
    UA_Variant_setScalar(&connectionConfig.address, &networkAddressUrl,
                         &UA_TYPES[UA_TYPES_NETWORKADDRESSURLDATATYPE]);

    /* creating pubsubchannel in the udp layer */
    UA_PubSubChannel* psc = udpLayer.createPubSubChannel(&connectionConfig);
    psc->regist(psc, NULL, NULL);

    /* while running and and listening return code GOOD, keep listening */
    UA_StatusCode retval = UA_STATUSCODE_GOOD;
    while(running && retval == UA_STATUSCODE_GOOD)
        retval = subscriberListen(psc);

    /* close the pubsub channel */
    psc->close(psc);
        
    return 0;
}

int opcuaSubscriberRun(){

    run();

}
