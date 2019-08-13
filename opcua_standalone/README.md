## opcua_standalone
Standalone demonstrational OPC UA Client, Server, Publisher and Susbcriber nodes.


### Usage
Launch the Server and the Client in their own terminals (same goes for publisher and subscriber). 


### Compilation
Compile using the -std=c99 flag. Link the open62541.c file in every compilation, it should be located at ros2-opcua/lib or wherever you've put it. It is originally found in open62541/build

	e.g. gcc -std=c99 -pthread ../lib/open62541.c cs_server_demo.c -o cs_server_demo