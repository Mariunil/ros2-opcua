# opcua


## Development status:  

    Stage 1 – OPC UA client/server TCP local communication. <span style="color:green"> DONE </span> 

    Stage 1.5 - OPC UA client/server over TCP ethernet (desktop<->Rpi3)  <span style="color:green"> DONE </span>

    Stage 2 – OPC UA pub/sub local UDPMP <span style="color:yellow"> DONE </span> 

    Stage 2.5 - OPC UA pub/sub over UDPMP ethernet (desktop<->Rpi3) 

    Stage 3 – OPC UA-publisher integration in ROS 2.0 system 

    Stage 4 – OPC UA-subscriber integration in ROS 2.0 system 

    Stage 4 – AAS extension through webservices 

 

Development notes: 

Installation guide and demo code is being made. The publisher and subscriber nodes should be deployable directly inside ROS 2.0 systems. The repository containing all the code and guides will be available on github with the open62541 library as a submodule. 

Pub/sub documentation is not finished, but there are several tutorials and some example code. There is high level APIs available for clients, servers and publishers but the subscriber API is currently not finished (05.08.2019). 
