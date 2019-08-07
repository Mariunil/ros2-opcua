# opcua

#Description
This repository will provide OPC UA pubsub publisher and subcriber nodes that can be implemented in ROS 2.0 pubsub systems. Details of deployment in ROS 2.0 pubsub systems will come. At the present time the repository is in a developmental stage where the publisher and subscribers only can be run from the src subdirectory due to some relational #include statements.



## Development status:  

    Stage 1   - OPC UA client/server local communication.                       DONE  

    Stage 1.5 - OPC UA client/server over TCP ethernet (desktop<->Rpi3)         DONE

    Stage 2   - OPC UA pub/sub local                                            DONE		

    Stage 2.5 - OPC UA pub/sub over ethernet (desktop<->Rpi3)                   PENDING

    Stage 3   - OPC UA publisher integration in ROS 2.0 system 

    Stage 4   - OPC UA subscriber integration in ROS 2.0 system 

    Stage 5   - AAS extension through webservices 

 

## Development notes: 

Installation guide and demo code is being made. The publisher and subscriber nodes should be deployable directly inside ROS 2.0 systems. 

The open62541 pub/sub documentation is not finished, but there are several tutorials and some example code. There is high level APIs available for clients, servers and publishers but the subscriber API is currently not finished (07-08-2019).
