# opcua

## Description
The goal of this repository is to provide open62541 based OPC UA pubsub nodes that can be implemented in ROS 2.0 systems as ROS2.0/OPC-UA hybrid nodes to enable intercommunciation between the ROS 2.0 system and other automation equipment.

Essentially utilizing OPC UA as industry 4.0 communication between the robot programmed and controlled with ROS 2.0 and the rest of the automation equipment. 

Details of deployment in ROS 2.0 pubsub systems will be provided. 

At the present time the repository is in a developmental stage where the publisher and subscribers only can be run from the src subdirectory due to some relational #include statements.


## Development status:  

    Stage 1   - OPC UA client/server local communication.                       DONE  

    Stage 1.5 - OPC UA client/server TCP based over ethernet (desktop<->Rpi3)   DONE

    Stage 2   - OPC UA pub/sub local                                            DONE		

    Stage 2.5 - OPC UA pub/sub UDP based over ethernet (desktop<->Rpi3)         DONE

    Stage 3   - open62541 integration in ROS 2.0 system                         PENDING

    Stage 4   - OPC-UA-Publisher/ROS2.0-subscriber hybrid node 

    Stage 5   - OPC-UA-Subscriber/ROS2.0-publisher hybrid node

    Stage 6   - Adding component structure to OPC UA publisher to enable 
                node-hierarchy

    Stage 7   - OPC UA subscriber with web extensions launching web based AAS
                with component information 

    Stage 8   - OPC UA subscriber/webAAS: adding oppurtunity to call callbacks through
                webservices in the web AAS  

 


 

## Development notes: 

Installation guide and demo code is being made. The publisher and subscriber nodes should be deployable directly inside ROS 2.0 systems. 

The open62541 pub/sub documentation is not finished, but there are several tutorials and some example code. There is high level APIs available for clients, servers and publishers but the subscriber API is currently not finished (07-08-2019).
