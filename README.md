# ros2-opcua

## Description
The goal of this repository is to provide open62541 based OPC UA pubsub nodes that can be implemented in ROS 2.0 systems as ROS2.0/OPC-UA hybrid nodes to enable intercommunciation between the ROS 2.0 system and other automation equipment.

Essentially utilizing OPC UA as industry 4.0 communication between the robot programmed and controlled with ROS 2.0 and the rest of the automation equipment. 

Details of deployment in ROS 2.0 systems will be provided. 

At the present time the repository is in a developmental stage where the publisher and subscribers only can be run from the src subdirectory due to some relational #include statements.
(**IN PROGRESS:** Restructuring to simplify installation and removal of relative paths)

## Contents
This repository contains two OPC UA Pubsub implementations based on open62541.

- OPC UA Standalone: Easy to deploy standalone OPC UA Publisher Subsriber nodes that should be deployable on any Linux system. This section includes a set of demo nodes that will serve as a minimalistic example of use and a extended version of both nodes with more data fields, features etc.

- ROS2 Hybdrid Nodes: "hybrid" OPCUAPublisher/ROS2Subscriber and  OPCUA/Subscriber/ROS2Publisher nodes that can be included easily into your ROS2 workspace as a new package to accommodate for Industry 4.0 communication to your ROS2 controlled robot. 


## Development status / roadmap:  

    Stage 1   - OPC UA client/server local communication.                       DONE  

    Stage 1.5 - OPC UA client/server TCP based over ethernet (desktop<->Rpi3)   DONE

    Stage 2   - OPC UA pub/sub local                                            DONE		

    Stage 2.5 - OPC UA pub/sub UDP based over ethernet (desktop<->Rpi3)         DONE

    Stage 3   - open62541 integration in ROS 2.0 system                         DONE

    Stage 4   - OPC-UA-Publisher/ROS2.0-subscriber hybrid node                  DONE

    Stage 5   - OPC-UA-Subscriber/ROS2.0-publisher hybrid node                  PENDING

    Stage 6   - Code clean-up, improve modularity, improve extensibility,       
                informative in-line descriptions and comments

    Stage 7   - Introduce node hierarchy to OPC UA publisher to present
                component structure/hierarchy.                

    Stage 8   - Introduce a web interface to a component. Web interface
                should be launched from a seperate c-file with web service
                callable callbacks providing information from the OPC UA 
                information model.

    Stage 9   - Introduce encryption capabilities from open62541.
                demand login info to st up a pubsub-connection.           

    Stage 10   - Introduce open62541 TSN-extension, develop real-time
                pub/sub code to be runned with linux RT-patched linux system.



 

## Development notes: 

The open62541 pub/sub documentation is not finished, but there are several tutorials and some example code. There is high level APIs available for clients, servers and publishers but the subscriber API is currently not finished (07-08-2019).
