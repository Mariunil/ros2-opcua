# ros2-opcua

## Description
The goal of this repository is to provide open62541 based OPC UA pubsub nodes that can be implemented in ROS2 systems as ROS2/OPC UA hybrid nodes to enable intercommunciation between the ROS2 system and other automation equipment.


## Contents
This repository contains two OPC UA Pubsub implementations based on open62541.

- OPC UA Standalone: Easy to deploy standalone OPC UA Publisher Subsriber nodes that should be deployable on any Linux system. This section includes a set of demo nodes that will serve as a minimalistic example of use and a extended version of both nodes with more data fields, features etc.

- ROS2 Hybdrid Nodes: "hybrid" OPCUAPublisher/ROS2Subscriber and  OPCUA/Subscriber/ROS2Publisher nodes that can be included easily into your ROS2 workspace as a new package to accommodate for communication to your ROS2 controlled robot. 


## Development status / roadmap:  

    Stage 1   - OPC UA Client/Server local communication.                       DONE  

    Stage 1.5 - OPC UA Client/Server TCP based over ethernet (desktop<->Rpi3)   DONE

    Stage 2   - OPC UA Pub/Sub local                                            DONE		

    Stage 2.5 - OPC UA Pub/Sub UDP based over ethernet (desktop<->Rpi3)         DONE

    Stage 3   - open62541 integration in ROS2 system                            DONE

    Stage 4   - OPC UA Publisher/ROS2 Subscriber hybrid node                    DONE

    Stage 5   - OPC UA Subscriber/ROS2 Publisher hybrid node                    DONE

    Stage 5.5 - Hybrid node communication from desktop ROS2 system              DONE
                to Raspberry Pie deployed OPC UA Publisher and Subscriber
                (UDP transportlayer protocol over ethernet)

Development will stop here. Roadmap is mapped below.

    Stage 6   - Extend OPC UA standalone and hybrid nodes to include several
                data fields.     

    Stage 7   - Introduce node hierarchy to OPC UA publisher to present
                component structure/hierarchy and self-descriptive information   

    Stage 8   - Code clean-up: improve modularity, improve extensibility.               

    Stage 9   - Introduce a web interface to a component. Web interface
                should be launched from a seperate .c/.cpp file with web service
                callable callbacks providing information from the OPC UA 
                information model.

    Stage 10   - Introduce encryption capabilities from open62541.
                 Demand login info to set up a pubsub-connection.           

    Stage 11   - Introduce open62541 TSN-extension, develop real-time
                 pub/sub code to be runned with linux RT-patched linux system.



 

## Development notes: 

The open62541 pub/sub documentation is not finished, but there are several tutorials and some example code. There is high level APIs available for clients, servers and publishers but the subscriber API is currently not finished (07-08-2019).


## Installation 

A Installation guide is provided in installation_guide.txt