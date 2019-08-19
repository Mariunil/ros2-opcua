# ros2-opcua

## Description
The goal of this repository is to provide open62541 based OPC UA pubsub standalone nodes and nodes that can be implemented in ROS2 systems as ROS2/OPC UA hybrid nodes to enable intercommunciation between the ROS2 system and other automation equipment.


## Contents
This repository contains two OPC UA Pubsub implementations based on open62541.

- OPC UA Standalone: Easy to deploy standalone OPC UA Publisher Subsriber nodes that should be deployable on any Linux system. This section includes a set of demo nodes that will serve as a minimalistic example of use and a extended version of both nodes with more data fields, features etc.

- ROS2 Hybdrid Nodes: "hybrid" OPCUAPublisher/ROS2Subscriber and  OPCUA/Subscriber/ROS2Publisher nodes that can be included easily into your ROS2 workspace as a new package to accommodate for communication with your ROS2 controlled robot.


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

Development stopped. Roadmap is mapped below.

      
    Stage 6   - Impelemnt the semantic information model of OPC UA
                properly by modelling devices with a proper component 
                hierarchy with self-descriptive information. 
                                
    Stage 7   - Find a way to install a common vocabulary in conformance to
                I4.0 standardization. Rename and all names and IDs to
                conform to the vocabulary, both inside OPC UA's 
                information model and ROS 2 topic names.
    
    Stage 8   - By leveraging the vocabulary and the semantic information
                model of OPC UA , set up automatic translation of the
                "sensors" data modelled in ROS 2 and OPC UA.

    Stage 9   - Introduce a web interface for the sensor. Web interface
                should be launched from a seperate .c/.cpp file with web service
                callable callbacks providing information from the OPC UA 
                information model. 

    Stage 10  - Introduce encryption capabilities from open62541.
                Demand login info to set up a pubsub-connection.           

    Stage 11  - Introduce open62541 TSN-extension and develop real-time
                pub/sub code. Test real-time performance on a real-time
                capable connection and OS.


 

## Development notes: 

The open62541 pub/sub documentation is not finished, but there are several tutorials and some example code to be found at their github https://github.com/open62541/open62541/tree/master/examples/pubsub. 

There is high level APIs available for clients, servers and publishers but the subscriber API is currently not finished (07-08-2019). The standalone and ROS2 nodes utilize the highest abstraction API available at the time.


## Installation 

A complete installation guide is provided in installation_guide.txt


## Inspecting 

For inspection purposes UAExpert and Wireshark are recommended. UAExpert provides a client-side GUI for inspecting the nodes and values of the server or publisher providng data.

Wireshark is a useful tool for monitoring packet transfer.
