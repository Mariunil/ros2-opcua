#include "ros2Sub_opcuaPub_node.h"
#include "ros2_sub.h"

// clarifying that the contents of this header are C code
extern "C" {
	#include "opcua_pub.h"
}

// Initialization to zero
double globalVal = 0.0;


int main(){

	/* Spins out thread running ros2 subscriber, listening on the Int32 topic,
	   continuously updating the global variable through a callback */
	pthread_t thread1;
	pthread_create(&thread1, NULL, &ros2SubscriberRun, NULL);

	/* contains a internal loop, continuously publishing the value 
	   of the global variable */
	opcuaPublisherRun();

	return 0;

}