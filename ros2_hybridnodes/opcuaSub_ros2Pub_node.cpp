#include "opcuaSub_ros2Pub_node.h"
#include "ros2_pub.h"

extern "C" {
	#include "opcua_sub.h"
}

double globalVal = 0.0;


int main(){

	pthread_t thread1;
	pthread_create(&thread1, NULL, &ros2PublisherRun, NULL);

	opcuaSubscriberRun();


	return 0;
}