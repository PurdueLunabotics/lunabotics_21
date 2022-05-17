#include <ros.h>
#include <std_msgs/Int8.h>
#include "actuator_config.h"
#include "sensor_config.h"

#ifndef DEPOSITION_H
#define DEPOSITION_H

// enums for determining motor pin index
namespace deposition
{

	extern HallSensor dep_hall;
	// DEPOSITION FUNCTIONS
	void init();
	void run_deposition(const std_msgs::Int8 &command, ros::NodeHandle *nh);
}

#endif
