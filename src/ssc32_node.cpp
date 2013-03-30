#include <ros/ros.h>
#include "lynxmotion_ssc32/ssc32_driver.h"

int main( int argc, char** argv )
{
	ros::init( argc, argv, "ssc32_node" );
	ros::NodeHandle nh;

	lynxmotion_ssc32::SSC32Driver ssc32( nh );

	ssc32.spin( );

	return 0;
}
