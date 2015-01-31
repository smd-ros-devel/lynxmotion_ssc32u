#include "lynxmotion_ssc32/ssc32_driver.h"
#include "math.h"
#include <algorithm>

namespace lynxmotion_ssc32
{

SSC32Driver::SSC32Driver( ros::NodeHandle &nh ) :
	nh( nh )
{
	for( int i = 0; i < 32; i++ )
		channels[i] = NULL;

	ros::NodeHandle priv_nh( "~" );

	priv_nh.param<std::string>( "port", port, "/dev/ttyUSB0" );
	priv_nh.param<int>( "baud", baud, 115200 );
	priv_nh.param<bool>( "publish_joint_states", publish_joint_states, true );

	// 180 degree servos seem to have a slightly greater range than 180 degrees.
	// This parameter allows for scaling the range for attempting to get
	// it to be closer to 180 degrees, or 90 degrees for 90 degree servos.
	priv_nh.param<double>( "range_scale", range_scale, 1.0 );

	if ( range_scale > 1 || range_scale <= 0 )
		range_scale = 1.0;
	scale = range_scale * 2000.0 / M_PI;

	// Parse joints ros param
	XmlRpc::XmlRpcValue joints_list;
	if( priv_nh.getParam( "joints", joints_list ) )
	{
		ROS_ASSERT( joints_list.getType( ) == XmlRpc::XmlRpcValue::TypeStruct );

		XmlRpcValueAccess joints_struct_access( joints_list );
		XmlRpc::XmlRpcValue::ValueStruct joints_struct = joints_struct_access.getValueStruct( );

		XmlRpc::XmlRpcValue::ValueStruct::iterator joints_it;

		for( joints_it = joints_struct.begin( ); joints_it != joints_struct.end( ); joints_it++ )
		{
			Joint *joint = new Joint;
			joint->name = static_cast<std::string>( joints_it->first );

			std::string joint_graph_name = "joints/" + joint->name + "/";

			priv_nh.param<int>( joint_graph_name + "channel", joint->properties.channel, 0 );

			// Channel must be between 0 and 31, inclusive
			ROS_ASSERT( joint->properties.channel >= 0 );
			ROS_ASSERT( joint->properties.channel <= 31 );

			priv_nh.param<double>( joint_graph_name + "max_angle", joint->properties.max_angle, M_PI_2 );
			priv_nh.param<double>( joint_graph_name + "min_angle", joint->properties.min_angle, -M_PI_2 );
			priv_nh.param<double>( joint_graph_name + "offset_angle", joint->properties.offset_angle, 0 );
			priv_nh.param<double>( joint_graph_name + "default_angle", joint->properties.default_angle, joint->properties.offset_angle );
			priv_nh.param<bool>( joint_graph_name + "initialize", joint->properties.initialize, true );
			priv_nh.param<bool>( joint_graph_name + "invert", joint->properties.invert, false );

			// Make sure no two joints have the same channel
			ROS_ASSERT( channels[joint->properties.channel] == NULL );

			// Make sure no two joints have the same name
			ROS_ASSERT( joints_map.find( joint->name ) == joints_map.end( ) );

			channels[joint->properties.channel] = joint;
			joints_map[joint->name] = joint;
		}
	}
	else
	{
		ROS_FATAL( "No joints were given" );
		ROS_BREAK( );
	}

	// Parse controllers ros param
	XmlRpc::XmlRpcValue controllers_list;
	if( priv_nh.getParam( "controllers", controllers_list ) )
	{
		ROS_ASSERT( controllers_list.getType( ) == XmlRpc::XmlRpcValue::TypeStruct );

		// Get the controllers ValueStruct
		XmlRpcValueAccess controllers_struct_access( controllers_list );
		XmlRpc::XmlRpcValue::ValueStruct controllers_struct = controllers_struct_access.getValueStruct( );

		XmlRpc::XmlRpcValue::ValueStruct::iterator controllers_it;

		// For each controller, parse its type and the joints associated with the controller
		for( controllers_it = controllers_struct.begin( ); controllers_it != controllers_struct.end( ); controllers_it++ )
		{
			Controller *controller = new Controller;
			controller->name = static_cast<std::string>( controllers_it->first );

			std::string controller_graph_name = "controllers/" + controller->name + "/";

			std::string controller_type;
			priv_nh.param<std::string>( controller_graph_name + "type", controller_type, "joint_controller" );

			// Validate the controller type
			if( controller_type == "joint_controller" )
				controller->type = ControllerTypes::JointController;
			else if( controller_type == "diff_drive_controller" )
				controller->type = ControllerTypes::DiffDriveController;
			else
			{
				ROS_FATAL( "Unknown controller type [%s] for controller [%s]",
					controller_type.c_str( ), controller->name.c_str( ) );
				delete controller;
				ROS_BREAK( );
			}

			priv_nh.param<bool>( controller_graph_name + "publish_joint_states", controller->publish_joint_states, true );

			// Get publish rate
			priv_nh.param<double>( controller_graph_name + "publish_rate", controller->publish_rate, 10.0 );
			if( controller->publish_rate <= 0.0 )
			{
				controller->expected_publish_time = 0.0;
				controller->publish_joint_states = false;
			}
			else
				controller->expected_publish_time = ( 1.0 / controller->publish_rate );

			// Make sure the controller has joints
			if( priv_nh.getParam( controller_graph_name + "joints", joints_list ) )
			{
				ROS_ASSERT( joints_list.getType( ) == XmlRpc::XmlRpcValue::TypeArray );

				// Get joints array
				XmlRpcValueAccess joints_array_access( joints_list );
				XmlRpc::XmlRpcValue::ValueArray joints_array = joints_array_access.getValueArray( );

				// Parse the joint names and verify the joint exists
				for( unsigned int i = 0; i < joints_array.size( ); i++ )
				{
					std::string joint_name = static_cast<std::string>( joints_array[i] );

					if( joints_map.find( joint_name ) != joints_map.end( ) )
					{
						controller->joints.push_back( joints_map[joint_name] );
					}
					else // joint doesn't exist
					{
						ROS_FATAL( "Joint [%s] for controller [%s] does not exist",
							joint_name.c_str( ), controller->name.c_str( ) );
						delete controller;
						ROS_BREAK( );
					}
				}
			}
			else // No joints were provided
			{
				ROS_FATAL( "Controller [%s] has no joints listed.", controller->name.c_str( ) );
				delete controller;
				ROS_BREAK( );
			}

			controllers.push_back( controller );
			controllers_map[controller->name] = controller;
		}
	}
	else
	{
		/*!
		 * \todo Instead of throwing an error if no controllers are give,
		 *       maybe put all joints into a 'global' controller that can
		 *       be controlled over the topic cmd_joint_traj.
		 */
		ROS_FATAL( "No controllers were given" );
		ROS_BREAK( );
	}

	relax_joints_service = nh.advertiseService( "relax_joints", &SSC32Driver::relaxJointsCallback, this );
}

SSC32Driver::~SSC32Driver( )
{
	stop( );

	for( int i = 0; i < 32; i++ )
		if( channels[i] )
			delete channels[i];

	while( !controllers.empty( ) )
	{
		Controller *controller = controllers.back( );
		controllers.pop_back( );
		delete controller;
	}

	for( std::map<std::string, std::queue<Command> >::iterator it = command_queues.begin( ); it != command_queues.end( ); it++ )
	{
		while( !command_queues[it->first].empty( ) )
		{
			Command command = command_queues[it->first].front( );
			command_queues[it->first].pop( );
			delete[] command.cmd;
		}
	}
}

bool SSC32Driver::init( )
{
	SSC32::ServoCommand *cmd;
	bool success = true;

	// Initialize each controller
	for( unsigned int i = 0; i < controllers.size( ); i++ )
	{
		ROS_DEBUG( "Initializing controller %s", controllers[i]->name.c_str( ) );

		// Only initialize the controller if it's a joint controller
		if( controllers[i]->type == ControllerTypes::JointController )
		{
			cmd = new SSC32::ServoCommand[controllers[i]->joints.size( )];

			for( unsigned int j = 0; j < controllers[i]->joints.size( ); j++ )
			{
				Joint *joint = controllers[i]->joints[j];

				if( joint->properties.initialize )
				{
					cmd[j].ch = joint->properties.channel;
					cmd[j].pw = ( unsigned int )( scale * ( joint->properties.default_angle - joint->properties.offset_angle ) + 1500 + 0.5 );


					ROS_INFO( "Initializing channel %d to pulse width %d", cmd[j].ch, cmd[j].pw );

					if( joint->properties.invert )
						cmd[j].pw = 3000 - cmd[j].pw;

					if( cmd[j].pw < 500 )
						cmd[j].pw = 500;
					else if( cmd[j].pw > 2500 )
						cmd[j].pw = 2500;
				}
			}

			// Send command
			if( !ssc32_dev.move_servo( cmd, controllers[i]->joints.size( ) ) )
			{
				ROS_ERROR( "Failed initializing controller %s", controllers[i]->name.c_str( ) );
				success = false;
			}

			delete[] cmd;
		}
	}

	return success;
}

bool SSC32Driver::relaxJoints( )
{
	ROS_INFO( "Relaxing joints" );

	if( ssc32_dev.is_connected( ) )
	{
		for( unsigned int i = 0; i < 32; i++ )
		{
			if( channels[i] != NULL )
				ssc32_dev.discrete_output( i, SSC32::Low );
		}
	}

	return true;
}

bool SSC32Driver::relaxJointsCallback( std_srvs::Empty::Request& request, std_srvs::Empty::Response& response )
{
	return relaxJoints( );
}

bool SSC32Driver::spin( )
{
	bool result = true;

	if( start( ) && init( ) )
	{
		ROS_INFO( "Spinning..." );

		while( ros::ok( ) )
		{
			update( );

			ros::spinOnce( );
		}
	}
	else
	{
		ROS_ERROR( "Failed to start" );
		result = false;
	}

	stop( );

	return result;
}

bool SSC32Driver::start( )
{
	ROS_INFO( "Starting SSC32Driver" );

	// Start device
	if( !ssc32_dev.open_port( port.c_str( ), baud ) )
		return false;

	std::string version = ssc32_dev.get_version( );
	if( version.empty( ) )
	{
		ROS_ERROR( "Unable to get software version" );
		ROS_INFO( "Verify the baud rate is set to the correct value" );
		return false;
	}

	ROS_INFO( "SSC32 Software Version: %s", version.c_str( ) );

	// Subscribe and advertise for every controller
	for( unsigned int i = 0; i < controllers.size( ); i++ )
	{
		joint_state_pubs_map[controllers[i]->name] = nh.advertise<sensor_msgs::JointState>( controllers[i]->name + "/joint_states", 1 );
		joint_subs.push_back( nh.subscribe( controllers[i]->name + "/command", 1, &SSC32Driver::jointCallback, this ) );
	}

	return true;
}

void SSC32Driver::stop( )
{
	ROS_INFO( "Stopping SSC32Driver" );

	relaxJoints( );

	nh.shutdown( );

	joint_state_pubs_map.clear( );

	joint_subs.clear( );

	ssc32_dev.close_port( );
}

void SSC32Driver::update( )
{
	current_time = ros::Time::now( );

	if( publish_joint_states )
	{
		publishJointStates( );
	}

	for( std::map<std::string, std::queue<Command> >::iterator it = command_queues.begin( ); it != command_queues.end( ); it++ )
		execute_command( it->first );

	last_time = current_time;
}

void SSC32Driver::publishJointStates( )
{
	for( unsigned int i = 0; i < controllers.size( ); i++ )
	{
		if( controllers[i]->publish_joint_states &&
			fabs( ( controllers[i]->last_publish_time - current_time ).toSec( ) ) >= controllers[i]->expected_publish_time)
		{
			sensor_msgs::JointState joints;
			joints.header.stamp = current_time;

			for( unsigned int j = 0; j < controllers[i]->joints.size( ); j++ )
			{
				joints.name.push_back( controllers[i]->joints[j]->name );

				int pw = ssc32_dev.query_pulse_width( controllers[i]->joints[j]->properties.channel );

				if( controllers[i]->joints[j]->properties.invert )
					pw = 3000 - pw;

				double angle = ( ( double ) pw - 1500.0 ) / scale + controllers[i]->joints[j]->properties.offset_angle;

				joints.position.push_back( angle );
			}

			joint_state_pubs_map[controllers[i]->name].publish( joints );

			controllers[i]->last_publish_time = current_time;
		}
	}
}

void SSC32Driver::jointCallback( const ros::MessageEvent<trajectory_msgs::JointTrajectory const>& event )
{
	ros::M_string connection_header = event.getConnectionHeader( );
	const trajectory_msgs::JointTrajectoryConstPtr &msg = event.getMessage( );

	std::string topic = connection_header["topic"];

	if( topic.empty( ) )
	{
		ROS_ERROR( "The connection header topic is empty" );
		return;
	}

	// Remove beginning '/'
	if( topic[0] == '/')
		topic.erase( 0, 1 );

	// Extract the controller name from the topic
	std::string::iterator it = find( topic.begin( ), topic.end( ), '/' );
	if( it != topic.end( ) )
		topic.erase( it, topic.end( ) );

	// Validate the controller name
	if( controllers_map.find( topic ) == controllers_map.end() )
	{
		ROS_ERROR( "[%s] is not a valid controller name.", topic.c_str( ) );
		return;
	}

	int num_joints = controllers_map[topic]->joints.size( );

	ros::Duration prev_time_from_start = ros::Duration( 0 );

	for( unsigned int i = 0; i < msg->points.size(); i++ )
	{
		SSC32::ServoCommand *cmd = new SSC32::ServoCommand[num_joints];
		bool invalid = false;

		for( unsigned int j = 0; j < msg->joint_names.size( ) && !invalid; j++ )
		{
			if( joints_map.find( msg->joint_names[j] ) != joints_map.end( ) )
			{
				Joint *joint = joints_map[msg->joint_names[j]];

				double angle = msg->points[i].positions[j];

				// Validate the commanded position (angle)
				if( angle >= joint->properties.min_angle && angle <= joint->properties.max_angle )
				{
					cmd[j].ch = joint->properties.channel;
					cmd[j].pw = ( unsigned int )( scale * ( angle - joint->properties.offset_angle ) + 1500 + 0.5 );
					if( joint->properties.invert )
						cmd[j].pw = 3000 - cmd[j].pw;
					if( cmd[j].pw < 500 )
						cmd[j].pw = 500;
					else if( cmd[j].pw > 2500 )
						cmd[j].pw = 2500;

					if( msg->points[i].velocities.size( ) >= j && msg->points[i].velocities[j] > 0 )
						cmd[j].spd = scale * msg->points[i].velocities[j];
				}
				else // invalid angle given
				{
					invalid = true;
					ROS_ERROR( "The given position [%f] for joint [%s] is invalid", angle, joint->name.c_str( ) );
				}
			}
			else
			{
				invalid = true;
				ROS_ERROR( "Joint [%s] does not exist", msg->joint_names[i].c_str( ) );
			}
		}

		// Queue the command for execution
		if(!invalid)
		{
			Command command;
			command.cmd = cmd;
			command.num_joints = num_joints;
			command.start_time = current_time + prev_time_from_start;
			command.duration = msg->points[i].time_from_start - prev_time_from_start;

			// Queue the command on the controller's command queue
			command_queues[topic].push( command );
		}
		else
			delete[] cmd;

		prev_time_from_start = msg->points[i].time_from_start;
	}
}

void SSC32Driver::execute_command( std::string controller )
{
	if( !command_queues[controller].empty( ) )
	{
		Command command = command_queues[controller].front( );

		if( command.start_time <= current_time )
		{
			if( !ssc32_dev.move_servo( command.cmd, command.num_joints, ( int )( command.duration.toSec() * 1000 + 0.5 ) ) )
				ROS_ERROR( "Failed sending joint commands to controller" );

			command_queues[controller].pop( );
			delete[] command.cmd;
		}
	}
}

}
