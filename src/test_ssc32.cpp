#include <iostream>
#include <string>
#include "lynxmotion_ssc32/ssc32.h"

int atoi( const char *a, int n )
{
	int num = 0;
	bool negate = false;
	int i = 0;

	if( n <= 0 )
		return -1;

	if( a[i] == '-' )
	{
		negate = true;
		i++;
	}

	while( i < n )
	{
		num *= 10;
		num += ( a[i] - '0' );
		i++;
	}

	if( negate )
		num *= -1;

	return num;
}

int main( int argc, char **argv )
{
	std::string port = "/dev/ttyUSB0";
	int baud = 115200;
	lynxmotion_ssc32::SSC32 ssc32_device;
	std::string version;

	if( argc < 2 )
	{
		std::cout << argv[0] << ": You must specify a command" << std::endl;
		std::cout << "Try `" << argv[0] << " --help' for more information." << std::endl;
		return 1;
	}

	if( strcmp( argv[1], "--help" ) == 0 || strcmp( argv[1], "-h" ) == 0 )
	{
		std::cout << "Usage: " << argv[0] << " COMMAND" << std::endl;
		std::cout << std::endl;
		std::cout << "Possible commands:" << std::endl;
		std::cout << "  -a                     Read inputs as analog." << std::endl
			  << "  -c                     Cancel the current command." << std::endl
			  << "  -d CH LVL              Send discrete output to channel CH. LVL must be 0 or 1." << std::endl
			  << "  -i                     Read inputs as digital." << std::endl
			  << "  -m CH PW [S] [T]       Move a servo on channel CH to the given pulse width PW." << std::endl
			  << "  -o CH OFFSET           Offsets channel CH by OFFSET amount." << std::endl
			  << "  -s                     Checks if any servos are currently moving" << std::endl
			  << "  -v                     Get the software version." << std::endl;
	}
	else if( strcmp( argv[1], "-v" ) == 0 )
	{
		if( !ssc32_device.open_port( port.c_str( ), baud ) )
			return -1;

		std::cout << "Getting software version information..." << std::endl;

		version = ssc32_device.get_version( );

		std::cout << "SSC32 Software Version: " << version << std::endl;
	}
	else if( strcmp( argv[1], "-c" ) == 0 )
	{
		if( !ssc32_device.open_port( port.c_str( ), baud ) )
			return -1;

		std::cout << "Cancelling current command..." << std::endl;

		if( !ssc32_device.cancel_command( ) )
			return -1;

		std::cout << "Done" << std::endl;
			
	}
	else if( strcmp( argv[1], "-m" ) == 0 )
	{
		if( argc < 4 )
		{
			std::cout << "Usage: " << argv[0] << " -m CHANNEL PULSE_WIDTH [SPEED] [TIME]" << std::endl;
			return 1;
		}

		lynxmotion_ssc32::SSC32::ServoCommand cmd;
		cmd.ch = atoi( argv[2], strlen( argv[2] ) );
		cmd.pw = atoi( argv[3], strlen( argv[3] ) );

		int time = -1;

		if( argc > 4 )
			cmd.spd = atoi( argv[4], strlen( argv[4] ) );

		if( argc > 5 )
			time = atoi( argv[5], strlen( argv[5] ) );

		if( !ssc32_device.open_port( port.c_str( ), baud ) )
			return 1;

		std::cout << "Sending command to move servo " << cmd.ch << " to pulse width " << cmd.pw << std::endl;

		if( !ssc32_device.move_servo( cmd, time ) )
			return 1;
	}
	else if( strcmp( argv[1], "-o" ) == 0 || strcmp( argv[1], "--offset" ) == 0 )
	{
		if( argc != 4 )
		{
			std::cout << "Usage: " << argv[0] << " -o CHANNEL OFFSET" << std::endl;
			return 1;
		}

		int ch = atoi( argv[2], strlen( argv[2] ) );
		int offset = atoi( argv[3], strlen( argv[3] ) );

		if( !ssc32_device.open_port( port.c_str( ), baud ) )
			return 1;

		std::cout << "Sending command to offset servo " << ch << " by pulse width " << offset << std::endl;

		if( !ssc32_device.pulse_offset( ch, offset ) )
			return 1;
	}
	else if( strcmp( argv[1], "-s" ) == 0 || strcmp( argv[1], "--status" ) == 0 )
	{
		if( !ssc32_device.open_port( port.c_str( ), baud ) )
			return 1;

		if( ssc32_device.query_movement_status( ) )
			std::cout << "Servos are currently moving." << std::endl;
		else
			std::cout << "No servos are currently moving." << std::endl;
	}
	else if( strcmp( argv[1], "-d" ) == 0 )
	{
		if( argc != 4 )
		{
			std::cout << "Usage: " << argv[0] << " -d CHANNEL LEVEL" << std::endl;
			return 1;
		}

		int ch = atoi( argv[2], strlen( argv[2] ) );
		lynxmotion_ssc32::SSC32::LogicLevel level;

		if( strcmp( argv[3], "0" ) == 0 )
			level = lynxmotion_ssc32::SSC32::Low;
		else if( strcmp( argv[3], "1" ) == 0 )
			level = lynxmotion_ssc32::SSC32::High;
		else
		{
			std::cout << "Level must be 0 (0V) or 1 (5V)" << std::endl;
			return 1;
		}

		if( !ssc32_device.open_port( port.c_str( ), baud ) )
			return 1;

		if( !ssc32_device.discrete_output( ch, level ) )
			return 1;
	}
	else if( strcmp( argv[1], "-a" ) == 0 )
	{
		lynxmotion_ssc32::SSC32::Inputs inputs[4];
		float data[4];

		inputs[0] = lynxmotion_ssc32::SSC32::PinA;
		inputs[1] = lynxmotion_ssc32::SSC32::PinB;
		inputs[2] = lynxmotion_ssc32::SSC32::PinC;
		inputs[3] = lynxmotion_ssc32::SSC32::PinD;

		if( !ssc32_device.open_port( port.c_str( ), baud ) )
			return 1;

		if( !ssc32_device.read_analog_inputs( inputs, data, 4 ) )
			return 1;

		for( int i = 0; i < 4; i++ )
			std::cout << "Pin " << ( ( char )i + 'A' ) << ": " << data[i] << std::endl;
	}
	else if( strcmp( argv[1], "-i" ) == 0 )
	{
		lynxmotion_ssc32::SSC32::Inputs inputs[4];
		unsigned int data[4];

		inputs[0] = lynxmotion_ssc32::SSC32::PinA;
		inputs[1] = lynxmotion_ssc32::SSC32::PinB;
		inputs[2] = lynxmotion_ssc32::SSC32::PinC;
		inputs[3] = lynxmotion_ssc32::SSC32::PinD;

		if( !ssc32_device.open_port( port.c_str( ), baud ) )
			return 1;

		if( !ssc32_device.read_digital_inputs( inputs, data, 4 ) )
			return 1;

		for( int i = 0; i < 4; i++ )
			std::cout << "Pin " << ( ( char )i + 'A' ) << ": " << data[i] << std::endl;
	}
	else
	{
		std::cout << argv[0] << ": invalid option -- '" << argv[1]+1 << "'" << std::endl;
		std::cout << "Try `" << argv[0] << " --help' for more information." << std::endl;
                return 1;
	}

	return 0;
}
