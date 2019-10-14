#ifndef LYNX_MOTION_SSC32_SSC32_H
#define LYNX_MOTION_SSC32_SSC32_H

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <math.h>
#include <string>
#include <sstream>
#include <cstdio>

namespace lynxmotion_ssc32
{

class SSC32
{
	public:
		const static unsigned int MAX_PULSE_WIDTH =	2500;
		const static unsigned int CENTER_PULSE_WIDTH =	1500;
		const static unsigned int MIN_PULSE_WIDTH =	500;

		const static unsigned int MAX_CHANNELS = 32;

		struct ServoCommand
		{
		        unsigned int ch;
		        unsigned int pw;
		        int spd;

		        ServoCommand( ) :
		                ch( 0 ),
		                pw( SSC32::CENTER_PULSE_WIDTH ),
		                spd( -1 )
		        {
		        }
		};

		SSC32( );
		~SSC32( );
		bool open_port( const char *port, int baud );
		bool is_connected( );
		void close_port( );
		bool move_servo( struct ServoCommand cmd, int time = -1 );
		bool move_servo( struct ServoCommand cmd[], unsigned int n, int time = -1 );
		bool cancel_command( );
		bool pulse_offset( unsigned int ch, int value );
		bool pulse_offset( unsigned int ch[], int value[], unsigned int n );

		/*!
		 * \enum LogicLevel
		 * \brief Servo channel logic level.
		 */
		enum LogicLevel
		{
			Low,	/**< Low logic level (0V) */
			High	/**< High logic level (+5V) */
		};

		bool discrete_output( unsigned int ch, LogicLevel lvl );
		bool discrete_output( unsigned int ch[], LogicLevel lvl[], unsigned int n );

		/*!
		 * \brief
		 *
		 * \description
		 * This command allows 8 bits of binary data to be written at once. All pins of the bank are
		 * updated simultaneously. The banks will be updated within 20mS of receiving the message.
		 *
		 * \param bank 0 = Pins 0-7, 1 = Pins 8-15, 2 = Pins 16-23, 3 = Pins 24-31.
		 * \param value Decimal value to output to the selected bank (0-255). Bit 0 = LSB of bank.
		 *
		 * \returns True if the message was sent successfully; otherwise, false;
		 */
		bool byte_output( unsigned int bank, unsigned int value );

		/*!
		 * \brief Querys the SSC-32 if any servos are moving.
		 *
		 * \returns True if any servos are currently moving; otherwise, false.
		 */
		bool query_movement_status( );

		/*!
		 * \brief Queries the current pulse width of a servo.
		 *
		 * \param ch Servo to query.
		 *
		 * \returns The pulse width (500-2500) of the selected servo.
		 */
		int query_pulse_width( unsigned int ch );

		enum Inputs
		{
			PinA,
			PinAL,
			PinB,
			PinBL,
			PinC,
			PinCL,
			PinD,
			PinDL,
                        PinE,  // SSC-32U only
			PinEL, // SSC-32U only
			PinF,  // SSC-32U only
			PinFL, // SSC-32U only
			PinG,  // SSC-32U only
			PinH,  // SSC-32U only
		};

		bool read_digital_inputs( Inputs inputs[], unsigned int outputs[], unsigned int n );

		bool read_analog_inputs( Inputs inputs[], float outputs[], unsigned int n );

		std::string get_version( );

	private:
		/**
		 * \brief Sends a message to the SSC-32.
		 *
		 * \param msg Message to send to the servo controller.
		 * \param size Size of the message.
		 *
		 * \returns True if there were no errors while sending the message; otherwise, false;
		 */
		bool send_message( const char *msg, int size );

		unsigned int recv_message( unsigned char *buf, unsigned int size );

		int fd; // file descriptor for the serial port
		int first_instruction[32];
};

} //namespace

#endif
