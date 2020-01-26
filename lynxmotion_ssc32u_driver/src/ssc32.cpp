// BSD 3-Clause License
// 
// Copyright (c) 2019, Matt Richard
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "lynxmotion_ssc32u_driver/ssc32.hpp"

#include <stdio.h>
#include <stdarg.h>

#ifndef DEBUG
#define DEBUG 0
#endif

namespace lynxmotion_ssc32u_driver
{

//Constructor
SSC32::SSC32()
: fd(-1)
{
  unsigned int i;
  for (i = 0; i < SSC32::MAX_CHANNELS; i++) {
    first_instruction[i] = 0;
  }
}

//Destructor
SSC32::~SSC32()
{
  close_port();
}

bool SSC32::open_port(const char *port, int baud)
{
  struct termios options;

  close_port();

  switch (baud) {
    case 2400:
      baud = B2400;
      break;
    case 9600:
      baud = B9600;
      break;
    case 38400:
      baud = B38400;
      break;
    case 115200:
      baud = B115200;
      break;
    default:
      printf("ERROR: Invalid baud [%d] -- must be 2400, 9600, 38400, or 115200\n", baud);
      return false;
  }

  fd = open(port, O_RDWR | O_NOCTTY);

  if (fd < 0) {
    printf("ERROR: Unable to open device on port %s\n", port);
    return false;
  }

  if (fcntl(fd, F_SETFL, 0) < 0) {
    printf("ERROR: port [%s] is already locked\n", port);
    close_port();
    return false;
  }

  memset(&options, 0, sizeof(options));
  cfsetispeed(&options, baud);
  cfsetospeed(&options, baud);

  options.c_iflag = IGNBRK | IGNPAR;
  options.c_oflag = 0;
  options.c_cflag |= CREAD | CS8 | CLOCAL;
  options.c_lflag = 0;

  if (tcsetattr(fd, TCSANOW, &options) < 0) {
    printf("ERROR: setting termios options\n");
    close_port();
    return false;
  }

  // Make sure queues are empty
  tcflush(fd, TCIOFLUSH);

  printf("Successfully opened port %s\n", port);

  return true;
}

bool SSC32::is_connected()
{
  return (fd != -1);
}

void SSC32::close_port()
{
  unsigned int i;

  if (fd != -1) {
    printf("Closing port\n");

    close(fd);

    fd = -1;

    for (i = 0; i < SSC32::MAX_CHANNELS; i++) {
      first_instruction[i] = 0;
    }
  }
}

bool SSC32::send_message(const char *msg, int size)
{
  if (fd != -1) {
    tcflush(fd, TCIOFLUSH);

#if DEBUG
    printf("INFO: [send_message] Sending message: ");
    for (unsigned int i = 0; i < strlen(msg); i++) {
      if (msg[i] == '\r') {
        printf("<cr>");
      } else if (msg[i] == 27) {
        printf("<esc>");
      } else {
        printf("%c", msg[i]);
      }
    }
    printf("\n");
#endif

    if (write(fd, msg, size) < 0) {
      log("ERROR: [send_message] Failed to write to device\n");
      return false;
    }
  } else {
    log("ERROR: [send_message] Device is not open\n");
    return false;
  }

  return true;
}

unsigned int SSC32::recv_message(unsigned char *buf, unsigned int size)
{
  int bytes_read;
  unsigned int total_bytes = 0;

  while (total_bytes != size) {
    if ((bytes_read = read(fd, buf + total_bytes, 1)) < 0) {
      log("ERROR: [recv_message] Failed to read from device\n");
      return total_bytes;
    }

    total_bytes += bytes_read;
  }

  return total_bytes;
}

bool SSC32::move_servo(struct ServoCommand cmd, int time)
{
  return move_servo(&cmd, 1, time);
}

bool SSC32::move_servo(struct ServoCommand cmd[], unsigned int n, int time)
{
  char msg[1024] = { 0 };
  char temp[32];
  int time_flag;
  unsigned int i;
  bool result;

  time_flag = 0;

  if (n > SSC32::MAX_CHANNELS) {
    log("ERROR: [move_servo] Invalid number of channels [%u]\n", n);
    return false;
  }

  for (i = 0; i < n; i++) {
    if (cmd[i].ch > 31) {
      log("ERROR: [move_servo] Invalid channel [%u]\n", cmd[i].ch);
      return false;
    }

    if (cmd[i].pw < SSC32::MIN_PULSE_WIDTH || cmd[i].pw > SSC32::MAX_PULSE_WIDTH) {
      log("ERROR: [move_servo] Invalid pulse width [%u]\n", cmd[i].pw);
      return false;
    }


    sprintf(temp, "#%u P%u ", cmd[i].ch, cmd[i].pw);

    strcat(msg, temp);

    if (first_instruction[cmd[i].ch] != 0) {
      if (cmd[i].spd > 0) {
        sprintf(temp, "S%d ", cmd[i].spd);
        strcat(msg, temp);
      }
    } else { // this is the first instruction for this channel
      time_flag++;
    }
  }

  // If time_flag is 0, then this is not the first instruction
  // for any channels to move the servo
  if (time_flag == 0 && time > 0) {
    sprintf(temp, "T%d ", time);
    strcat(msg, temp);
  }

  strcat(msg, "\r");

  result = send_message(msg, strlen(msg));

  // If the command was success, then the channels commanded
  // are not on their first instuction anymore.
  if (result) {
    for (i = 0; i < n; i++) {
      first_instruction[cmd[i].ch] = 1;
    }
  }

  return result;
}

bool SSC32::cancel_command()
{
  char msg[4];
  sprintf(msg, "%c \r", 27);

  return send_message(msg, strlen(msg));
}

bool SSC32::pulse_offset(unsigned int ch, int value)
{
  return pulse_offset(&ch, &value, 1);
}

bool SSC32::pulse_offset(unsigned int ch[], int value[], unsigned int n)
{
  char msg[1024] = { 0 };
  char temp[12];
  unsigned int i;

  if (n > SSC32::MAX_CHANNELS) {
    log("ERROR: [pulse_offset] Invalid number of channels [%u]\n", n);
    return false;
  }

  for (i = 0; i < n; i++) {
    if (ch[i] > 31) {
      log("ERROR: [pulse_offset] Invalid channel [%u]\n", ch[i]);
      return false;
    }

    if (value[i] < -100 || value[i] > 100) {
      log("ERROR: [pulse_offset] Invalid offset value [%d]\n", value[i]);
      return false;
    }

    sprintf(temp, "#%u PO%d ", ch[i], value[i]);

    strcat(msg, temp);
  }

  strcat(msg, "\r");

  return send_message(msg, strlen(msg));
}

bool SSC32::discrete_output(unsigned int ch, LogicLevel lvl)
{
  return discrete_output(&ch, &lvl, 1);
}

bool SSC32::discrete_output(unsigned int ch[], LogicLevel lvl[], unsigned int n)
{
  char msg[1024] = { 0 };
  char temp[7];
  unsigned int i;

  if (n > SSC32::MAX_CHANNELS) {
    log("ERROR: [discrete_output] Invalid number of channels [%u]\n", n);
    return false;
  }

  for (i = 0; i < n; i++) {
    if (ch[i] > 31) {
      log("ERROR: [discrete_output] Invalid servo channel [%u]\n", ch[i]);
      return false;
    }

    sprintf(temp, "#%u %c ", ch[i], (lvl[i] == High) ? 'H' : 'L');

    strcat(msg, temp);
  }

  strcat(msg, "\r");

  return send_message(msg, strlen(msg));
}

bool SSC32::byte_output(unsigned int bank, unsigned int value)
{
  char msg[10];

  if (bank > 3) {
    log("ERROR: [byte_output] Invalid bank [%u]\n", bank);
    return false;
  }

  if (value > 255) {
    log("ERROR: [byte_output] Invalid value [%u]\n", value);
    return false;
  }

  sprintf(msg, "#%d:%d \r", bank, value);

  return send_message(msg, strlen(msg));
}

bool SSC32::query_movement_status()
{
  unsigned char buffer;
  //int bytes_read = 0;
  const char *msg = "Q \r";

  if (!send_message(msg, strlen(msg))) {
    log("ERROR: [query_movement_status] Failed to send message\n");
    return false;
  }

  // There is a delay of at least 50uS to 5mS, so sleep for 5ms.
  usleep(10000);

  // Continue reading from controller until a response is received
  if (recv_message(&buffer, 1) != 1) {
    log("ERROR: [query_movement_status] Failed to receive message\n");
    return false;
  }

  // Check response value
  if (buffer == '+') {
    return true;
  }

  return false;
}

int SSC32::query_pulse_width(unsigned int ch)
{
  unsigned char buffer;
  char msg[7];

  // Check if the servo channel is valid
  if (ch > 31) {
    log("ERROR: [query_pulse_width] Invalid servo channel [%u]\n", ch);
    return false;
  }

  sprintf(msg, "QP%d \r", ch);

  if (!send_message(msg, strlen(msg))) {
    log("ERROR: [query_pulse_width] Failed to send message\n");
    return false;
  }

  // Give time for the controller to respond
  usleep(1000);

  if (recv_message(&buffer, 1) != 1) {
    log("ERROR: [query_pulse_width] Failed to receive message\n");
    return false;
  }

  return (10 * (int)buffer);
}

bool SSC32::read_digital_inputs(Inputs inputs[], unsigned int outputs[], unsigned int n)
{
  unsigned char buffer[8];
  char msg[255] = { 0 };
  unsigned int i;

  // SSC-32U documentation states that only up to 8 values can be read at once
  if (n > 8) {
    log("WARNING: reading digital inputs -- n must not be greater than 8\n");
    n = 8;
  }

  for (i = 0; i < n; i++) {
    switch (inputs[i]) {
      case PinA:  strcat(msg, "A ");  break;
      case PinAL: strcat(msg, "AL "); break;
      case PinB:  strcat(msg, "B ");  break;
      case PinBL: strcat(msg, "BL "); break;
      case PinC:  strcat(msg, "C ");  break;
      case PinCL: strcat(msg, "CL "); break;
      case PinD:  strcat(msg, "D ");  break;
      case PinDL: strcat(msg, "DL "); break;
      case PinE:  strcat(msg, "E ");  break;
      case PinEL: strcat(msg, "EL "); break;
      case PinF:  strcat(msg, "F ");  break;
      case PinFL: strcat(msg, "FL "); break;
      default:
        log("WARNING: [read_digital_inputs] Unrecognized input value [%d]\n", inputs[i]);
        break;
    }
  }

  strcat(msg, "\r");

  if (!send_message(msg, strlen(msg))) {
    log("ERROR: [read_digital_inputs] Failed to send message\n");
    return false;
  }

  if (recv_message(buffer, n) != n) {
    log("ERROR: [read_digital_inputs] Failed to receive message\n");
    return false;
  }

  for (i = 0; i < n; i++) {
    outputs[i] = buffer[i] - '0';
  }

  return true;
}

bool SSC32::read_analog_inputs(Inputs inputs[], float outputs[], unsigned int n)
{
  unsigned char buffer[8];
  char msg[255] = { 0 };
  unsigned int i;

  if (n > 8) {
    log("WARNING: reading analog inputs -- n must not be greater than 8\n");
    n = 8;
  }

  for (i = 0; i < n; i++) {
    switch (inputs[i]) {
      case PinA: strcat(msg, "VA "); break;
      case PinB: strcat(msg, "VB "); break;
      case PinC: strcat(msg, "VC "); break;
      case PinD: strcat(msg, "VD "); break;
      case PinE: strcat(msg, "VE "); break;
      case PinF: strcat(msg, "VF "); break;
      case PinG: strcat(msg, "VG "); break;
      case PinH: strcat(msg, "VH "); break;
      default:
        log("WARNING: [read_analog_inputs] Unrecognized input value [%d]\n", inputs[i]);
        break;
    }
  }

  strcat(msg, "\r");

  if (!send_message(msg, strlen(msg))) {
    log("ERROR: [read_analog_inputs] Failed to send message\n");
    return false;
  }

  if (recv_message(buffer, n) != n) {
    log("ERROR: [read_analog_inputs] Failed to receive message\n");
    return false;
  }

  for (i = 0; i < n; i++) {
    outputs[i] = 5.0 * buffer[i] / 256.0;
  }

  return true;
}

std::string SSC32::get_version()
{
  char data[255];
  int bytes_read;
  int total_bytes;
  int i;
  std::string version;
  const char *msg = "VER\r";

  total_bytes = 0;

  if (!send_message(msg, strlen(msg))) {
    log("ERROR: [get_version] Failed to send message\n");
    return "error";
  }

  usleep(100000);

  log("INFO: [get_version] Reading response\n");

  while ((bytes_read = read(fd, data + total_bytes, 1)) > 0) {
    total_bytes += bytes_read;
  }

#if DEBUG
  printf("INFO: [get_version] Data: ");
  for (i = 0; i < total_bytes; i++) {
    if (data[i] == '\r') {
      printf("<cr>");
    } else if (data[i] == '\n') {
      printf("<nl>");
    } else {
      printf("%c", data[i]);
    }
  }
  printf("\n");
#endif

  if (bytes_read < 0) {
    log("ERROR: [get_version] Failed to read from device\n");
  } else if (total_bytes > 0) {
    log("Read %d bytes\n", total_bytes);

    if (data[total_bytes - 1] == '\r') {
      data[total_bytes - 1] = '\0';
    } else {
      log("WARNING: [get_version] Timeout while reading\n");

      data[total_bytes] = '\0';
    }

    i = total_bytes - 2;

    while (i >= 0 && data[i] != '\r') {
      i--;
    }

    version = data + i + 1;
  } else {
    log("WARNING: [get_version] Timeout while reading\n");
  }

  return version;
}

void SSC32::log(const char* format, ...)
{
#if DEBUG
  va_list args;
  va_start(args, format);
  vprintf(format, args);
  va_end(args);
#else
  // make the build tool happy
  format = format;
#endif
}

}  // namespace lynxmotion_ssc32u_driver
