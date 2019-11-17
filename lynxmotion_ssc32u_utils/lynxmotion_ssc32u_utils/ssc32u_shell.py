# BSD 3-Clause License
# 
# Copyright (c) 2019, Matt Richard
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import argparse
import serial

def parse_args():
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--port', dest='port', default='/dev/ttyUSB0',
        help='port the device is located at')
    parser.add_argument('--baud', dest='baud', type=int, default=9600,
        help='baud rate')

    return parser.parse_args()

def validate_baud(baud):
    valid_bauds = [9600, 38400, 115200]

    if not baud in valid_bauds:
        print('Invalid baud: ', baud)
        print('Baud must be 9600, 38400, or 115200. Falling back 9600.')
        baud = 9600

    return baud

def print_help():
    print()
    print('                        Command reference')
    print()
    print('  Name                        Format')
    print('  ----                        ------')
    print('  Single servo command        #<ch> P<pw> S<spd> T<time>')
    print('  Multiple servo command      #<ch> P<pw> S<spd>...#<ch> P<pw> S<spd> T<time>')
    print('  Servo position offset       #<ch> PO<offset value>... #<ch> PO<offset value>')
    print('  Discrete output             #<ch> <lvl>...#<ch> <lvl>')
    print('  Byte output                 #<bank>:<value>')
    print('  Query movement status       Q')
    print('  Query pulse width           QP <arg>')
    print('  Digital input               A B C D E F AL BL CL DL EL FL')
    print('  Analog input                VA VB VC VD VE VF VG VH')
    print('  Version                     VER')
    print()

def main(args=None):
    args = parse_args()
    baud = validate_baud(args.baud);

    print('Opening port', args.port, 'with baud rate', baud)
    ser = serial.Serial(args.port, baud, timeout=1)

    loop = True
    while loop:
        command = input('> ')

        if command == 'quit' or command == 'exit':
            loop = False
        elif command == 'help':
            print_help()
        else:
            command = command + '\r'

            ser.write(command.encode('utf-8'))
            result = ser.read(100).decode('utf-8')

            if len(result) > 0:
                print(result.decode('utf-8'))
    
    ser.close()

if __name__ == '__main__':
    main()