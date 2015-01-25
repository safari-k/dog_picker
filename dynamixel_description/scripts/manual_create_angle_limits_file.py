#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Keith Kraft
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Arizona nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.



import sys
from optparse import OptionParser

from dynamixel_driver import dynamixel_io
from dynamixel_driver.dynamixel_const import *

def print_set_limits(f,values):
    ''' Takes a dictionary with all the motor values and does a formatted print.
    '''
    f.write('''        
dynamixel_AX12_%(motor_id)d_controller:
    controller:
        package: dynamixel_controllers
        module: joint_position_controller
        type: JointPositionController
    joint_name: dynamixel_AX12_%(motor_id)d_joint
    joint_speed: 2.0
    motor:
        id: %(motor_id)d
        init: %(init)d
        min: %(min)d
        max: %(max)d      
''' %values)
    print '''        
%(motor_id)d : %(min)d %(init)d %(max)d     
''' %values

if __name__ == '__main__':
    usage_msg = 'Usage: %prog [options] IDs'
    desc_msg = 'Prints the current status of specified Dynamixel servo motors.'
    epi_msg = 'Example: %s --port=/dev/ttyUSB1 --baud=1000000 --init=0 --safe=16 1 2 3 4 5' % sys.argv[0]
    
    parser = OptionParser(usage=usage_msg, description=desc_msg, epilog=epi_msg)
    parser.add_option('-p', '--port', metavar='PORT', default='/dev/ttyUSB0',
                      help='motors of specified controllers are connected to PORT [default: %default]')
    parser.add_option('-b', '--baud', metavar='BAUD', type="int", default=1000000,
                      help='connection to serial port will be established at BAUD bps [default: %default]')
    parser.add_option('-s', '--safe', metavar='SAFE', type="int", default=16,
                      help='min and max will be established at SAFE delta to measurement [default: %default]')
    parser.add_option('-i', '--init', metavar='INIT', type="int", default=0,
                      help='initial setting will be [default: mean of min/max] or INIT')                       
    parser.add_option('-f', '--file', metavar='FILE', default='dynamixel_limits.yaml',
                      help='filename of results [default: %default]')   
                    
    (options, args) = parser.parse_args(sys.argv)
    
    if len(args) < 2:
        parser.print_help()
        exit(1)
        
    port = options.port
    baudrate = options.baud
    motor_ids = args[1:]
    
    f = open(options.file, 'w')
    try:
        dxl_io = dynamixel_io.DynamixelIO(port, baudrate)
    except dynamixel_io.SerialOpenError, soe:
        print 'ERROR:', soe
    else:
        print 'Pinging motors:'
        for motor_id in motor_ids:   
            motor_id = int(motor_id)
            print '%d ...' % motor_id,
            p = dxl_io.ping(motor_id)
            if p:
                dxl_io.set_torque_enabled(motor_id, False)
                print 'Rotate to a maximum'
                variable = raw_input('Press enter when finished')
                values = dxl_io.get_feedback(motor_id)
                position1 = values['position']
                print position1
                print 'Rotate to other maximum'
                variable = raw_input('Press enter when finished')
                values = dxl_io.get_feedback(motor_id)
                position2 = values['position']
                print position2
     
                #setup values dictionary with results           
                values = {}
                values['motor_id'] = motor_id
                if position1 > position2:
                    values['min'] = position2 + int(options.safe)
                    values['max'] = position1 - int(options.safe)              
                else:
                    values['min'] = position1 + int(options.safe)
                    values['max'] = position2 - int(options.safe)
                #either mean or a certain value
                if options.init==0:    
                    values['init'] = int((position2 + position1)/2)
                else:
                    values['init'] = int(options.init)
                print_set_limits(f,values)           
                
            else:
                print 'error no ping'


