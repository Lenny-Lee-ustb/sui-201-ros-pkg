#!/usr/bin/env python3

import rospy
import serial
import math
from geometry_msgs.msg import PointStamped   
from std_msgs.msg import Float64      

def DueData(inputdata):  
    CheckSum = 0
    for i in range(0,22):
        CheckSum += inputdata[i] # Add up data.
    if inputdata[22] == (CheckSum & 0xff): # Stop if checksum is wrong.
        meaVoltage = int.from_bytes(inputdata[6:10], "big", signed=True) / 1000.0 # V
        meaCurrent = int.from_bytes(inputdata[10:14], "big", signed=True) / 1000.0 # A
        meaPower   = int.from_bytes(inputdata[14:18], "big", signed=True) /1000.0 # W
        meaPowerUsage = int.from_bytes(inputdata[18:22], "big", signed=True) /10000.0 # Wh
        pub_Voltage(meaVoltage)
        pub_Power(meaCurrent,meaPower,meaPowerUsage)


def pub_Voltage(meaVoltage):
    pub_vol = Float64()
    pub_vol.data = meaVoltage
    pubVoltage.publish(pub_vol)


def pub_Power(meaCurrent,meaPower,meaPowerUsage):
    pub_measure = PointStamped()
    pub_measure.header.frame_id = 'Unit(X: A, Y: W, Z: Wh)'
    pub_measure.header.stamp = rospy.Time.now()
    pub_measure.point.x = meaCurrent
    pub_measure.point.y = meaPower
    pub_measure.point.z = meaPowerUsage
    pubPower.publish(pub_measure)

 
if __name__=='__main__': 
    try:
        # set Node name 
        node_name = 'SUI-201-Power-Unit'

        # 
        port = '/dev/power'
        # port = '/dev/ttyUSB0'
        
        # set Baudrate
        baud = 115200

        rospy.init_node(node_name)
        rate = rospy.Rate(20) # max feedback speed
        
        ser = serial.Serial(port, baud, timeout=0.5)
        pubPower = rospy.Publisher('/power_comsume', PointStamped, queue_size=1, latch=True)
        pubVoltage = rospy.Publisher('/battery_voltage', Float64, queue_size=1, latch=True)

        # print(port,ID)
        print('Port is open: '+str(ser.is_open))

        powerQuery = b'\x55\x55\x01\x01\x00\x00\xAC' # Query Full state

        while not rospy.is_shutdown():
            ser.write(powerQuery)
            datahex = ser.read(23)
            DueData(datahex)
            rate.sleep()
    
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start Power Measure node.')