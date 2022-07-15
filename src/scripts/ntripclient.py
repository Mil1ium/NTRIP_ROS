#!/usr/bin/python

import rospy
import datetime
import time
from httplib import HTTPConnection
from httplib import HTTPException
import socket
from base64 import b64encode
from threading import Thread
import serial
from ntrip_ros.msg import GnssPVTSolnMsg # Copy from ublox_driver

######################################### Parameters ########################################

## UART configuration
f9p_uart2_port = '/dev/ttyUSB0'
f9p_uart2_baud = 460800

## NTRIP configuration
_ntrip_server = 'rtk.ntrip.qxwz.com'
_ntrip_port = 8002
_ntrip_mountpoint = 'RTCM32_GGB'
_ntrip_username = 'username'
_ntrip_password = 'password'

#############################################################################################

# Send RTCM Message to UART2 of Ublox-F9P module
uart_port = f9p_uart2_port
uart_baud = f9p_uart2_baud
uart_isopened = False
try:
    uart2_f9p = serial.Serial(uart_port, uart_baud, timeout = 0)
    uart_isopened = True
except:
    rospy.loginfo("Open %s failed! Please make sure you entered right port and baudrate!" % (uart_port))

def calcultateCheckSum(stringToCheck):
        xsum_calc = 0
        for char in stringToCheck:
            xsum_calc = xsum_calc ^ ord(char)
        return "%02X" % xsum_calc

class ntripconnect(Thread):
    def __init__(self, ntc):
        super(ntripconnect, self).__init__()
        self.ntc = ntc
        self.connected = False
        self.connection = HTTPConnection(self.ntc.ntrip_server, self.ntc.ntrip_port, timeout = 5)
        self.stop = False

    def do_connect(self):
        self.connection = HTTPConnection(self.ntc.ntrip_server, self.ntc.ntrip_port, timeout = 5)
        usr_pwd = self.ntc.ntrip_username + ':' + self.ntc.ntrip_password
        headers = {
            'Ntrip-Version': 'Ntrip/1.0',
            'User-Agent': 'NTRIP ntrip_ros',
            'Connection': 'close',
            'Authorization': 'Basic ' + b64encode(usr_pwd)
        }
        try:
            self.connection.request('GET', '/'+self.ntc.ntrip_mountpoint, self.ntc.getGGA(), headers)
            self.connected = True
            print " ------ In connecting process: Connected ------ "
            return True
        except KeyboardInterrupt:
            print " ------ In connecting process: Keyboard interrupt ------ "
            self.connected = False
            self.stop = True
            return False
        except (HTTPException, socket.error) as e:
            print " ------ In connecting process: HTTP error ------ "
            print e
            return False
        

    def run(self):
        while not self.connected:
            try:
                self.connected = self.do_connect()
            except KeyboardInterrupt:
                print " Keyboard interrupt "
                self.connected = False
                self.stop = True
            time.sleep(0.5)

        response = self.connection.getresponse()       
        buf = ""

        while (not self.stop) and self.connected:
            if response.status != 200:
                rospy.logerr("HTTP status is not 200! Reconnecting... ")
                self.do_connect()
                response = self.connection.getresponse()
                continue

            try:
                data = response.read(1) # [1] Sync code : first byte should be '211' , that is 'D3' in hex
            except (HTTPException, socket.error) as e:
                print " Reconnecting..."
                if self.do_connect():
                    response = self.connection.getresponse()
                continue
            except KeyboardInterrupt:
                print "Keyboard interrupt!"
                self.stop = True
                break
            

            if data!=chr(211):
                rospy.logwarn("Header not in sync, continue!")
                continue
            # [2] Reversed code : 6bit , 000000
            # [3] Msg length : 10bit
            l1 = ord(response.read(1)) # Read 2nd byte , including 2 bits in Msg length
            l2 = ord(response.read(1)) # Read 3rd byte , including 8 bits remaining in Msg length
            pkt_len = ((l1&0x3)<<8)+l2 # Got msg length : 2bit + 8bit = 10bit 
    
            pkt = response.read(pkt_len) # [4] Data body : read from 4th byte
            parity = response.read(3) # [5] CRC : read last 3 bytes
            if len(pkt) != pkt_len:
                rospy.logwarn("Wrong package length, continue!")
                continue

            if self.ntc.pvt_valid:
                rospy.loginfo("Got %d bytes from %s" % (pkt_len, self.ntc.ntrip_server))
            else:
                rospy.loginfo(" -[Invalid GGA]- Got %d bytes from %s" % (pkt_len, self.ntc.ntrip_server))

            # RTCM data = Sync code + Reversed code + Msg length + Data body + CRC
            rtcm_data = data + chr(l1) + chr(l2) + pkt + parity
            if uart_isopened and self.ntc.pvt_valid:
                # Send to UART2 of Ublox-F9P
                try:
                    uart2_f9p.write(rtcm_data)
                except:
                    print "did not send!"
                    pass
            elif not uart_isopened:
                rospy.logwarn("Uart not opened! RTCM data won't be sent!")
            elif not self.ntc.pvt_valid:
                rospy.logwarn("Position not fixed! RTCM data won't be sent!")
            time.sleep(0.2)

        self.connection.close()
        self.connected = False
        print "End of the process."

class ntripclient:
    def __init__(self):
        rospy.init_node('ntripclient', anonymous=True)

        self.pvt_topic = "/ublox_driver/receiver_pvt" # Topic from ublox_driver

        self.ntrip_server = _ntrip_server
        self.ntrip_port = _ntrip_port
        self.ntrip_username = _ntrip_username
        self.ntrip_password = _ntrip_password
        self.ntrip_mountpoint = _ntrip_mountpoint 

        self.sub_pvt = rospy.Subscriber(self.pvt_topic, GnssPVTSolnMsg, self.pvt_callback)

        # Some init value
        self.pvt_lat = 34.0     # GGA - <2>
        self.pvt_lon = 108.0    # GGA - <4>
        self.pvt_alt = 430.0    # GGA - <9>
        self.pvt_status = 0     # GGA - <6>
        self.pvt_num_sv = 4     # GGA - <7>
        self.pvt_pdop = 2.0     # GGA - <8>
        self.pvt_geoid_sep = -27.9  # GGA - <10>

        # When fix state turns to 3D fix, pvt_valid = True
        self.pvt_valid = False

        self.connection = None
        self.connection = ntripconnect(self)
        self.connection.start()

    def run(self):
        rospy.spin()
        if self.connection is not None:
            self.connection.stop = True

    def getGGA(self):
        now = datetime.datetime.utcnow()
        ggaString = "GPGGA,%02d%02d%04.2f,%04.4f,N,%05.4f,E,1,%02d,%02.2f,%04.1f,M,-27.9,M,," % \
            (now.hour,now.minute,now.second,self.pvt_lat,self.pvt_lon,self.pvt_num_sv,self.pvt_pdop,self.pvt_alt)
        checksum = calcultateCheckSum(ggaString)
        ggaString = "$%s*%s\r\n" % (ggaString, checksum) # DO NOT forget to add \r\n
        return ggaString


    # Because ublox_driver doesn't support NMEA , so here I use PVT message to generate GPGGA message
    # For NTRIP, latitude, longitude and altitude is enough
    # But for precision, I think maybe it's better to directly get GGA string from Ublox receiver
    def pvt_callback(self, pvt_msg):
        # Wait for 3D fix
        if pvt_msg.fix_type >= 3:
            self.pvt_valid = True
        else:
            self.pvt_valid = False
            rospy.logwarn("Position not fix yet! GGA message won't update and maybe untrusted!")
        
        if self.pvt_valid:
            self.pvt_lat = pvt_msg.latitude * 100   # x100 for GxGGA
            self.pvt_lon = pvt_msg.longitude * 100  # x100 for GxGGA
            self.pvt_alt = pvt_msg.altitude
            self.pvt_status = 1
            self.pvt_num_sv = pvt_msg.num_sv # Used sv of GPS,GLONASS,GALILEO,Beidou , not only GPS
            if self.pvt_num_sv > 12:    # max = 12 in GPGGA
                self.pvt_num_sv = 12    # can't get used GPS number, but it doesn't matter
            self.pvt_pdop = pvt_msg.p_dop


if __name__ == '__main__':
    c = ntripclient()
    c.run()

