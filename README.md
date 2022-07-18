# NTRIP_ROS

First time to upload my code here. This is a beta version for testing.

This program helps you get RTCM messages from NTRIP caster and get RTK solution:
1) Recieve PVT topic from [ublox_driver](https://github.com/HKUST-Aerial-Robotics/ublox_driver);
2) Generate GPGGA string by PVT message;
3) Send GPGGA string to NTRIP server(NTRIP caster) by httplib.HTTPConnection, and get RTCM data from response;
4) Send recieved RTCM data to Ublox-F9P module via UART when the port is opened and single point position valid;
5) Reconnect if network failed.

# 1. Prerequisites
## 1.1 ROS
ROS melodic (tested)
## 1.2 Python and 3rdparty library
**Python 2.7** is required, NOT Python 3.

**httplib** (if not installed, run ```pip install``` to install this module).

## 1.3 Test information
I tested this program in Ubuntu 18.04 (with ROS melodic and Python 2.7), it seems to work well.

But in Ubuntu 20.04 (with ROS noetic, Python 3 and http.client), errors occurred.

I did a lot of research, I think the main reason may be that **http.client in python 3 DOES NOT support** `ICY 200 OK` **response header sent by NTRIP caster**, but the problem does not exist in httplib.HTTPConnection in Python 2.7 . 

# 2. Build this program
Clone the code to your workspace and:
```
catkin_make
```

# 3. Run with you F9P module
## 3.1 Install u-center
The version I'm using is v21.09 , you can test NTRIP username and password in ```Reciever-NTRIP Client```.

## 3.2 Configure PORT and RATE of your F9P module
Firstly you need to correctly configure UART port of your F9P module in ```Configure-PRT```, such as ```Baudrate``` and ```Protocol in/out```.

In my case, I use UART1 to send ```UBX-RXM-RAWX```, ```UBX-RXM-SFRBX``` and ```UBX-PVT``` message via USB-UART module to my laptop (and parsed by [ublox_driver](https://github.com/HKUST-Aerial-Robotics/ublox_driver), published as ROS topic), and send RTCM message to UART2 via USB-UART module to F9P, so ```UBX``` in ```Potocol out``` of UART1 is needed, and ```RTCM``` in ```Potocol in``` of UART2 is needed.

Message rate in ```Configure-RATE``` may also be set, for example 10Hz.

After configuration, you can see messages printed in ```Packet Console```.

## 3.3 Enable PVT message
Because I'm using [ublox_driver](https://github.com/HKUST-Aerial-Robotics/ublox_driver), I need to enable ```UBX-RXM-RAWX```, ```UBX-RXM-SFRBX``` and ```UBX-PVT``` message in ```u-center```.

Actually, **only** ```UBX-PVT``` **message is needed for RTK**. You can use other ublox driver to extract ```PVT``` message and publish as ROS topic so ```RAWX``` and ```SFRBX``` is not needed anymore. Or you could use ublox driver which supports NMEA, then you just need to enable NMEA-GxGGA and subscribe NMEA message and send it to NTRIP caster.

## 3.4 Run with your Ublox-F9P
1) Plug in your F9P's UART ports to your computer via USB-UART module, and check ports' name, like ```/dev/ttyUSB0```, then change the __port name and baudrate__ in ```ntripclient.py``` (F9P's UART2, for sending RTCM data to module) and ```driver_config.yaml``` (F9P's UART1, for recieve PVT message from module) of [ublox_driver](https://github.com/HKUST-Aerial-Robotics/ublox_driver), don't forget to obtain r/w permissions of the ports. 

2) Launch [ublox_driver](https://github.com/HKUST-Aerial-Robotics/ublox_driver), and ```rostopic echo``` topic ```/ublox_driver/receiver_pvt```, remember to ```source devel/setup.bash``` in your terminal in ublox_driver workspace.

3) Run roscore in a terminal:
```
roscore
```
4) In the other terminal, run this program in your workspace:
```
source devel/setup.bash
rosrun ntrip_ros ntripclient.py
```

If your network is fine, UART successfully opened and GNSS single point position is fixed, the recieved RTCM data is sent to UART2 of F9P module.

__Then, wait for__ ```carr_soln``` __of topic__ ```/ublox_driver/receiver_pvt``` __turning to__ ```2```__, which means RTK fixed solution__ (```0``` means no RTK, ```1``` means RTK float solution), __and enjoy it.__

__Note that RTK solution is obtained in topic__ ```/ublox_driver/receiver_pvt```.

## 3.5 Hardware configuration
In my case, I'm using 
1) Beitian BT-F9PK4 module;
2) Beitian BT-345 antenna;
3) Quectel EC20 4G module;
4) CH340 USB to UART module made by myself;
5) Lenovo laptop with i5-8265U.

# 4. Reference
## 4.1 Msg files
ROS message file ```GnssPVTSolnMsg.msg``` and ```GnssTimeMsg.msg``` is copied from [ublox_driver](https://github.com/HKUST-Aerial-Robotics/ublox_driver). Many thanks to Dr. Cao for his work.
## 4.2 Code reference
Thanks to these code contributors:

[ntrip_ros](https://github.com/tilk/ntrip_ros) by tilk;

[ntrip_client](https://github.com/LORD-MicroStrain/ntrip_client) by LORD-MicroStrain;

[RTK-NTRIP-RTCM](https://github.com/Archfx/RTK-NTRIP-RTCM) by Archfx;

[ntrip_ros](https://github.com/MikHut/ntrip_ros) by MikHut;

[ntrip_ros](https://github.com/Road-Balance/ntrip_ros) by Road-Balance.

# 5. TO DO
TCP method (send RTCM data to a given port to [ublox_driver](https://github.com/HKUST-Aerial-Robotics/ublox_driver), like ```str2str``` in [RTKLIB](https://github.com/tomojitakasu/RTKLIB)) is not supported yet.

Problem in ```http.client``` in Python 3 might be solved by [ntrip_ros_python3](https://github.com/bjajoh/ntrip_ros_python3), need to check it out later.

And using ```socket``` instead of ```http.client``` may be a good idea but script written by myself does not work correctly for now, need to work it out later.
