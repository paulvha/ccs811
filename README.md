# version 1.0	initial version  December 2017

Copyright (c) 2017 Paul van Haastrecht <paulvha@hotmail.com>


## Background
As part of a larger project I am looking at analyzing and understanding the air quality. 
The aim of this project was to better understand the kind of gas-types that are in the air. 

I have ported the library to CPP on a Raspberry PI running Raspbian Jessie release. It has been 
adjusted and extended for stable working.

# version 2.0	/  October 2018

 
## Software installation

Make your self superuser : sudo bash

3.1 BCM2835 library
Install latest from BCM2835 from : http://www.airspayce.com/mikem/bcm2835/

1. wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.56.tar.gz
2. tar -zxf bcm2835-1.56.tar.gz		// 1.56 was version number at the time of writing
3. cd bcm2835-1.56
4. ./configure
5. sudo make check
6. sudo make install

In order for this software to run you should NOT enable i2C in raspi-config to load the kernel drivers. 
It works directly on the hardware, but you’ll have to run program as root.

3.2 twowire library
Obtain the latest version from : https://github.com/paulvha/twowire

1. download the zip-file (clone or download / download zip-file) in the wanted directory
2. unzip twowire-master.zip (*1)
3. cd twowire-master
4. make install

*1) if you do not have unzip : sudo apt-get install zip unzip

3.3 CCS811 software
Obtain the latest version from : https://github.com/paulvha/ccs811

1. Download the zip-file (clone or download / download zip-file) in the wanted directory
2. unzip ccs811-master.zip (*1)
3. cd ccs811-master
4. create the executable : make  
5. To run you have to be as sudo ./ccs811 -h ….

(detailed description of the many options in  ccs811.odt in the documents directory)

