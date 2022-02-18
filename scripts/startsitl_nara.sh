#!/bin/bash
cd ~/Akasasura/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --out=udp:192.168.18.63:14550 --out=udp:127.0.0.1:14553
