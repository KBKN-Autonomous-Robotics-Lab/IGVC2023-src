#!/usr/bin/env python3
# maintainer:kbkn/mori

'''
******************************************
This node displays the current latitude 
and longitude information of the robot.
******************************************
'''
import rospy
import serial
import math
import ruamel.yaml

class GPSCurrentData:
  def __init__(self):
    # name of the port that connects to gps device
    self.dev_name = ''
    # country_id == 0, then Japan
    # country_id == 1, then USA
    self.country_id = 0
    # south, west, north, east. Each (latitude, longitude)
    # file directory for reading parameters
    self.param_file = "/home/ubuntu/catkin_ws/src/igvc2023/config/gps_parameters.yaml"

  '''
  ++++++++++++++++++++++++++++++++++
          get_gps function
  ++++++++++++++++++++++++++++++++++
  This function outputs latitude and longitude coordinates.
  '''
  def get_gps(self, dev_name, country_id):
    # vairalbes
    count = 0
    latitude_sum = 0
    longitude_sum = 0
    latitude_list = []
    longitude_list = []

    # interface with sensor device(as a serial port)
    try:
      serial_port = serial.Serial(dev_name, 19200)
    except serial.SerialException as serialerror:
      print(serialerror)
      rospy.logerr("%s", serialerror)
      serial_port = None

    # country info
    if country_id == 0:   # Japan
      initial_letters = b"GNGGA"
    elif country_id == 1: # USA
      initial_letters = b"GPGGA"
    else:                 # not certain
      initial_lettes = None
    
    failure_count1 = 30
    failure_count2 = 30
    while count < 30:
      # gps_data = ["$G?GGA", "UTC", "Latitude", "N(north)", "Longitude", "E(east)", "1", "# of stalites being tracked", "Horizontal dilution of position", "Altitude", "M(meter)", "Height of geoid", "M(meter)", "", "checksum"]
      line = serial_port.readline()
      talker_ID = line.find(initial_letters)
      if talker_ID != -1:
        line = line[(talker_ID-1):]
        gps_data = line.split(b",")
        latitude_data = gps_data[2]
        longitude_data = gps_data[4]
        if latitude_data == b"":
          if failure_count1 == 30:
            rospy.logwarn("Data acquisition failure!! Retrying...")
            failure_count1 = 0
          failure_count1 += 1
          continue
        else :
          latitude_minute = float(latitude_data[0:2]) + float(latitude_data[2:]) / 60
        if longitude_data == b"":
          if failure_count2 == 30:
            rospy.logwarn("Data acquisition failure!! Retrying...")
            failure_count2 = 0
          failure_count2 += 1
          continue
        else :
          longitude_minute = float(longitude_data[0:3]) + float(longitude_data[3:]) / 60
        latitude_sum += latitude_minute
        longitude_sum += longitude_minute
        latitude_list.append(latitude_minute)
        longitude_list.append(longitude_minute)
        count += 1

    latitude_mean = sum(latitude_list) / len(latitude_list)    # mean value
    longitude_mean = sum(longitude_list) / len(longitude_list) # mean value

    latitude_list.sort()
    longitude_list.sort()
    latitude_median = latitude_list[ int(len(latitude_list) // 2) ]    # meadian value
    longitude_median = longitude_list[ int(len(longitude_list) // 2) ] # median value
    serial_port.close()
    
    if gps_data[3] == b"S":
      latitude_median *= latitude_mean

    if gps_data[5] == b"W":
      longitude_median *= -1

    else :
      rospy.logerr(" error ")
    latitude_longitude_coordinate = (latitude_median, longitude_median)
    rospy.loginfo("current latitude and longitude (latitude, longitude):")
    print(latitude_longitude_coordinate)
    
  '''
  ++++++++++++++++++++++++++++++++++
      load_parameters function
  ++++++++++++++++++++++++++++++++++
  Get parameters from yaml file.
  '''
  def load_parameters(self, file):
    yaml = ruamel.yaml.YAML()
    with open(file) as file:
      parameters = yaml.load(file)
      
    self.dev_name = parameters["dev_name"]
    self.country_id = parameters["country_id"]
    rospy.loginfo("parameters successfully import from %s", file)  

  '''
  ++++++++++++++++++++++++++++++++++
            main function
  ++++++++++++++++++++++++++++++++++
  '''     
  def main(self):
    # get parameters from yaml file
    self.load_parameters(self.param_file)
    # get latitude and longitude from gps module
    self.get_gps(self.dev_name, self.country_id)

'''
++++++++++++++++++++++++++++++++++
              main
++++++++++++++++++++++++++++++++++
'''
if  __name__ == "__main__":
    # init node
    rospy.init_node("gps_data_acquisition")
    gcd = GPSCurrentData()
    gcd.main()
    # spin
    rospy.spin()
