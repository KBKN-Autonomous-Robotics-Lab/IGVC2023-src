#!/usr/bin/env python3
# maintainer:kbkn/mori
'''
******************************************
This node gets the map coordinates to the 
given goal based on the current latitude, 
longitude and azimuth of the robot.
******************************************
'''
import rospy
import serial
import math
import ruamel.yaml

class GPSDataToxyz:
  def __init__(self):
    # latitude and longitude information given by the organizer at IGVC
    self.given_data = []
    # azimuth
    # true north is 0 degrees, clockwise angle
    self.theta = 0.0
    # name of the port that connects to gps device
    self.dev_name = ""
    # country_id == 0, then Japan
    # country_id == 1, then USA
    self.country_id = 0
    # south, west, north, east. Each (latitude, longitude)
    # file directory for reading parameters
    self.input_file = "/home/ubuntu/catkin_ws/src/igvc2023/config/gps_parameters.yaml"
    # directory of the file where waypoints will be saved
    self.output_file = ""

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
        rospy.loginfo("Data acquisition succeeded!! count: %d", count)
        count += 1

    latitude_mean = sum(latitude_list) / len(latitude_list)    # mean value
    longitude_mean = sum(longitude_list) / len(longitude_list) # mean value

    latitude_list.sort()
    longitude_list.sort()
    latitude_median = latitude_list[ int(len(latitude_list) // 2) ]    # meadian value
    longitude_median = longitude_list[ int(len(longitude_list) // 2) ] # median value
    serial_port.close()
    if gps_data[3] == b"N":   # NORTH
      rospy.loginfo(" NORTH ")
    elif gps_data[3] == b"S": # SOUTH
      latitude_median *= latitude_mean
      rospy.loginfo(" SOUTH ")
    if gps_data[5] == b"E":   # EAST
      rospy.loginfo(" EAST ")
    elif gps_data[5] == b"W": # WEST
      longitude_median *= -1
      rospy.loginfo(" WEST ")
    else :
      rospy.logerr(" error ")
    latitude_longitude_coordinate = (latitude_median, longitude_median)
    rospy.loginfo("current latitude and longitude (latitude, longitude):")
    print(latitude_longitude_coordinate)
    return latitude_longitude_coordinate

  '''
  ++++++++++++++++++++++++++++++++++
         conversion function
  ++++++++++++++++++++++++++++++++++
  Convert (latitude , longitude, altitude) coordinates to (x, y, z) for a waypoint.
  Altitude is an additional factor. It can be calculated even if it is not included.
  '''
  def conversion(self, coordinate, origin, theta):
    if len(coordinate) == 3:
      z = coordinate[2] - origin[2]
    else:
      z = 0

    # set up constants
    a = 1.0 * 6378137
    f = 1.0 * 35 / 10439
    e1 = 1.0 * 734 / 8971
    e2 = 1.0 * 127 / 1547
    n = 1.0 * 35 / 20843
    a0 = 1.0 * 1
    a2 = 1.0 * 102 / 40495
    a4 = 1.0 * 1 / 378280
    a6 = 1.0 * 1 / 289634371
    a8 = 1.0 * 1 / 204422462123
    degree_to_radian= math.pi / 180

    # calculation below
    delta_latitude = coordinate[0] - origin[0]
    delta_longitude = coordinate[1] - origin[1]

    r_latitude = coordinate[0] * degree_to_radian
    r_longitude = coordinate[1] * degree_to_radian
    r_latitude_origin = origin[0] * degree_to_radian
    r_longitude_origin = origin[1] * degree_to_radian
    r_delta_latitude = delta_latitude * degree_to_radian
    r_delta_longitude = delta_longitude * degree_to_radian
    W = math.sqrt(1 - (e1 ** 2) * (math.sin(r_latitude) ** 2) )
    N = a / W
    t = math.tan(r_latitude)
    ai = e2*math.cos(r_latitude)

    S = a * (a0 * r_latitude - a2 * math.sin(2 * r_latitude) + a4 * math.sin(4 * r_latitude ) - a6 * math.sin(6 * r_latitude ) + a8 * math.sin(8 * r_latitude )) / (1 + n)
    if S != 0:
      S0 = a * (a0 * r_latitude_origin - a2 * math.sin(2 * r_latitude_origin) + a4 * math.sin(4 * r_latitude_origin) - a6 * math.sin(6 * r_latitude_origin) + a8 * math.sin(8 * r_latitude_origin)) / (1 + n)
      m0 = S / S0
      B = S - S0
      y1 = (r_delta_longitude ** 2) * N * math.sin(r_latitude) * math.cos(r_latitude) / 2
      y2 = (r_delta_longitude ** 4) * N * math.sin(r_latitude) * (math.cos(r_latitude) ** 3) * (5 - (t ** 2) + 9 * (ai ** 2) + 4 * (ai ** 4)) / 24
      y3 = (r_delta_longitude ** 6) * N * math.sin(r_latitude) * (math.cos(r_latitude) ** 5) * (61 - 58 * (t ** 2) + (t ** 4) + 270 * (ai ** 2) - 330 * (ai ** 2) * (t ** 2)) / 720
      y = m0 * (B + y1 + y2 + y3)

      x1 = r_delta_longitude * N * math.cos(r_latitude)
      x2 = (r_delta_longitude ** 3) * N * (math.cos(r_latitude) ** 3) * (1 - (t ** 2)+(ai ** 2)) / 6
      x3 = (r_delta_longitude ** 5) * N * (math.cos(r_latitude) ** 5) * (5 - 18 * (t ** 2) + (t ** 4) + 14 * (ai ** 2) - 58 * (ai ** 2) * (t ** 2)) / 120
      x = m0 * (x1 + x2 + x3)

    r_theta = theta * degree_to_radian
    h_x = math.cos(r_theta) * x  -  math.sin(r_theta) * y
    h_y = math.sin(r_theta) * x + math.cos(r_theta) * y
    waypoint = (h_y, -h_x, z)
    rospy.loginfo("waypoint:")
    print(waypoint)
    return waypoint

  '''
  ++++++++++++++++++++++++++++++++++
       dump_waypoints function
  ++++++++++++++++++++++++++++++++++
  Save waypoints to the file.
  '''
  def dump_waypoints(self, waypoints, file, reverse):
    if reverse == True:
      waypoints.reverse()

    yaml = ruamel.yaml.YAML()
    yaml.indent(mapping=4, sequence=4, offset=4)

    # variables
    yaml_contents = {}
    yaml_points = []

    for point in waypoints:
      tmp = zip(["x", "y", "z"], point)
      point_dict = {"point" : {i:j for i, j in tmp}}
      yaml_points.append(point_dict)

    yaml_contents["waypoints"] = yaml_points

    # save to a file
    with open(file, "w") as file:
      yaml.dump(yaml_contents, file)

    rospy.loginfo("File successfully saved to %s", file)

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
    self.output_file = parameters["output_file"]
    self.country_id = parameters["country_id"]
    self.theta = parameters["azimuth"]
    self.given_data = parameters["latitude_longitude"]
    rospy.loginfo("parameters successfully import from %s", file)  

  '''
  ++++++++++++++++++++++++++++++++++
            main function
  ++++++++++++++++++++++++++++++++++
  '''     
  def main(self):
    # get parameters from yaml file
    self.load_parameters(self.input_file)
    # get latitude and longitude from gps module
    initial_coordinate = self.get_gps(self.dev_name, self.country_id)

    # calculation
    print(self.given_data)
    waypoints = [self.conversion(each, initial_coordinate, self.theta) for each in self.given_data]
    # save waypoint's data to yaml file
    self.dump_waypoints(waypoints, self.output_file, False)

'''
++++++++++++++++++++++++++++++++++
              main
++++++++++++++++++++++++++++++++++
'''
if  __name__ == "__main__":
    # init node
    rospy.init_node("gps_data_acquisition")
    gtxyz = GPSDataToxyz()
    gtxyz.main()
    # spin
    rospy.spin()
