#!/usr/bin/env python

import csv
import rospy
from o2as_calibration.calibrator import HandeyeCalibrator
from geometry_msgs.msg import Vector3, Quaternion, Transform

def read_samples(filepath):
  samples = []
  with open(filepath, 'r') as inputfile:
    reader = csv.reader(inputfile)
    header = next(reader)
    for line in reader:
      # transform from robot base to end efector
      rob = Transform()
      rob.translation.x = float(line[0])
      rob.translation.y = float(line[1])
      rob.translation.z = float(line[2])
      rob.rotation.x    = float(line[3])
      rob.rotation.y    = float(line[4])
      rob.rotation.z    = float(line[5])
      rob.rotation.w    = float(line[6])
      # transform from camera to marker
      opt = Transform()
      opt.translation.x = float(line[7])
      opt.translation.y = float(line[8])
      opt.translation.z = float(line[9])
      opt.rotation.x    = float(line[10])
      opt.rotation.y    = float(line[11])
      opt.rotation.z    = float(line[12])
      opt.rotation.w    = float(line[13])
      samples.append({'robot': rob, 'optical': opt})
  return samples

def main():
  # read samples from csv file
  filename = rospy.get_param("~sample_filename")
  samples = read_samples(filename)
  # calculate calibration
  calibrator = HandeyeCalibrator()
  calibrator.set_transforms(samples)
  calibrator.compute_calibration()

if __name__ == '__main__':
  rospy.init_node('offline_calibration', anonymous=True, log_level=rospy.DEBUG)
  rospy.loginfo("offline calibration")
  main()
