#!/usr/bin/env python
import csv
import os

import rospy
import rospkg

rp = rospkg.RosPack()

with open(os.path.join(rp.get_path("o2as_scene_description"), "urdf/templates", "placement_mat.csv"), 'r') as csvfile:
    reader = csv.reader(csvfile)
    # print reader
    header = next(reader)
    
    f = open(os.path.join(rp.get_path("o2as_scene_description"), "urdf/templates", 'o2as_mat_template_front.xacro'),'r')
    template_front = f.read()
    f.close()
    
    outfile = open(os.path.join(rp.get_path("o2as_scene_description"), "urdf", 'o2as_mat.xacro'),'w+')
    outfile.write(template_front)
    for row in reader:
        row[1]=float(row[1])/1000.
        row[2]=float(row[2])/1000.
        outfile.write("    <joint name=\"${matname}_part"+row[0]+"\" type=\"fixed\">\n")
        outfile.write("      <parent link=\"${matname}\"/>\n")
        outfile.write("      <child link=\"${matname}_part"+str(row[0])+"\"/>\n")
        outfile.write("      <origin rpy=\"0.0 0.0 ${pi/2}\" xyz=\""+str(row[1])+" "+str(row[2])+" 0.0\"/>\n")
        outfile.write("    </joint>\n")
        outfile.write("    <link name=\"${matname}_part"+str(row[0])+"\"/>\n\n")

    f = open(os.path.join(rp.get_path("o2as_scene_description"), "urdf/templates", 'o2as_mat_template_tail.xacro'),'r')
    template_tail = f.read()
    f.close()
    outfile.write(template_tail)
