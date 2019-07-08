#!/usr/bin/env python
import csv
import os
import sys
import yaml
from operator import itemgetter
from collections import OrderedDict

import rospy
import rospkg
rp = rospkg.RosPack()

class BinDefinition:
    def __init__(self, bin_type, width,length, height,z_origin_offset):
        self.bin_type = bin_type
        self.width = width
        self.length = length
        self.height = height
        self.z_origin_offset =z_origin_offset

class Bin:
    def __init__(self, parts_name, bin_type, bin_name, pos_x=0, pos_y=0, pos_z=0):
        self.parts_name = parts_name
        self.bin_type = bin_type
        self.bin_name = bin_name
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.pos_z = pos_z
    
    def definition(bin_definition):
        self.bin_type = []
        self.width = 0
        self.length = 0
        self.height = 0
        self.z_origin_offset =0

class Sets:
    def __init__(self,origin_x,origin_y,origin_z):
        self.bins =[]
        self.origin_x=origin_x
        self.origin_y=origin_y
        self.origin_z=origin_z

    def add_bin(self, bin):
        self.bins.append(bin)

# write bin id for graspability based vision
def write_fge_bin_id_to_yaml(set_list):
    bin_id_dict = dict()
    bin_id_dict["fge_bin_id"] = dict()
    i = 1
    for s in set_list:
        for b in s.bins:
            bin_id_dict["fge_bin_id"][b.bin_name] = i
            # print("%d, %s"%(i, b.bin_name))
            i+=1
    with open(os.path.join(rp.get_path("graspability_estimation"), "config", "fge_bin_id.yaml"), 'w') as f:
        yaml.dump(bin_id_dict, f)

# write to outfile
def write_head(outfile,directory):
    f = open(os.path.join(rp.get_path("o2as_scene_description"), "urdf/templates", 'o2as_bin_template_front.xacro'),'r')
    template_front = f.read()
    f.close()
    outfile.write(template_front)
    outfile.write("\n\n")

# write bin information
def write_bin_definition(outfile,directory,bin_definition):
    # write bin definition to the file in xml format
    outfile.write(" <!-- space for bin -->\n\n")  
    
    for i in range(len(bin_definition)):
        outfile.write(" <!-- bin"+str(i+1)+"_definition -->\n")
        outfile.write(" <xacro:property name=\""+str(bin_definition[i].bin_type)+"_width\" value=\""+str(bin_definition[i].width)+"\"/>\n")
        outfile.write(" <xacro:property name=\""+str(bin_definition[i].bin_type)+"_length\" value=\""+str(bin_definition[i].length)+"\"/>\n")
        outfile.write(" <xacro:property name=\"z_origin_offset_"+str(i+1)+"\" value=\""+str(bin_definition[i].z_origin_offset)+"\" />\n\n")

def write_bin_set_origin(outfile,directory,set_list):
    for i in range(len(set_list)):
        outfile.write(" <!-- A row of bin"+str(i+1)+" -->\n")
 
        outfile.write(" <xacro:property name=\"set"+str(i+1)+"_x\" value=\""+str(set_list[i].origin_x)+"\"/>\n")
        outfile.write(" <xacro:property name=\"set"+str(i+1)+"_y\" value=\""+str(set_list[i].origin_y)+"\"/>\n")
        outfile.write(" <xacro:property name=\"set"+str(i+1)+"_z\" value=\""+str(set_list[i].origin_z)+"\"/>\n")

def write_bin_layout(outfile,directory,set_list):
    for s in set_list:

        for i in range(len(s.bins)):
            outfile.write(" <!--"+str(s.bins[i].bin_name)+"_-->\n ")
            outfile.write(" <xacro:kitting_bin_"+str(s.bins[i].bin_type[3])+" binname=\""+str(s.bins[i].bin_name)+"\" parent=\"workspace_center\" z_origin_offset=\""+str(s.bins[i].definition.z_origin_offset)+"\">\n ")
            outfile.write("     <origin xyz=\""+str(s.bins[i].pos_x)+" "+str(s.bins[i].pos_y)+" "+str(s.bins[i].pos_z)+"\" rpy=\"0 0 0\" />\n ")
            outfile.write(" </xacro:kitting_bin_"+str(s.bins[i].bin_type[3])+">\n ")

# write tail
def write_tail(outfile,directory):
    f = open(os.path.join(rp.get_path("o2as_scene_description"), "urdf/templates", 'o2as_bin_template_tail.xacro'),'r')
    template_tail = f.read()
    f.close()
    outfile.write("\n\n")
    outfile.write(template_tail)

#calc bins positions
def calc_bin_pos_y(bin,last_bin):
    return float(last_bin.pos_y) - float(bin.definition.width) * 0.5 - float(last_bin.definition.width) * 0.5

def calc_bin_pos_z(bin,origin_z):
    return float(origin_z) - float(bin.definition.z_origin_offset)

def find_bin_definition(bin_definition,bin):
    pass
    for j in range(len(bin_definition)):
        if bin_definition[j].bin_type==bin.bin_type:
            definition=bin_definition[j]
    return definition

def calc_position_of_bins(set_list,bin_definition):
    for s in set_list:
        origin_x = s.origin_x
        origin_y = s.origin_y
        origin_z = s.origin_z
        last_bin = Bin(" ", " ", " ", origin_x, origin_y, origin_z)
        last_bin.definition=BinDefinition(" ", 0, 0, 0, 0)
        for i in range(len(s.bins)):
            bin = s.bins[i]
            bin.definition = find_bin_definition(bin_definition,bin)  
            bin.pos_x = origin_x
            bin.pos_y = calc_bin_pos_y(bin,last_bin)
            bin.pos_z = calc_bin_pos_z(bin,origin_z)  
            last_bin = bin
            s.bins[i] = bin
    return set_list


# read csv files
def read_bin_layout(directory,set_origin):
    with open(os.path.join(rp.get_path("o2as_scene_description"), "urdf/templates", 'placement_bin_layout.csv'), 'r') as f:
        reader = csv.reader(f)
        header = next(reader)

        set_number = 0
        set_list = set_origin
        for line in reader:
          #  token = line.split(",")
            token=line
            if len(token) == 3:
                parts_name = token[0]
                bin_type = token[1]
                bin_name = token[2]
                bin = Bin(parts_name, bin_type, bin_name)
                set_list[set_number].add_bin(bin)
                
            elif token[0] == "---":
                set_number = set_number+1
        return set_list


def read_bins_origin(directory):
    with open(os.path.join(rp.get_path("o2as_scene_description"), "urdf/templates", 'bins_origin.csv'), 'r') as f:
        reader = csv.reader(f)
        header = next(reader)
        set_list = []
        for row in reader:
            origin_x = row[1]
            origin_y = row[2]
            origin_z = row[3]
            set_list.append(Sets(origin_x,origin_y,origin_z))
        return set_list


def read_bin_definition(directory):
    with open(os.path.join(rp.get_path("o2as_scene_description"), "urdf/templates", 'placement_bin_definitions.csv'), 'r') as f:
        reader = csv.reader(f)
        header = next(reader)
        bin_definition=[]
        for row in reader:
            bin_type = row[0]
            width = row[1]
            length = row[2]
            height = row[3]
            z_origin_offset=row[4]
            bin_definition.append(BinDefinition(bin_type,width,length,height,z_origin_offset))
        return bin_definition


def write_file(outfile,directory,set_list,bin_definition):
    #  write front
    write_head(outfile,directory)
    #  write to change bin
    write_bin_definition(outfile,directory,bin_definition)
    write_bin_set_origin(outfile,directory,set_list)
    write_bin_layout(outfile,directory,set_list)
    #  write tail
    write_tail(outfile,directory)


def read_csv_and_calc_bins_positions(directory):
    #read csv files
    bin_definition=read_bin_definition(directory)
    set_origin=read_bins_origin(directory)
    set_list=read_bin_layout(directory,set_origin)
    #calc bins positions
    set_list=calc_position_of_bins(set_list,bin_definition)
    return set_list,bin_definition

def main():
    os.chdir('../')
    directory=os.getcwd()

    set_list,bin_definition=read_csv_and_calc_bins_positions(directory)
    outfile = open(os.path.join(rp.get_path("o2as_scene_description"), "urdf", 'kitting_bins.xacro'),'w+')
    write_file(outfile,directory,set_list,bin_definition)

    write_fge_bin_id_to_yaml(set_list)
    
if __name__ == "__main__":
    main()
