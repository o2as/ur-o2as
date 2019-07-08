#!/usr/bin/env python

import csv
import os
from operator import itemgetter

import rospy
import rospkg
rp = rospkg.RosPack()

from bin_layout_generator import Bin, Sets

def read_orderlist():

    part_set = set()
    with open(os.path.join(rp.get_path("o2as_scene_description"), "urdf/templates", "ExampleOfSetListFile.csv"), 'r') as f:
        reader = csv.reader(f)
        header = next(reader)
        # [0, 1, 2, 3] = ["Set", "No.", "ID", "Name", "Note"]
        for data in reader:
            part_set.add("part_" + data[2])

    # print(part_set)

    part_bin_definition = dict()
    with open(os.path.join(rp.get_path("o2as_scene_description"), "urdf/templates", "part_bin_definitions.csv"), 'r') as f:
        reader = csv.reader(f)
        header = next(reader)

        for data in reader:
            part_bin_definition[data[0]] = data[1]

    # print(part_bin_definition)

    part_bin_list = list()
    for part in part_set:
        part_bin_list.append([part, part_bin_definition[part]])
    # print(part_bin_list)

    part_bin_list_sorted = sorted(part_bin_list, key=itemgetter(1), reverse=True)
    # print(part_bin_list_sorted)

    sets = [part_bin_list_sorted[:5], part_bin_list_sorted[-1:4:-1]]
    # print(sets)

    set_number = 0
    set_list = list()
    for s in sets:
        set_number += 1
        bin_num = 1
        previous_bin_type = ''
        for b in s:
            if previous_bin_type == b[1]:
                bin_num += 1
            else:
                bin_num = 1
            # print("parts_name: {}, bin_type: {}, bin_name: {}".format(b[0], b[1], "set" + str(set_number) + "_" + b[1] + "_" + str(bin_num)))
            set_list.append([b[0], b[1], "set" + str(set_number) + "_" + b[1] + "_" + str(bin_num)])
            previous_bin_type = b[1]
    print(set_list)

def main():
    os.chdir('../')
    directory = os.getcwd()

    read_orderlist()

if __name__ == '__main__':
    main()
