#!/usr/bin/env python

import csv
import os

import rospy
import rospkg
rp = rospkg.RosPack()

def OrderDefinition(object):
    def __init__(self, no, part_id, part_name):
        self.no = no
        self.part_id = part_id
        self.part_name = part_name

def read_order_list(filepath):

    orders = list()

    with open(filepath, 'r') as inputfile:
        reader = csv.reader(inputfile)
        header = next(reader)

        for line in reader:
            order_num = int(line[0])
            no = line[1]
            part_id = line[2]
            part_name = line[3]
            # part_desc = line[4]   # in the csv, description has undecodable code on utf-8
            if orders[order_num] == None:
                orders[order_num] = list()
            orders[order_num].append(OrderDefinition(no, part_id, part_name))

    return orders

def main():
    orders = read_order_list(os.path.join(rp.get_path("o2as_examples"), "config", "ExampleOfSetListFile.csv"))
    print orders

if __name__ == '__main__':
    main()
