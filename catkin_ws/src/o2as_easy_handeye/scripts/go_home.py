#!/usr/bin/env python

import sys
from o2as_routines.base import O2ASBaseRoutines


if __name__ == '__main__':
    
    robot_name   = sys.argv[1]
    assert(robot_name in {"a_bot", "b_bot", "c_bot"})

    baseRoutines = O2ASBaseRoutines()
    baseRoutines.go_to_named_pose("home", robot_name)
