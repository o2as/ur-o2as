#!/bin/bash

roscore

set -e
matlab -nosplash -nodesktop -r '../matlab_graspability/testCreateService'

trap 'matlab -nosplash -nodesktop -r "exit"' 2 | 9
