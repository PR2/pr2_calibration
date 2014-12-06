#! /bin/bash

if [ -f robot_calibrated.xml ]; then
  echo "./robot_calibrated.xml already exists. Either back up this file or remove it before continuing"
  exit 1
fi

echo "Checking if we can write to ./robot_calibrated.xml..."
touch robot_calibrated.xml
if [ "$?" -ne "0" ]; then
  echo "Not able to write to ./robot_calibrated.xml"
  echo "Make sure you run this script from a directory that for which you have write permissions."
  exit 1
fi
rm robot_calibrated.xml
echo "Success"

export PYTHONPATH=`rospack find pr2_calibration_launch`/pr2_urdf_parser_py:$PYTHONPATH

roslaunch pr2_calibration_launch pr2_se_params.launch
rosrun calibration_estimation multi_step_cov_estimator.py /tmp/pr2_calibration/cal_measurements.bag /tmp/pr2_calibration __name:=cal_cov_estimator

est_return_val=$?

if [ "$est_return_val" -ne "0" ]; then
  echo "Estimator exited prematurely with error code [$est_return_val]"
  exit 1
fi

# Make all the temporary files writable
chmod ag+w /tmp/pr2_calibration/*

rosrun pr2_calibration_launch backup_measurements.sh
