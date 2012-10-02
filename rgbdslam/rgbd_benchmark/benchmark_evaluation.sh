#!/bin/bash

echo Feature Type: $1 Bagfile Directory: $2 Output Directory Prefix: $3
if [[ "$1" == "" ]] || test  '!' -d "$2" ; then
  echo "Usage: $0 <feature-type> <directory-where-bagfiles-and-groundtruth-files-are>"
  echo "E.g.: $0 SURF ~/ros/rgbdslam/rgbd_benchmark/benchmark_data/"
  exit
fi
pushd `readlink -f $2` > /dev/null

export ROS_MASTER_URI=http://localhost:11381
roscore -p 11381&
sleep 1

##Evaluate Feature types
rosparam set /rgbdslam/config/feature_detector_type $1
rosparam set /rgbdslam/config/feature_extractor_type $1

##Evaluate SLAM Backends
#rosparam set /rgbdslam/config/feature_detector_type  SIFTGPU
#rosparam set /rgbdslam/config/feature_extractor_type SIFTGPU
#rosparam set /rgbdslam/config/backend_solver $1

echo "Will evaluate RGBD-SLAM on the following bagfiles:"
SELECTION=rgbd_dataset_freiburg1_*.bag
echo $SELECTION

for bagfile in $SELECTION; do
  BASE_NAME=`basename $bagfile .bag` 
  DIRECTORY=$1$3/$BASE_NAME
  mkdir -p $DIRECTORY
  if grep -q Coordinate $DIRECTORY/*estimate.txt 2> /dev/null; then 
    echo There are already results for $BASE_NAME in $DIRECTORY. Will skip this bagfile
    continue #don't overwrite existing results
  fi
  echo `date +%H:%M:%S` Results for $BASE_NAME are stored in `readlink -f $DIRECTORY`
  rosparam set /rgbdslam/config/bagfile_name `readlink -f $bagfile`
  sleep 1
  roslaunch rgbdslam settings_for_evaluation.launch >  $DIRECTORY/logfile 2>&1
  rosparam get /rgbdslam/config >>  $DIRECTORY/logfile 2>&1
  echo `date +%H:%M:%S` Finished processing $BASE_NAME

  #Move Result files, run evaluation routine
  mv ${bagfile}?* $DIRECTORY/
  cp ${BASE_NAME}-groundtruth.txt $DIRECTORY/
  LAUNCHFILE=`rospack find rgbdslam`/rgbd_benchmark/settings_for_evaluation.launch
  cp $LAUNCHFILE $DIRECTORY/settings.xml #renamed to avoid name conflict with original file in roslaunch command
  #cp `readlink -f $0` $DIRECTORY/`basename $0`

done

popd > /dev/null
