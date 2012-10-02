#!/bin/bash

DIR=`readlink -f $1`
if  test ! -d "$DIR"; then  
  echo "'$DIR' is not a directory" 
  echo "Usage: $0 <directory-where-results-are>"
  echo "e.g., $0 some/path/to/SURF/"
  exit
fi
pushd $DIR > /dev/null
rm -f ate_evaluation_*.csv
rm -f evaluation_*.csv
for num in  1 ; do 
  rm -f eval_translational.txt eval_translational.ate.txt eval_rotational.txt eval_runtime.txt 
  for BASENAME in `ls -d rgbd_dataset_freiburg*`; do
    echo $BASENAME
    if test ! -d "$BASENAME";then
      echo "Not ready yet"
      break;
    fi
    ESTIMATE_FILE=$BASENAME/${BASENAME}.bagiteration_${num}_estimate.txt
    if test ! -f $ESTIMATE_FILE;then
      echo "No estimate at level " $num 
      continue;
    fi
    EVAL_FILE=$ESTIMATE_FILE.evaluation
    if rosrun rgbd_benchmark_tools evaluate_rpe.py --verbose $BASENAME/${BASENAME}-groundtruth.txt $ESTIMATE_FILE > $EVAL_FILE; then
      rosrun rgbd_benchmark_tools evaluate_ate.py --plot $BASENAME/$BASENAME.difference_plot$num.png --verbose $BASENAME/${BASENAME}-groundtruth.txt $ESTIMATE_FILE > $EVAL_FILE.ate
      #rosrun rgbd_benchmark_tools align_and_plot.py --plot $BASENAME/$BASENAME.alignment_plot$num.png --verbose $BASENAME/${BASENAME}-groundtruth.txt $ESTIMATE_FILE > /dev/null

      #RMSE
      echo -n "FR${BASENAME#rgbd_dataset_freiburg}; " >> eval_translational.txt
      echo -n "FR${BASENAME#rgbd_dataset_freiburg}; " >> eval_translational.ate.txt
      echo -n "FR${BASENAME#rgbd_dataset_freiburg}; " >> eval_rotational.txt
      grep translational_error.rmse $EVAL_FILE |sed "s#$DIR##g" | sed 's/ /; /g' >> eval_translational.txt
      grep rotational_error.rmse $EVAL_FILE |sed "s#$DIR##g" | sed 's/ /; /g' >> eval_rotational.txt
      grep translational_error.rmse $EVAL_FILE.ate |sed "s#$DIR##g" | sed 's/ /; /g' >> eval_translational.ate.txt

      #OVERALL RUNTIME
      STARTTIME=`grep "First RGBD-Data Received" $BASENAME/logfile |head -n1|grep -o '13[0-9]*\.'` #timestamp first relevant action
      STARTTIME_NSEC=`grep "First RGBD-Data Received" $BASENAME/logfile |head -n1|grep -o '\.[0-9]*'` #timestamp first relevant action
      ENDTIME=`grep "Finished with optimization iteration $num[^0-9]" $BASENAME/logfile |head -n1|grep -o '13[0-9][0-9]*\.'` #timestamp first relevant action
      ENDTIME_NSEC=`grep "Finished with optimization iteration $num[^0-9]" $BASENAME/logfile |head -n1|grep -o '\.[0-9][0-9]*'` #timestamp first relevant action
      echo -n "Start; ${STARTTIME%.}.${STARTTIME_NSEC#.};s; End; ${ENDTIME%.}.${ENDTIME_NSEC#.};s;" >> eval_runtime.txt

      if [[ "$NO_OPTIMIZER_EVAL" == "" ]]; then 
        #OPTIMIZER RUNTIME
        OPT_TIME=`grep -B10 "Finished with optimization iteration $num[^0-9]" $BASENAME/logfile |grep -o 'Optimizer Runtime; [0-9.]*'` #timestamp first relevant action
        echo -n "${OPT_TIME};s;" >> eval_runtime.txt


        #NUMBER OF NODES
        G2O_LINE=`grep -B10 "Finished with optimization iteration $num[^0-9]" $BASENAME/logfile |grep "G2O Statistics:"|tail -n1`
        NODE_NUM=`echo $G2O_LINE | grep -o '[0-9]* nodes'` #timestamp first relevant action
        EDGE_NUM=`echo $G2O_LINE | grep -o '[0-9]* edges'` #timestamp first relevant action
        echo -n "Number of Nodes/Edges; ${NODE_NUM% nodes};${EDGE_NUM% edges};" >> eval_runtime.txt
      fi
      echo >> eval_runtime.txt

    else
      echo "Evaluation Failed"
    fi
  done
  paste "-d;" eval_rotational.txt eval_translational.txt eval_runtime.txt |sed "s#$DIR/##g" | sed 's/rgbd_dataset_freiburg/FR/g' |sed 's/.evaluation//g' |sed 's/.bagafter._optimization_estimate.txt//g'|sed 's/.bag//g'|sed 's/flowerbouquet/flwrbqt/g' |sed 's/background/bg/g'|sed 's#/FR[^/]*/##g'|sed 's/_/ /g' > evaluation_$num.csv
  paste "-d;" eval_translational.ate.txt eval_runtime.txt |sed "s#$DIR/##g" | sed 's/rgbd_dataset_freiburg/FR/g' |sed 's/.evaluation//g' |sed 's/.bagafter._optimization_estimate.txt//g'|sed 's/.bag//g'|sed 's/flowerbouquet/flwrbqt/g' |sed 's/background/bg/g'|sed 's#/FR[^/]*/##g'|sed 's/_/ /g' > ate_evaluation_$num.csv
  echo ATE Results at Level $num are stored in $DIR/ate_evaluation_$num.csv
  column '-s;' -t  ate_evaluation_$num.csv
  echo RPE Results at Level $num are stored in $DIR/evaluation_$num.csv
  column '-s;' -t  evaluation_$num.csv
done
rm -f eval_translational.txt eval_translational.ate.txt eval_rotational.txt eval_runtime.txt 

popd > /dev/null
