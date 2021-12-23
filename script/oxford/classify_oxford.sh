
BAG_BASE_DIR="${BAG_LOCATION}/oxford-eval-sequences/"
SEQUENCE="2019-01-10-11-46-21-radar-oxford-10k"
BAG_FILE_PATH="${BAG_BASE_DIR}/${SEQUENCE}/radar/${SEQUENCE}.bag"
current_date=`date '+%Y-%m-%d_%H:%M'`
EVAL_NAME="PROTOTYPE_eval_${current_date}"
OUTPUT_EVAL_PATH="${BAG_LOCATION}/CoralRadarEval/${EVAL_NAME}"
mkdir -p ${OUTPUT_EVAL_PATH}
#rosnode kill -a

roslaunch alignment_checker vis.launch&
rosrun alignment_checker alignment_service.py&
rosrun alignment_checker evaluate_scans --input-file-path ${BAG_FILE_PATH} --output-dir ${OUTPUT_EVAL_PATH} --eval-name ${EVAL_NAME} --sequence ${SEQUENCE} --method P2L --scan-type cfear --range-error 0.5 --rosbag-offset 7000 --frame-delay 0.0 &> /dev/null
