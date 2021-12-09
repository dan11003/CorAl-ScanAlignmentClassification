
BAG_BASE_DIR="${BAG_LOCATION}/oxford-eval-sequences/"
SEQUENCE="2019-01-10-11-46-21-radar-oxford-10k"
BAG_FILE_PATH="${BAG_BASE_DIR}/${SEQUENCE}/radar/${SEQUENCE}.bag"
roslaunch alignment_checker vis.launch&
rosrun alignment_checker evaluate_scans --input-file-path ${BAG_FILE_PATH}
