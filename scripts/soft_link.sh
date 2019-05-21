# MUST call this bash file at current directory.
#CALL this bash file with 2 arguments;
#First argument must be either blue or red;
#Second argument must be either 1 or 2;
#For example, source soft_link.sh blue 1;


#set prefix of path
path=/home/nvidia/robows/src/RoboRTS-v2/scripts
constraint_set_path="$path/../roborts_detection/armor_detection/constraint_set/config/"
decision_path="$path/../roborts_decision/config/"
armor_detection_path="$path/../roborts_detection/armor_detection/config/"
localization_path="$path/../roborts_localization/config/"
yolo_path="$path/../roborts_detection/armor_detection/yolo/config/"


# making soft link
echo $1 $2
ln -sf "$constraint_set_path/constraint_set_$1_$2.prototxt" "$constraint_set_path/constraint_set.prototxt"
ln -sf "$decision_path/decision_$1_$2.prototxt" "$decision_path/decision.prototxt"
ln -sf "$armor_detection_path/armor_detection_$1_$2.prototxt" "$armor_detection_path/armor_detection.prototxt"
ln -sf "$localization_path/localization_$1_$2.yaml" "$localization_path/localization.yaml"
ln -sf "$yolo_path/yolo_$1_$2.prototxt" "$yolo_path/yolo.prototxt"
