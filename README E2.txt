Avans Hogeschool Breda - 30-6-2026
README Project 2.4 Robot Learning E2 
Opdracht: Een AI-Model trainen die in staat is objecten op te pakken en te verplaatsen. 
Hardware: UFactory UR5 Robot - TOS Teachbot - Robotiq Gripper - PC - OpenCV Camera's (3x)
-------------------------------------------------------------------------------------------------
Stappen voor het opstarten huidige system (5 verschillende terminals)
-------------------------------------------------------------------------------------------------
	1 Start Teachbot
-------------------------------------------------------------------------------------------------
## Start teachbot
# Controleer de joint angles(ongeveer): -8, 106, -72 -25 10 -7
ros2 launch teachbot_ros teachbot_rviz.launch.py

---------------------------------------------------------------------------------
## Informatie over teachbot verkrijgen
ping 192.168.1.50                           # Controleer verbinding met teachbot
ros2 topic echo /teachbot/joint_states      # joint state info
ros2 topic echo /teachbot/state             # status sensoren op de teachbot

---------------------------------------------------------------------------------
## Teachbot starten zonder Teachbot Control monitor
ros2 launch teachbot_ros teachbot_rviz.launch.py use_monitor_gui:=false
-------------------------------------------------------------------------------------------------
	2 Start UR Robot
-------------------------------------------------------------------------------------------------
## UR bot starten 
# Start eerst UR bot (met paneel) op en verbind met zijn wifi netwerk: TP-Link_84B8
ros2 launch my_ur_bringup real_robot.launch.py initial_joint_controller:=forward_position_controller

# check of process succesvol gestart is (zie terminal), druk dan op start op het paneel

---------------------------------------------------------------------------------
## Informatie
ping 192.168.1.101                  # Controleer verbinding met UR bot
ros2 topic echo /joint_states       # joint state info
----------------------------------------------------------------------------------------------------
	3 Start Movegroup
----------------------------------------------------------------------------------------------------
## Start Movegroup
ros2 launch my_ur_bringup movegroup.launch.py launch_servo:=true
----------------------------------------------------------------------------------------------------
	4 Start Robotiq Gripper
----------------------------------------------------------------------------------------------------
## Start gripper
# Controleer of de gripper open en dicht gaat, anders run het command nog een keer
ros2 launch my_robotiq_controller robotiq_controller.launch.py 

---------------------------------------------------------------------------------
## Handmatig de gripper openen en sluiten
ros2 run robotiq_app control_gripper --open
ros2 run robotiq_app control_gripper --close

---------------------------------------------------------------------------------
## Informatie
ros2 topic echo /robotiq_gripper_controller/joint_states    # joint state info
---------------------------------------------------------------------------------------------------
	5 Start Teleoperation
---------------------------------------------------------------------------------------------------
## Naar omgeving gaan

cd ~/lerobot
source .venv/bin/activate

---------------------------------------------------------------------------------
## Alleen bewegen met teachbot

lerobot-teleoperate \
  --robot.type=lerobot_robot_ur \
  --teleop.type=lerobot_teleoperator_teachbot \
  --fps=75

---------------------------------------------------------------------------------
## Dataset maken
# Check camera type met: lerobot-find-cameras
# Check camera of beeld goed is
# Verander XXX in: --dataset.repo_id=zjandaman/XXX

lerobot-record \
    --robot.type=lerobot_robot_ur \
    --robot.id=Hughie \
    --teleop.type=lerobot_teleoperator_teachbot \
    --teleop.id=TeachHughie \
    --robot.cameras="
    {side: {type: opencv, index_or_path: 3, width: 640, height: 480, fps: 30},
    wrist: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30},
    top: {type: opencv, index_or_path: 6, width: 640, height: 480, fps: 30}}" \
    --display_data=true \
    --dataset.fps 15 \
    --dataset.repo_id=zjandaman/XXX \
    --dataset.num_episodes=30 \
    --dataset.single_task="Grab banana and deposit it in the box" \
    --dataset.streaming_encoding=true

---------------------------------------------------------------------------------
## Replay dataset episode
# Episode 0 = 1e episode
# Vernader XXX naar gewenst dataset: --dataset.repo_id=zjandaman/XXX

lerobot-replay \
    --robot.type=lerobot_robot_ur \
    --robot.id=Hughie \
    --dataset.repo_id=zjandaman/Hail_Mary \
    --dataset.episode=0

---------------------------------------------------------------------------------
## Dataset trainen
# Verander policy.repo_id, dataset.repo_id en output_dir van naam(XXX): --policy.repo_id=zjandaman/x, --dataset.repo_id=zjandaman/x, --output_dir=outputs/train/x

lerobot-train \
  --policy.type=smolvla \
  --policy.device=cuda \
  --policy.repo_id=zjandaman/XXX \
  --dataset.repo_id=zjandaman/XXX \
  --batch_size=16 \
  --steps=20000 \
  --output_dir=outputs/train/XXX \
  --job_name=my_smolvla_training \
  --wandb.enable=false

---------------------------------------------------------------------------------
## AI-model 
# Verander X bij: --dataset.repo_id=zjandaman/eval_XXX en --dataset.num_episodes=XXX
# Check of naam overeenkomt met getrainde dataset bij: --policy.path=zjandaman/smolvla

lerobot-record \
    --robot.type=lerobot_robot_ur \
    --teleop.type=lerobot_teleoperator_teachbot \
    --robot.id=Hughie \
    --robot.cameras="
    {side: {type: opencv, index_or_path: 3, width: 640, height: 480, fps: 30},
    wrist: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30},
    top: {type: opencv, index_or_path: 6, width: 640, height: 480, fps: 30}}" \
    --dataset.single_task="Grab banana and deposit it in the box" \
    --display_data=true \
    --dataset.repo_id=zjandaman/eval_XXX \
    --dataset.episode_time_s=50 \
    --dataset.num_episodes=XXX \
    --policy.path=zjandaman/smolvlaXXX

---------------------------------------------------------------------------------
## Laatst gemaakte smolVLA model runnen

rm -rf /tmp/smolvla_test
lerobot-record \
    --robot.type=lerobot_robot_ur \
    --robot.id=Hughie \
    --teleop.type=lerobot_teleoperator_teachbot \
    --teleop.id=TeachHughie \
    --robot.cameras="
    {side: {type: opencv, index_or_path: 3, width: 640, height: 480, fps: 30},
    wrist: {type: opencv, index_or_path: 4, width: 640, height: 480, fps: 30},
    top: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" \
    --dataset.repo_id=debug/eval_no_save_test \
    --dataset.root=/tmp/smolvla_test \
    --dataset.single_task="Grab banana and deposit it in the box" \
    --dataset.episode_time_s=50 \
    --dataset.num_episodes=30 \
    --dataset.fps 15 \
    --dataset.push_to_hub=false \
    --policy.path=zjandaman/smolvla5 \
    --display_data=true
-------------------------------------------------------------------------------------------------------
	ROS2 Communicatie (puur ter info, heft geen commands in terminal) 
-------------------------------------------------------------------------------------------------------
## gebruikte topics

/joint_states
/robotiq_gripper_controller/joint_states
/robotiq_gripper_controller/set_joint_state
/teachbot/joint_states
/teachbot/state

