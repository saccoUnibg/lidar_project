#source ros2
echo "1. ros2 source"
source install/setup.bash 
#build
echo "2. Build colcon"
colcon build
#print ok
echo "--- Done. ---"
