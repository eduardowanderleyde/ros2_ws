Como testar no Linux (Jazzy)
1) Build + run
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select route_tool
source install/setup.bash
ros2 run route_tool route_tool

2) Em outro terminal, com Nav2 rodando
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 action list | grep navigate
# você deve ver: /navigate_through_poses

ros2 service call /start_route std_srvs/srv/Trigger {}
# teleopa o robô fazendo um caminho
ros2 service call /stop_route  std_srvs/srv/Trigger {}
ros2 service call /play_route  std_srvs/srv/Trigger {}

Se der erro “action server not available”

Significa uma destas:

Nav2 não está rodando

o nome da action é diferente no teu setup

Ajusta via parâmetro:

ros2 run route_tool route_tool --ros-args -p nav2_action_name:=/navigate_through_poses


Se você quiser o “Play” lendo do YAML (não só da memória), eu adiciono +1 service tipo /load_route ou faço o /play_route carregar automaticamente routes/route1.yaml.