# To Use Autoware.Universe University of Delaware Version #
1. Download repo
2. Setup Docker and Rocker
3. docker pull williamhecoin/blueice:hydrau-env
4. ```
   sudo rocker --x11 --nvidia --network=host --volume {pathtodirectory}/UDAutoware:/home/carla/autoware --volume {pathtodirectory}/UDAutoware/autoware_map:/home/carla/autoware_map -- williamhecoin/blueice:hydrauenv
   ```
5. Once inside the container:
   ```
   cd /home/carla/autoware/
   ```
7. To compile all of the packages:
   ```
   colcon build --symlink-install
   ```
9. ```
    source install/setup.bash
   ```
11. Command to Run Autoware
    ```
    ros2 launch autoware_launch autoware.launch.xml vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit map_path:=/home/carla/autoware_map/{map_name}
    ```
   
