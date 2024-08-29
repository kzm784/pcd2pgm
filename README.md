
# pcd2pgm

A tool based on ROS2 and the PCL library to convert `.pcd` point cloud files into `.pgm` grid maps for use in Navigation.

|pcd|pgm|
|:-:|:-:|
|![pcd](.docs/pcd.png)|![pgm](.docs/pgm.png)|

## 1. Features

- Read a specified `.pcd` file

- Create the `.pgm` file and `.yaml` file from `.pcd` file

## 2. Usage Guide

### 2.1 Clone the Repository

  ```sh
  git clone https://github.com/kzm784/pcd2pgm.git
  ```

### 2.2 Install Dependencies

[ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) (Has not been tested with other ROS versions)

```sh
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

### 2.3 Build

```sh
cd /path/to/your/ros2_ws/
colcon build --symlink-install
```

### 2.4 Launch the PCD2PGM 

```sh
source install/setup.sh
ros2 launch pcd2pgm pcd2pgm.launch.py
```

## 3. Parameter Description

You can configure the node's parameters by modifying the `pcd2pgm/pcd2pgm.yaml` file.

  ```yaml
  pcd2pgm_node:
    ros__parameters:
      resolution: 0.1
      min_points_in_pix: 0
      max_points_in_pix: 1
      min_height: 0.5
      max_height: 100.0
      input_pcd: "/home/kazuma/kuams_ws/src/map_editer/map/test0312.pcd"
      dest_directory: "/home/kazuma/dev_ws/src/pcd2pgm/map"
      output_pgm_name: "test"
  ```
