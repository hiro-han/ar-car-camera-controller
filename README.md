# ar-car-camera-controller

## Build
```
$ cd {catkin_workspace}/src
$ git clone https://github.com/hiro-han/ar-car-camera-controller.git camera_controller
$ cd {catkin_workspace}
$ catkin_make
$ source devel/setup.bash
```

## Run
```
<launch>
  <node name="camera_controller" pkg="camera_controller" type="camera_controller_node"/>
</launch>
```
