# TDK_等賴博程取隊名

read the following details to know how to run code

## ros2 environment

```bash
//run once each terminal
source install/setup.bash

//if you change the coding content
colcon build 
```

## usb device - find stm and camera tunnel
- connect from bottom to up
- there are many tunnel for camera, you just need to find if the target tunnel is exist
- if the tunnel path is incorrect, unplug all and retry

```bash
//run to inspect when you unplug stm or camera
ls /dev/ttyACM*

ls /dev/video*
```

  stm wheel: /dev/ttyACM0
  stm tasks: /dev/ttyACM1
  camera for coffee card: /dev/video17
  camera for desk: /dev/video4

## about uart and pi system

```bash
stty -F /dev/ttyACM0
setserial /dev/ttyACM0 low_latency
stty -F /dev/ttyACM1
setserial /dev/ttyACM1 low_latency

systemctl status motor_serial_node.service
systemctl status mission_serial_node.service
//if

```


