# TDK_等賴博程取隊名

read the following details to know how to run code

## Setup

- `ros2 environment`

```bash
source install/setup.bash
colcon build   //if you change the coding content
```

- `usb device` - find stm and camera tunnel
  - connect from bottom to up
  - there are many tunnel for camera, you just need to find if the target tunnel is exist
  - if the tunnel path is incorrect, unplug all and retry

```bash
ls /dev/ttyACM*
//stm for wheel: /dev/ttyACM0
//stm for tasks: /dev/ttyACM1

ls /dev/video*
//camera for coffee card: /dev/video17
//camera for desk: /dev/video4
```

- `about uart`
  




