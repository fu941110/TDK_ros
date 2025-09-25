# TDK_等賴博程取隊名

read the following details to know how to run code

## connect to pi

```bash
ssh fu941@pi3.local

# password: Aa0908057190

# network (both your device and pi should be the same one)

# if login false, wait, maybe it is not open completely
```

- `after login`
```bash
ls

cd TDK_ros
# here is the ros2 workspace
```

## ros2 environment

```bash
# run once each terminal
source install/setup.bash

# if you change the coding content
colcon build 
```

## usb device - find stm and camera tunnel
- connect from bottom to up
- there are many tunnel for camera, you just need to find if the target tunnel is exist
- if the tunnel path is incorrect, unplug all and retry

```bash
# run to inspect when you unplug stm or camera
ls /dev/ttyACM*
ls /dev/video*
```
- tunnel 
  - stm wheel: /dev/ttyACM0
  - stm tasks: /dev/ttyACM1
  - camera for coffee card: /dev/video17
  - camera for desk: /dev/video4

## about uart and pi system

```bash
# run once
stty -F /dev/ttyACM0
setserial /dev/ttyACM0 low_latency
stty -F /dev/ttyACM1
setserial /dev/ttyACM1 low_latency
```
## before launch

- you should do this every time before launch
- `inspect node`
```bash
ros2 node list

# run if there is any node exist
ps aux | grep <node_name>

# it will output a <PID>, kill it
sudo kill -9 <PID>
```

## launch

- if there is any [ERROR] on terminal, inspect usb device tunnel

- `launch`
```bash
ros2 launch mainspace mainNodes.launch.py
```
- some info will be shown on this termianl, [INFO], or [WARN] 
- you can also open another terminal to watch somthing
- `topic` - track if message is sent
    - /stm_position
    - /commandToSTM
    - /commandToROS
    - /pause
    - /coffee  -> coffeecolor, coffeedesk
```bash
ros2 topic echo /topic_name

# we usually watch coffee
ros2 topic echo /coffee
```


