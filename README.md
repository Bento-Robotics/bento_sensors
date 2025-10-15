# bento_sensors

ROS integration for all sorts of sensors we use in our robots, all connected directly to linux/raspberry pi GPIO.

---

## Launching
To start this software, use the [launch file](./bento_sensors.launch.yaml).
You need to select which sensors to use in it's arguments,
per default none are selected, so nothing will happen.

List arguments:
```console
$ ros2 launch bento_sensors bento_sensors.launch.yaml --show-args
Arguments (pass arguments as '<name>:=<value>'):

     'bento_ns':
        set namespace for sensor nodes
        (default: 'bento')

    'do_demo_sensor':
        run example node for a python sensor
        (default: 'False')
```

Select `demo_sensor`:
```console
$ ros2 launch bento_sensors bento_sensors.launch.yaml do_demo_sensor:=True
...
[INFO] [demo_sensor_node.py-1]: process started with pid [578]
[demo_sensor_node.py-1] [INFO] [1760563864.579302090] [bento.demo_sensor_py]: Publishing: "Hello World"
...
```
