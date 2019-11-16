lynxmotion_ssc32u_driver
========================

ROS driver for the Lynxmotion SSC-32U servo controller.

Usage
-----

The driver is exposed as a component named `ssc32u_driver_node`, but you may also run it as an individual node with the following command.

```bash
ros2 run lynxmotion_ssc32u_driver ssc32u_driver
```

**Note**: You may first need to give read/write permissions on the port the device is located. For example, if the device is on `/dev/ttyUSB0`, then use the following command before running the driver for the first time.

```bash
sudo chmod a+rw /dev/ttyUSB0
```

Parameters
----------

| Parameter   | Type        | Default      | Description |
| ----------- | ----------- | -------      | ----------- |
| **port**    | string      | /dev/ttyUSB0 | Port where the device is located. |
| **baud**    | int         | 9600         | Baud rate of the device. Possible values are 9600, 38400, 115200. Using 115200 is recommended. |
| **publish_pulse_width** | bool | true | Indicates if the current pulse widths for each channel should be published. |
| **publish_rate** | int | 5 | Publish rate of the pulse widths in Hz. Not recommened to go higher than 10. |
| **channel_limit** | int | 16 | Restricts the number of channels the driver will use. Reducing the number will allow you to increase the publish_rate. |
