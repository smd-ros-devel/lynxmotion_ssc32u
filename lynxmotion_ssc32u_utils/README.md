lynxmotion_ssc32u_utils
=======================

Contains various utilites for the Lynxmotion SSC-32U servo controller.

SSC-32U Shell
-------------

The ssc32u_shell lets you send raw commands to the device for testing purposes. Run the shell using the following command.

```bash
ros2 run lynxmotion_ssc32u_utils ssc32u_shell --port <port> --baud <baud>
```

If port and baud are left off, they will default to /dev/ttyUSB0 and 9600, respectively.

Once the shell is running you can send raw commands to the device. Here's and example of setting the pulse width of channel 0 to 1500.

```bash
# Run this inside the ssc32u_shell
> #0P1500
```

For convienience a full command reference is include by typing the 'help' command.

```bash
# Run this inside the ssc32u_shell
> help
```

To exit the shell, type 'exit' or 'quit'.
