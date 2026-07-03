# **Complete Guide for Hardware Integration**
This page contains all of our knowledge of setting up drones/planes for our ROS2/PX4 stack. Steps for setting up flight controllers, hardware testing between Pis and FCs, and common flight test failures and how to fix them.


## **PX4 Flight Controller Setup**

#### **Power Setup**
If using power module (HolyBro QAV 250) make sure power readings are calibrated. You don't really need to do this often but it is always good to check. Bad power calibrations can cause random battery failsafes if battery is at a medium to medium high voltage. [QGC Power Setup](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/power.html)



## **HITL Testing**
Many problems encountered during flight tests can be prevented by testing each hardware component beforehand. The follow section will cover tests to make sure PX4 is working properly, individual Pi tests and finally Pi + Pixhawk tests.

#### **Pi &harr; Pixhawk Middleware**

[Reference: PX4 docs](https://docs.px4.io/main/en/companion_computer/pixhawk_rpi)

We use UXRCE-DDS/MicroXRCEAgent over a wired serial UART connection from Pi to Pixhawk. You can test if this is working with just an FC and a Pi.



- Make sure that uxrce_dds_client is running properly on the FC. You can do this by running `uxrce_dds_client status` in QGC MavLink Console. If not started, run `uxrce_dds_client start -t serial -d /dev/ttyS3 -b 921600`. If PX4 somehow doesn't start the middleware client on startup, you can create a startup script in the Pixhawk SD Card
- Run the middleware on a Pi ssh session. It should be something like `MicroXRCEAgent serial --dev /dev/serial0 -b 921600`


**If it doesn't work, double check these things:**


- Make sure that the `UXRCE_DDS_CFG` param is set to the correct port. (For 6C-mini its usually set to TELEM 2). Also make sure the baud rate for the corresponding port is set the same as specified in the client command (`SER_TEL2_BAUD`)
- Double check the wiring. Make sure you have PX4 GND &harr; PI GND, PX4 TX &harr; PI RX, and PX4 RX &harr; PI TX
- On Pi side double check that you can see the serial ports. `ls -l /dev/ttyAMA*` or `ls -l /dev/serial*`. Also double check to see if any other process are grabbing the UART GPIO pins for another process (like pigpiod)





## Other Errors You Might Encounter:

**Pixhawk keeps logging messages about regaining and losing manual control which causes a failsafe in flight.**
You can set the `COM_RC_IN_MODE` to value 4 which is (disable/ignore all) to ignore all manual control input.

**"Crash Dumps present on SD Card" message that prevents arming**
This usually happens when something crashes in firmware and it creates fault/dump files in logs. The issue can be resolved by manually deleting the files from the sd card. To be safe you should save params and reflash firmware beforehand.

I ran into this issue once and I had to go to mavlink console on QGC and go to /fs/microsd/log and delete a <fault_date_and_time.log> file
