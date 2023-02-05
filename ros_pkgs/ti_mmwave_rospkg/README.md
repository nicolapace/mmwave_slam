## TI mmWave ROS Package (Customized)

### Main Features 
This is a Customized packege of TI mmWave ROS Package from Arizona University (https://github.com/radar-lab/ti_mmwave_rospkg).
In this Packege it was add an optimal configuration for AWR1843AOP mmWave Sensor and an Agglomerative Clustering algorithm to make the 
PointCloud more dense. You can use this package with all Ti mmWave devices by adding a cfg file of the corresponding model and changing 
the sensor orientation coordinates according to the chosen model in the src/DataHandlerClass.cpp file (it was tested with IWR6843AOP and AWR1843AOP).
---
### Message format:
```
header: 
  seq: 6264
  stamp: 
    secs: 1538888235
    nsecs: 712113897
  frame_id: "ti_mmwave"   # Frame ID used for multi-sensor scenarios
point_id: 17              # Point ID of the detecting frame (Every frame starts with 0)
x: 8.650390625            # Point x coordinates in m (front from antenna)
y: 6.92578125             # Point y coordinates in m (left/right from antenna, right positive)
z: 0.0                    # Point z coordinates in m (up/down from antenna, up positive)
range: 11.067276001       # Radar measured range in m
velocity: 0.0             # Radar measured range rate in m/s
doppler_bin: 8            # Doppler bin location of the point (total bins = num of chirps)
bearing: 38.6818885803    # Radar measured angle in degrees (right positive)
intensity: 13.6172780991  # Radar measured intensity in dB
```
---
### Troubleshooting
1.
```
mmWaveCommSrv: Failed to open User serial port with error: IO Exception (13): Permission denied
mmWaveCommSrv: Waiting 20 seconds before trying again...
```
This happens when serial port is called without superuser permission, do the following steps:
```
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyACM1
```
2.
```
mmWaveQuickConfig: Command failed (mmWave sensor did not respond with 'Done')
mmWaveQuickConfig: Response: 'sensorStop
'?`????`????`???~' is not recognized as a CLI command
mmwDemo:/>'
```
When this happens, re-run the command you send to sensor. If it continues, shut down and restart the sensor.


Note: As serial connection and the original code, you need to launch devices separately using different launch files.


## Enable mmWave sensors reading:

- Copy Packagein your PC by git clone 
- Copy Package in your RaspberryPI (It was tasted with Melodic and Noetic Version):
```sh
	$ scp -r <Package_local_path> <RaspberryPI_username>@<IP_address>:~/catkin_ws/src/ 
```
- Open a shell and connect to RaspberryPI in ssh mode:
```sh
	$ ssh <username>@<IP_address> 
```
- run catkin make:
```sh
	$ cd catkin_ws
	$ catkin make
```
- Add USB rules on RaspberryPI:
```sh
	$ cd /etc/udev/rules.d/
	$ sudo nano 99-usb-rules
```
- Paste these lines into the file and save:
```sh
	#SUBSYSTEM=="tty", ATTRS{serial}=="0001" , SYMLINK+="LIDAR" , MODE="0666"
	SUBSYSTEM=="tty",ATTRS{serial}=="00E24511", ENV{ID_USB_INTERFACE_NUM}=="00", SYMLINK+="MMWAVE1843AOP_FRONT_COMMAND" , MODE="0666"
	SUBSYSTEM=="tty",ATTRS{serial}=="00E24511", ENV{ID_USB_INTERFACE_NUM}=="01", SYMLINK+="MMWAVE1843AOP_FRONT_DATA" , MODE="0666"
	SUBSYSTEM=="tty",ATTRS{serial}=="00E244FB", ENV{ID_USB_INTERFACE_NUM}=="00",SYMLINK+="MMWAVE1843AOP_RX_COMMAND" , MODE="0666"
	SUBSYSTEM=="tty",ATTRS{serial}=="00E244FB", ENV{ID_USB_INTERFACE_NUM}=="01",SYMLINK+="MMWAVE1843AOP_RX_DATA" , MODE="0666"
	SUBSYSTEM=="tty",ATTRS{serial}=="00E2450B", ENV{ID_USB_INTERFACE_NUM}=="00",SYMLINK+="MMWAVE1843AOP_LX_COMMAND" , MODE="0666"
	SUBSYSTEM=="tty",ATTRS{serial}=="00E2450B", ENV{ID_USB_INTERFACE_NUM}=="01",SYMLINK+="MMWAVE1843AOP_LX_DATA" , MODE="0666"
	SUBSYSTEM=="tty",ATTRS{serial}=="00E24533", ENV{ID_USB_INTERFACE_NUM}=="00",SYMLINK+="MMWAVE1843AOP_BACK_COMMAND" , MODE="0666"
	SUBSYSTEM=="tty",ATTRS{serial}=="00E24533", ENV{ID_USB_INTERFACE_NUM}=="01",SYMLINK+="MMWAVE1843AOP_BACK_DATA" , MODE="0666"

	N.B.: These lines are valid for actually mounted mmWave Sensors on Koala Robot. If you have new mmWave Sensor, you have to change Serial Number
		according to it. 
```
- Reboot to apply changes:
```sh
	$ sudo reboot
```
- Open one shell for any sensor thet you could active
- Digit one command for shell in ssh mode:
```sh
	$ roslaunch ti_mmwave_rospkg sensors_front.launch
	$ roslaunch ti_mmwave_rospkg sensors_rx.launch
	$ roslaunch ti_mmwave_rospkg sensors_lx.launch
	$ roslaunch ti_mmwave_rospkg sensors_back.launch
```
- For Rviz Visualization open a shall and digit:
```sh
	~ rosrun rviz rviz 
```
- Open Singlesensor , 3Sensors or 4Sensors configuration or create a new config 
