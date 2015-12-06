# robo-games

### Coordinating Multiple Neatos:

#### usage:
In order to fully launch a single neato with two Pis and two cameras so that it publishes its position relative to the room using the AR ceiling markers, run `neato_core ar_bringup.launch` with arguments `namespace`, `host`, `host_up`, `receive_port`, and `correction`.

###### `namespace`:
The base namespace for the robot; basic neato robot things will publish to this namespace (odom, LIDAR, forward camera).<br>
Topics related to the upwards facing camera and AR locating will publish to the namespace `\namespace_ar`.

###### `host`:
IP address of main Pi
###### `host_up`:
IP of Pi for upwards facing camera

###### `receive_port`:
First three digits of a port to receive camera images on (should start with 50).  The two images will be sent over ports `<input>1` and `<input>2`.

###### `correction`:
correction for offcenter/tilted camera - this can be found using `ar_locating calibrate_star_pose.py`

#### what it does:

##### Launch two Pis on a neato
We have neatos setup with one Pi controlling the neato and transmitting the image from a forward facing camera.  This Pi does exactly what all of the Pis on single Pi neatos have done all semseter, except that it publishes within a namespace (see below)<br>
A second Pi is only responsible for the upwards facing camera.  This image is sent through the `ar_pose` and `star_center_position.py` programs to give us the location of the neatos relative to the ceiling AR markers.

##### Namespaces
In order to run multiple neatos at once, all of the topics they publish to mush be separated.  We do this by grouping each neato into its own namespace, or two.  Most of the topics are published within the input namespace, but because there are two cameras per neato, the two image topics had to be in separate namespaces, so there is a second namespace for each robot (`<input_namespace>_ar`).  All of the code running in this second namespace is for the AR locating code.<br>
Note: TF frames aren't contained by namespaces, so they have been modified to include "namespace_" at the beginning of all of the robot specific frames.

##### Code Used:
The base code to communicate with the neato, `neato_core neato.py` is run in the base namespace.<br>
One instance of the camera code: `gscam raspi_nodelet_udp.launch` is run in the base namespace for the forwards facing camera (port `<input>1`)<br>
Another instance of the camera code is run in the "_ar" namespace (port port `<input>2`)<br>
The `ar_pose` code to find AR markers (`ar_pose ar_pose_multi.launch`) is launched in the "_ar" namespace.  <br>
And the code to find the robot's position in the room based on the ar markers (`ar_locating star_center_position.py`) is run in the "_ar" namespace.