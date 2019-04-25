# robosys_software_team
Repo for mapping, path planning, and robot interfacing

### Requirements

- Ubuntu 18.04
- ROS Melodic
- USB 3.0

### Installing dependencies

First, install ROS Melodic following [this tutorial](http://wiki.ros.org/melodic/Installation/Ubuntu). Then, install the libfreenect2 driver used in [this repository](https://github.com/OpenKinect/libfreenect2) up until the 'set udev rules' part.
After that, make sure you have pip installed. If not, just run ```sudo apt-get install python-pip```. Install seaborn using ```sudo pip install seaborn```. You're good to go!

### Compiling Publisher
First, to setup the KinectOneStream executable, clone the repository and switch to the correct branch. Today it is the ```master``` branch.

Follow these simple steps:
```
git clone https://github.com/robosys2019/robosys_software_team.git
cd robosys_software_team/
git checkout map
cmake .
make
```

To make sure the publisher is working, open another terminal and run the following command:
```
roscore
```

Then, to run the publisher, just use this inside the ```robosys_software_team``` folder:
```
./devel/lib/robosys/KinectOneStream
```

If the Kinect V2 is connected, it should run without errors.

### Running the Subscriber

In order to use the listener.py file to subscribe to the depth image publisher, make sure you compiled the project successfully.

With the publisher already running, just use the command ```python listener.py``` to see the image published by the KinectOneStream. If you want to change the topic it is subscribed to, just change the "/camera/depth_undistorted" from ```rospy.Subscriber``` to one of the topics available on ```rostopic list```.

