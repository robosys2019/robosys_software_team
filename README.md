# robosys_software_team
Repo for mapping, path planning, and robot interfacing


### Compiling Publisher
First, to setup the KinectOneStream executable, clone the repository and switch to the correct branch. Today it is the ```carl_dev``` branch.

Follow these simple steps:
```
git clone https://github.com/robosys2019/robosys_software_team.git
cd robosys_software_team/
git checkout carl_dev
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

