# SDC-ROS


## Setup:

Setup a Linux machine, we recommend using Ubuntu 16.04.

Don't forget to install Python and Pip:

```sudo apt install python-pip python-dev python-virtualenv```

### 1. Install ROS Kinetic according to their tutorial http://wiki.ros.org/kinetic/Installation.

### 2. Install all ros extention packages:

```sudo apt install ros-kinetic-joy ros-kinetic-rosserial-arduino ros-kinetic-rosserial ros-kinetic-rosserial-server ros-kinetic-rplidar-ros ros-kinetic-cv-bridge ros-kinetic-cv-camera ros-kinetic-usb_cam ros-kinetic-openni2-launch```


### 3. Install the required Python Packages:

```
sudo pip install opencv-contrib-python
sudo pip install scipy
sudo pip install ipdb
sudo pip install simple-pid
sudo pip install futures requests
sudo pip install python3-yaml
sudo pip install kivy-garden
sudo pip3 install --user rospkg catkin_pkg simple-pid
sudo pip3 install adafruit-circuitpython-gps
```


### 4. Install Kivy:

```
sudo add-apt-repository ppa:kivy-team/kivy
sudo apt update
sudo apt install python-kivy python3-kivy
sudo garden install mapview
```

### 5. Install Tensorflow preferably use their own Guide https://www.tensorflow.org/install/pip.
    
If their guide is not working for you try:

```sudo pip install tensorflow```

or

```
virtualenv --system-site-packages ~/tensorflow
source ~/tensorflow/bin/activate
easy_install -U pip
pip install --upgrade tensorflow
```

### 6. Clone the repository:
    
```
cd ~/catkin_ws
git clone https://github.com/al3xius/SDC-ROS.git
```

Rename the folder to src:

```sudo mv SDC-ROS src```

### 7. Clone dependend repoyitorys:
    
```
cd ~/catkin_ws/src
git clone https://github.com/Kukanani/vision_msgs.git
```

### 8. Run sdccommands:
    
```
cd ~/catkin_ws/src/commands
sudo bash sdccommands
```

### 9. Buld:
    
```sdcmake```

### 10. Try to run:
    
```sdclaunch```

If you get any error messages telling you to install missing Packages, try to do so.
If not all required hardware is connected you will get error messages.


## Make USB Devices static:

Get unique ID with `udevadm info --name=/dev/ttyUSBxx --attribute-walk`.

edit rules.d `sudo nano /etc/udev/rules.d/99-usb-serial.rules`

paste 
```
KERNEL=="ttyUSB*", ATTRS{idVendor}=="xxxx", SYMLINK+="arudino0"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="xxxx", SYMLINK+="arudino1" 
KERNEL=="ttyUSB*", ATTRS{idVendor}=="xxxx", SYMLINK+="lidar0"
```

reload rules ```udevadm control --reload-rules```