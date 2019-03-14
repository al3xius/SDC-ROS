# SDC-ROS


Setup:

Setup a Linux machine, we recommed using Ubuntu 16.04.
Don't forget to install Python and Pip:
sudo apt install python-pip python-dev python-virtualenv

Install ROS Kinetic according to their tutorial http://wiki.ros.org/kinetic/Installation.

Install all ros extentions packages:
sudo apt install ros-kinetic-joy ros-kinetic-rosserial-arduino ros-kinetic-rosserial ros-kinetic-rosserial-server ros-kinetic-rplidar-ros ros-kinetic-cv-bridge ros-kinetic-cv-camera ros-kinetic-usb_cam ros-kinetic-openni2-launch


Install required Python Packages:
sudo pip install opencv-contrib-python
sudo pip install scipy
sudo pip install ipdb
sudo pip install simple-pid
sudo pip install futures requests
sudo pip install python3-yaml
sudo pip install kivy-garden
sudo pip3 install --user rospkg catkin_pkg simple-pid
sudo pip3 install adafruit-circuitpython-gps

Install Kivy:
sudo add-apt-repository ppa:kivy-team/kivy
sudo apt update
sudo apt install python-kivy python3-kivy
sudo garden install mapview

Install Tensorflow preferably use their own Guide https://www.tensorflow.org/install/pip.
If their guide is not working for you try:
sudo pip install tensorflow or
virtualenv --system-site-packages ~/tensorflow
source ~/tensorflow/bin/activate
easy_install -U pip
pip install --upgrade tensorflow

Clone the repository:
cd ~/catkin_ws
git clone https://github.com/al3xius/SDC-ROS

Rename the folder to src:
sudo mv SDC-ROS src

Clone dependend repoyitorys:
cd ~/catkin_ws/src
git clone https://github.com/Kukanani/vision_msgs.git

Run sdccommands:
cd ~/catkin_ws/src/commands
sudo bash sdccommands

Buld:
sdcmake

Try to run:
sdclaunch

If you get any error messages telling you to install missing Packages, try to do so.