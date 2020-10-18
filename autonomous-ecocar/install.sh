#!/bin/bash
# Copy this file to the root of the PC and run:
# chmod +x install.sh
# ./install.sh

# Update Ubuntu and other software
sudo apt-get update
sudo apt-get upgrade -y

# Disable password login at boot
# Could not make this work in GDM3

# Install lightdm
echo ""
echo "Installing lightdm display manager. When prompted, choose lightdm."
sudo apt-get install -y lightdm
# Choose lightdm as default (manual operation)
sudo bash -c 'echo "[SeatDefaults]
autologin-user=dynamo
autologin-user-timeout=0" > /etc/lightdm/lightdm.conf'

# Tried to make gdm3 work with no luck.
#sudo sed -i 's/#WaylandEnable=false/WaylandEnable=true/g' /etc/gdm3/custom.conf
#sudo sed -i 's/WaylandEnable=false/WaylandEnable=true/g' /etc/gdm3/custom.conf
#gsettings set org.gnome.desktop.lockdown disable-lock-screen true

# (Optional) Uninstall unneeded software using Ubuntu Software or apt-get uninstall
# Not needed if minimal base install is selected during Ubuntu install
# amazon activity log manager aislerot_solitaire imagemagick mahjong mines thunderbird sudoku shotwell

# (Optional) update kernel
#..


# Install ROS Melodic Morenia
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update

sudo apt -y install ros-melodic-desktop-full

sudo rosdep init
rosdep update

# Install additional ROS packages (need some for camera? what else?)
sudo apt-get -y install ros-melodic-robot-upstart 
# ros-kinetic-usb-cam
# 

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get -y install python-rosinstall python-rosinstall-generator python-wstool build-essential

# Install nice to have utilities
# gitkraken, chromium, teamviewer, CLion, Roboware Studio
sudo apt-get -y install chromium-browser
sudo snap install gitkraken
# Cannot find Roboware, we go with VSCode
sudo snap install --classic code

mkdir tmp
cd tmp
# Teamviewer
wget https://download.teamviewer.com/download/linux/teamviewer_amd64.deb
sudo apt -y install ./teamviewer_amd64.deb

cd ~
rm -rf tmp

# Find password in docx file
# Clone Autonomous Ecocar repository (input password)
git init
git config credential.helper store # enables saving of password
git remote add origin https://DTUDynamo@bitbucket.org/dtucar/autonomous-ecocar.git
git pull origin master
git branch --set-upstream-to=origin/master master

git config --global user.email "dynamo@dtucar.com"
git config --global user.name "Dynamo"
git pull

# Set to executable
chmod +x ~/Scripts/*.sh
chmod +x ~/Desktop/*.desktop

# Install dependencies (these might already be installed)
sudo apt-get install autoconf libusb-1.0.0-dev 

# Serial (required anymore?)
#cd ~/Source/Linux/serial
#make && sudo make install

# libserialport
cd ~/Source/Linux/libserialport
./autogen.sh
./configure
make && sudo make install

# Serial tools
sudo cp ~/Source/Linux/SerialTools/* /usr/local/include

# libphidget needed for control of steering stepper
cd ~/Source/Linux/libphidget
autoreconf -f -i #(only needed if newer version than 1.14 of aclocal is installed)
./configure
make && sudo make install

# MyQueue (still needed?)
cd ~/Source/Linux/MyQueue
cmake ./
make && sudo make install

# Velodyne driver
cd ~/catkin_ws/src/velodyne/
rosdep install --from-paths ./ --ignore-src --rosdistro melodic -y

# Piksi SwitfNav
git clone --recurse-submodules https://github.com/swift-nav/libsbp.git ~/SwiftNav/libsbp
cd ~/SwiftNav
sudo apt-get install build-essential pkg-config cmake
cd libsbp/c/
mkdir build
cd build
cmake ../
make
sudo make install

# Build ROS packages
source /opt/ros/melodic/setup.bash
cd ~/catkin_ws/
#catkin_make clean
#rm -rf devel
#rm -rf install
#rm -rf build
catkin_make

echo "source /home/dynamo/catkin_ws/devel/setup.bash" >> ~/.bashrc
source devel/setup.bash

# Install for starting ROS and dynamo_helper at boot
# https://docs.ros.org/melodic/api/robot_upstart/html/
rosrun robot_upstart install dynamo/launch/base.launch
sudo systemctl daemon-reload && sudo systemctl start dynamo

# dynamo_helper is started from an autostart script

# What about USB permissions?
