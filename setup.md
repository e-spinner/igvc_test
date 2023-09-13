This is a “quick” guide to help set up ROS on your computer. If you have any questions, feel free to ask, and additionally, if you make any discoveries or have additional steps add them. This first section will details setting up Ubuntu through virtualization for Windows or Mac. Skip to the second section if you have Linux already. 

# Setting up a virtual machine 

Make sure that you have [Python 3.11.5](https://www.python.org/downloads/release/python-31013/) installed as this is the required version for several programs we will be using. 

The following links to instructions for virtualization of Linux once they tell you to install ROS, come back to this document. 

For simplicity’s sake, I recommend that you set the username and password of your Linux installation to dev, or something short, because in the Linux terminal it always displays your computer name and username before every line. For this reason, I would also suggest making your virtual computer name something short. 

Go [here](https://www.theroboticsspace.com/blog/How-To-Install-ROS-2-in-Ubuntu-22-04-VM-On-Windows/) and follow these instructions for windows machines. (You may need to get certain versions of Microsoft Visual C++ Redistributable) 

[Here](https://www.theroboticsspace.com/blog/How-To-Install-ROS-2-in-Ubuntu-22-04-On-M1-Mac/) is a tutorial from the same website for MAC machines. It might work, and I think VirtualBox also works on MAC machines. 

In VirtualBox's settings, go to general > Advanced > Shared Clipboard and set it to bidirectional so that you can copy and paste between machines. 

# Installing ROS onto Linux 

Run the following command in terminal to check if you have sudo privileges.  

```bash
sudo -l 
```

If that outputs anything other than: 
```bash
User {username} may run the following commands on Ubuntu: 

    (ALL : ALL) ALL 
```
You can run the following commands to fix that... 
```bash
su root 

nano /etc/sudoers 
```
This will allow you to edit the file in the terminal, Now you can add  

```bash
{username} ALL=(ALL:ALL) ALL 
```

under
```bash
#User privilege specification 
 
root     ALL=(ALL:ALL) ALL 
 ```
TO save your changes in this editor hit ctrl+o, enter, and then ctrl+x.

After this restart your machine, once you do follow the steps in this [documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) to install ROS, make sure to install the Desktop version and the optional development tools. The base version is not helpful. 

Once you have successfully done the above run the following code to make it, so Linux always knows about ROS: 

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc 
```
Now restart your machine... 

# Installing Additional tools 

Colcon is the program that is used to build ROS packages. To install it run the following: 
```bash
sudo apt install python3-colcon-common-extensions 
```
Go to [this site](https://docs.github.com/en/authentication/connecting-to-github-with-ssh) to set up SSH with git, it is kind of confusing I believe in you. Sadly, you should not use GitHub desktop with ROS. 

Navigate to the software store and download vs code which is just called code 

[Here](https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install) you can install gazebo-classic which is the simulation software, and with the following command you can install the necessary ROS – Gazebo pkgs  
```bash
sudo apt install ros-humble-gazebo-ros-pkgs 
```
Also install more needed programs with the following command: 
```bash
sudo apt install ros-humble-xacro ros-humble-joint-state-publisher-gui 
sudo apt install ros-humble-image-transport-plugins 
```
finally, run the following to verify that you are upto date, and restart your machine.
```bash
sudo apt update && sudo apt upgrade
```
# Getting started with ROS

ROS runs inside of a workspace named in the format of name_ws. I use igvc_ws for the main code in this project and test_ws for the test code. to create this directory and enter it's space in the terminal use the following commands. Graphically this folder can be hond in the home directory on linux.

```bash
mkdir -p ~/name_ws/src

cd name_ws/src
```

Inside of thies src folder is where each ROS package will live. to put them there run a git clone with a SSH link compied from the project you are cloning or run the ros2 pkg creater. These are both detailed below

```bash 
git clone git@github.com:e-spinner/igvc_test.git

# ^^^ cloning or VVV creating

ros2 pkg create --build-type ament_cmake <pkg_name>
```

now navigate back a level using ```cd ..``` and run the following comand to build the project. You will need to run these next few commands every time you make any major changes to the pakage and you want to run it.

```bash
# this command builds the ws, you must cd into the top level of the ws to run it. if you have multiple packages it defaults to building upto 16 at once, and if your machine can't handle that add --executor-sequential to the end
colcon build --symlink-install

# now that you have built your workspace you have to tell linux where to find your programs. you must do this evey time you open a brand new terminal to run a process from this workspace.
source install/setup.bash
```

This is the end. once again feel free to add any advice you discover along the way, or a few fun pictures, if you dare.