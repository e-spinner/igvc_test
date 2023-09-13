This is a “quick” guide to help set up ROS on your computer. If you have any questions, feel free to ask, and additionally, if you make any discoveries or have additional steps add them. This first section will details setting up Ubuntu through virtualization for Windows or Mac. Skip to the second section if you have Linux already. 

# Setting up a virtual machine 

Make sure that you have Python 3.11.5 installed as this is the required version for several programs we will be using. 

The following links to instructions for virtualization of Linux once they tell you to install ROS, come back to this document. 

For simplicity’s sake, I recommend that you set the username and password of your Linux installation to dev, or something short, because in the Linux terminal it always displays your computer name and username before every line. For this reason, I would also suggest making your virtual computer name something short. 

Go here and follow these instructions for windows machines. (You may need to get certain versions of Microsoft Visual C++ Redistributable) 

Here is a tutorial from the same website for MAC machines. It might work, and I think VirtualBox also works on MAC machines. 

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

After this restart your machine, once you do follow the steps in this documentation to install ROS, make sure to install the Desktop version and the optional development tools. The base version is not helpful. 

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
Go to this site to set up SSH with git, it is kind of confusing I believe in you. Sadly, you should not use GitHub desktop with ROS. 

Navigate to the software store and download vs code which is just called code 

Here you can install gazebo-classic which is the simulation software, and with the following command you can install the necessary ROS – Gazebo pkgs  
```bash
sudo apt install ros-humble-gazebo-ros-pkgs 
```
Also install more needed programs with the following command: 
```bash
sudo apt install ros-humble-xacro ros-humble-joint-state-publisher-gui 

sudo apt install ros-humble-image-transport-plugins 
```
 

 

 

 