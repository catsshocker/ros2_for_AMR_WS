# ros2_for_mymobile_robot

[![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue)]()

ros2 on VMX and someone still not give me data to update vmx2 so i also use pid on rpi  

this code is for world skill taiwan mobile robot race just for testing it can run on my mechine :)

## motor speed 
moter speed use tick/s(enc/s)  
on vmx enc -> 1464tick/1round  
on knr enc -> 4096tick/1round  

## file and function

### joyControl

/joy to /cmd_vel for ⁠Omnidirectional robot 

### DCspeed

/cmd_vel to motor speed   you can change this file to use three or four wheel robot

### motion

publish /cmd_vel for some move 

### Motor Manger

manage motor like pid (encoder feed back), EMS, and updown rail -> just for DC motor 