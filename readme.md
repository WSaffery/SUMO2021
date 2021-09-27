# BLUEPRINT LAB SUMO HACKATHON CHALLENGE 2021

## BRIEF: 
Design a computer vision solution to autonomously grab a known item of interest using the Reach Bravo 7 manipulator.
The known item is an autonomous underwater vehicle (AUV) that must be secured in order to lift it out of the water. 
The Reach Bravo 7 has been deployed at the surface of the water. It is fitted with a camera fixed to the wrist.
The camera is the input to your system. It must detect the April tag secured to the AUV and use this to grab the hook located next to it. 
The output to your system is the set of joint positions for the manipulator. 
You must perform analysis of the wrist camera stream to locate and grab the AUV's hook.

There are three rounds: 
1. The AUV is stationary.

2. The AUV moves slowly in one axis.

3. The AUV moves randomly in three axes. 

You have a maximum of 1 minute to complete each round.
Your score is calculated as the sum of the time to complete each round.
The lowest score wins. 

## INSTALLATION
1. Clone this repository onto a Windows system.
2. Ensure you have Python 3.7+ installed.
3. Run ./setup.ps1 script to create the virtual environment and install all dependencies.
4. Add your code to the user.py file. 

