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
2. Ensure you have Python 3.7+ installed (and your PATH environment variables set appropriately).
3. Ensure [Microsoft C++ Build Tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/) version 14.0 or greater is installed on your system.
4. Run ./setup.ps1 script in a PowerShell terminal to create the virtual environment and install all dependencies.
5. Add your code to the user.py file. 

## RUNNING THE SIMULATION
1. Ensure your virtual environment is activated (you've either run **setup.ps1** in the same terminal session, or you've activated it yourself with `.\env\Scripts\activate`).
2. Run `python main.py --round 1` to start the simulation. The *round* argument can be 1, 2 or 3.

## RECOMMENDED READING
- [**Robotics, Vision and Control**. *Fundamental algorithms in MATLAB: Second Edition*](https://petercorke.com/rvc/home/)