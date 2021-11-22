# RS-Group-project-team-41
IMU-based electronic stability control for the 3pi+ Romi

Here lies our meeting notes and objectives:

**Week 8:**
Objective for project: first step, IMU-assisted line following challenge, where line sensors serve as the initial reference point for it to follow a straight line, if the line sensors don't detect a line, then use IMU readings to adjust motor output and rotation to make it detect lines again -> measures the time when a line is not detected -> normal and slippery surfaces 

Alternative objective: second step, if that works, then move onto IMU being the sole reference point for the Romi to travel in a straight line (in reality you don’t have a straight line to guide the car!), the black line, therefore, only serves as a measurement of whether the Romi is travelling in a straight line, detection of a black line means it is travelling in a straight line, failing to detect one means that it is travelling off a straight line. 

Notes: Started to play around a bit with the IMU readings with guidance from the supplementary lab sheet, default sensitivity is good for gyro but maybe not for acceleration (need to turn up the sensitivity), need to adjust based on real testing on tracks later. Need to calibrate the gyro and acceleration readings for stationary range of numbers. Can we actually use the lsm6 library? (Yes we can!) Integrate the angular acceleration to find the heading angle (do we need to?). 

Funny thing is that the acceleration readings are positive no matter you move it left or right, so may need gyro readings to know which way it is really turning. 

Personally, I didn’t read many papers on other IMU usage and ESP implementation… but I will read them during the rest of the week. How do we keep our topic original? Because there are only so many components of the robots that we can work on? The rolling slippage sample report. When will we have feedback on the formative assessment? 

Feedbacks from the group tutor 

Here are your supervision meeting notes from Week 8 with Gabriella Miles: 
  
Contribution of team members (splitting a budget of £600 based on contribution): 
£300: £300 
  
Main concerns/progress/absence of team members: 
No major concerns. One team member absent: iq21685.  
 
Goals set looking forwards: 
Goals for this week:  
1. Integrate the IMU, gyro, and accelerometer into the line sensor challenge. 
2. Look through the PID and odometry labsheets 
  
Any additional comments: 
N/A 

Goals for next week: 

Next step for this week is to modify the line following challenge to implement a detection of the IMU to it (Objective #1), read through the odometry and PID sections, calibrate the IMU, find the slippery materials and test it and adjust the required sensitivity, write up the introduction part, start up a teams/drive folder for collaboration, takes minutes of the meetings.  

Objectives for coming weeks are: Logging the data, calculate the area (how long it has deviated time x distance integral), completing the remaining tests and write up the rest of the report. 
