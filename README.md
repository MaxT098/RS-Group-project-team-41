Week 8 

Conclusion and notes from last week 

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

Page Break
 

Week 9 

Notes: 

Constructed the mainframe, which is a state machine 

Need to compare correction using IMU with correction using heading based on odometry 

Need to figure out how to stop the robot 

Structure of the IMU-corrected model: 

Text BoxText BoxText BoxShapeShapeText Box 

 

Structure of the odometry-corrected model: 

Text BoxShapeShapeText BoxText BoxText Box 

 

Feedback from the group tutor 

Hi, Team 41, 

Here are your supervision meeting notes from Week 9 with Gabriella Miles: 

Contribution of team members (splitting a budget of £600 based on contribution): 

Equal contribution - £300: £300. 

Main concerns/progress/absence of team members: 

N/A 

Goals set looking forwards: 

The focus of this week should be implementation - aim to get something running practically on the robot. 

1. Implement IMU-based return-to-line behaviour  

2. Work through the labsheet for PID controllers 

3. Work through the labsheet for EEPROM 

Any additional comments: 

N/A 

 

Page Break
 

Week 10 

Notes: 

Finish by today: 

Finish and debug PID code, implement on romi (Max) 

Test all the sample EEPROM code – needs to be working (Max) 

Finish by tomorrow: 

Modify sample EEPROM code, make custom version 

Assume the romi travels the whole 2m in 10 seconds, collecting coordinates every 0.5 second, that means 20 coordinates will be saved. We will need to save a 2 x 20 map in the EEPROM. Might want to make a state when the grid is filled up, then move all the data to the EEPROM. (Max) (individual) 

Working code: save all the data in a 2x20 array (need to work on writing x-displacement and heading angle into the array, but printing static data from array has been tested to work) 

X position 

Y position 

X position 0.5 sec later 

Y position 0.5 sec later 

And so on… 

The 0.5 sec time interval is coded in the  

 

Gyro loop 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

Calibrate line sensor on test surfaces: 

Baking paper, cardboard, black tape; reflectance numbers from the line sensor (Viswas) (individual) 

Surfaces: 

Baking paper (smooth) + cardboard (rough), tyres removed. 

Baking paper (lubricated with oil or other lubricants, smooth) + cardboard (rough), tyres on. 

Baking paper (lubricated with oil or other lubricants, smooth) + cardboard (rough), tyres off. 

Text BoxShapeShapeShapeShape 

 

 

Finish by Thursday: 

Add a low-pass filter to the IMU signal (Max) 

Find ways to translate x-acceleration (and possibly angular acceleration) to positional data? 

X-acceleration (X-axis) * time interval 0.5 sec (y-axis), because we just need to know its horizontal displacement to check if it is deviating. (Just assume its vertical speed to be constant) (Probably don’t need angular acceleration as input, because it does not seem possible for the romi to go left without spinning or spinning right) (Viswas) 

 
s=ut+ 12at2
s=ut+ 12at2
 
 

Based on the above equation,  

u = at, we assume the acceleration remained as a constant in that 0.5 sec 

s=1.5at2
s=1.5at2
 
 

Good ol’ camera system to capture bird’s eye view of the robot. Map using a scale. 

Make sure the IMU correction system is working fine, calibration (Max and Viswas) 

How to stop making corrections based on previous correction? Once it displaces left/right set that as a correction direction, then only produce opposite action. E.g. once displaced left, only take further left displacements as input for correction. (Viswas) 

Need to add a state where facing forward && not on black line = state finished 

Positional data collection should always be on in the background/ in all states 

 

Finish by Friday: 

Combine IMU code and Line following code and test performance on arena. Obtain data. (Together in hackspace) 

 

Finish by Monday (having worked individually on Saturday and Sunday): 

Odometry correction code/ heading angle correction code (angular acceleration * time elapsed to predict heading angle at next time interval) and successful implementation, obtain data. 

 

Tuesday onwards: 

We should have all the data needed, writing commences. Split workload again. 

 

Equity share for week 10: 

£450:£150 

 

Feedback from week 10: 

Hi Team 41, 

 

Here are your supervision meeting notes from Week 10 with Gabriella Miles: 

 

Contribution of team members (splitting a budget of £600 based on contribution): 

£450 : £150  

 

Main concerns/progress/absence of team members: 

Team dynamics concerns: Max (co21442) is struggling to meet with his team member consistently (Viswas has now missed two supervision slots - although one was due to illness). This has resulted in Max shouldering a large amount of the programming work.  

 

Technical concerns: tuning the PID controller. The I term is very erratic.  

 

Goals set looking forwards: 

1. Continue with PID and EEPROM work 

2. Complete testing to see whether return-to-line behaviour is consistent 

3. Convert IMU data to positional data 

 

New objective: Line following when it is on line. compare correction by x-acceleration and heading angle when off line.  

 

Objective for 5/12/2021: 

Complete PID for line following, copy that to IMU adjustment system, taking x-acceleration and heading angle as input. Heading angle at any given time = angular acceleration x 0.5 sec (or whatever sampling interval), need to assume acceleration remained constant in that 0.5 sec. 

Objective for 7/12/2021:
Finish PID!!!! Luckily completed state machine but it is behaving strangely.
 

Any additional comments: 
