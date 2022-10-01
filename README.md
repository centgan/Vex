# 2021-2022 Vex tipping point season
Provided is the final code for the 2021-22 Vex season. Odometry was used however not very accurate, tracked the global position and orientation of the bot located in [Sensor.cpp](https://github.com/centgan/Vex/blob/master/src/sub/Sensor.cpp).
Globalized variables placed in [Global.cpp](https://github.com/centgan/Vex/blob/master/src/sub/Global.cpp) such as defining the motors etc. Main code such as drive functions and PID functions found in 
[Drive.cpp](https://github.com/centgan/Vex/blob/master/src/sub/Drive.cpp). Autos in [Auto.cpp](https://github.com/centgan/Vex/blob/master/src/sub/Auto.cpp)
no skills run was made for this year as motion profiling was never achieved. Glimpses of path following can be found in Drive.cpp and by all means take it and try to complete it. 

# Advice
For those new to Vex and just so happened to stumbly upon this code, this isn't the best code this is some pretty shit stuff that was slapped together 
pretty sloppily I highly recommend you to look on Vex forums to find others code. If you don't plan on creating your own motion algorithms or don't even know what motion algorithms are use [Okapi](https://okapilib.github.io/OkapiLib/index.html) as
there's no point in making yourself struggle with this stuff especially in the autos. If you plan on making your own motion algorithms look into combining pure pursuit, bezier curves and PID altogether to give you, in my 
opinion, best motion algorithm. Also from what I've found use PI and not D because D is used to reduce the change in your error. Making 3 different PID functions might also be the best way where you have one PID function for a large error, one for a mid error and one for a small error.
