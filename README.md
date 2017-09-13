# This is code for FRC team 4910, East Cobb Robotics, 2017 practice and competition robot.
This repository will likely be updated for the entire competition season. The code works with both our competition and practice bots, such that you switch a boolean in RobotMap.java.

You may also notice that we implement code that pushes a ton of data to a CSV file every iteration. We obviously plan on getting rid of that for compbot, and its only use is to provide testing data for us (using a python script to graph data). Many such functions are just for testing purposes.

The libraries needed to run this code include:

SF2 (Sensor Fusion Framework) - Direct link: http://www.kauailabs.com/public_files/sf2/sf2.zip - Chief Delphi page: https://www.chiefdelphi.com/forums/showthread.php?t=152956

NavX (AHRS 9 axis sensor) - Direct link: http://www.kauailabs.com/public_files/navx-mxp/navx-mxp.zip - Chief Delphi page: https://www.chiefdelphi.com/forums/showthread.php?t=131859 (also mentioned above)

opencsv-3.8 (CSV helper) - Sourceforge link: https://sourceforge.net/projects/opencsv/files/opencsv/3.8/

CTRLib (CANTalon libraries) - Indirect link: http://www.ctr-electronics.com/hro.html#product_tabs_technical_resources (under latest installer)

Both the SF2 and NavX libraries are automatically added to the wpilib/user/libs folder on installation. The CANTalon libraries are so large because CTR packed the Hero board libraries in with the rest of the installer.

There's also a pathfinder library but it isn't implemented, and likely won't be in the future. The SF2 library also is not implemented

There is a plotter tool to plot data from a csv file (documented in Robot.java) found in the tools folder. The syntax can be found within the file itself. If you encounter an error when running that script along the lines of "...has too many rows!", open the file and clear the last row. This is due to the robot being disabled while writing to it, which causes the data on the last row to be unfinished.

The only libraries you could get away with not installing and not have any errors are SF2, pathfinder, and possibly NavX.

# Completed and working functions as of directly after ~~states~~ worlds
+ Climbing
+ Intaking
+ Agitating and shooting
  + I'm not 100% satisfied with the class, as it isn't consistant with the rest of the program.
+ PID for the shooter
+ PID for driving to positions and headings, but not velocities
+ Working vision tracking code, but could use more tuning (too late now)
+ Working left (right turn) autonomous (but not middle or right)
+ Gate and shifter gear functionality
+ Left position only autonomous

# Things left to do
+ Add a DoubleSolenoid.Value object in RobotMap, as Value.Reverse isn't as helpful as RobotMap.GatesOpen
+ Add more if(button) statements in teleopPeriodic() as a better solution to the Shooter code inconsistencies
+ Figure out why I even needed to do regression in the vision tracking code, and then fix whatever issue there is
+ Add way more comments
+ Find a proper way to implement something like if(button.returnIsPressedAndThenInvert()) so I don't have to add a while(isPressed) statement that eats up precious time
  + Possibly add a buttonHandler class to handle this
+ Figure out what I did wrong with the compressor handler in teleop (it cycles between on and off when any of those booleans are true)
  + I know exactly what I did wrong, but I need to find a way to properly implement this.
+ Remove dead code
+ Allow for PID.controller.maxOutputs() to be set inside Path using a setMaxes() method, and then set it back to default after iterate() (some movements need to be more careful)
+ Find a way to tell which RIO I'm talking to, whether it's practice or comp bot, without having to manually switch a boolean (I keep forgetting to switch it sometimes)
  + Possibly exploit the fact that I can connect to a camera on practice bot with the static IP 10.49.10.17 and it's 10.49.10.18 on compbot
+ Add functionality to the plotter.py program to pull data directly from the RIO with no WinSCP/puTTY middleman.
+ Rename repo to 2017CompetitionBot and deal with every issue that will surely come up (for my successor to handle)

# Extra notes
Dead code where it basically says if(true) or if(isCombot && false), with one exception, is due to the fact parts kept changing on compbot so differences that used to exist no longer do. This really confused me with the shooter.
The one exception is that inside Path, I was planning on adding dead reckoning functionality but I abandoned it for the sake of time.

If you have any questions at all, even if it hardly pertains to the code, you can contact me at jbc218@gmail.com. Even if it's been 5 years since my last commit and everythings deprecated, I'm willing to help.

As for the near future, there will be 2 remaining commits for this project. First one will be "Robot code as it was after states" and the other one will be "Cleaned up code". Anything after those two will be done by my successor, where he'll add all that other functionality listed above.
