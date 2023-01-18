# Swerve Sim

<video src="https://user-images.githubusercontent.com/113283/208272266-f540216a-5384-438a-8fc9-4e9412928481.mp4"></video>

This is the simplest possible simulation of a swerve drive, using forward kinematics based on the feedforwards of each motor, with no
attention to momentum, friction, etc. If you're interested in a more realistic approach,
check out [Chris Gerth's PR](https://github.com/wpilibsuite/allwpilib/pull/3374) and [6391's BearSwerve](https://github.com/6391-Ursuline-Bearbotics/BearSwerve/blob/master/src/main/java/frc/wpiClasses/SwerveModuleSim.java).

If you load the project and click "Simulate Robot Code" from the WPI menu, and choose "sim gui," you'll get the field with the
robot starting in the lower-left corner, and you can use the keyboard or a real controller to roam around.  The autonomous
mode also works, making a zig-zag down the field.

For now, it might be good enough to play around with waypoint commands and/or semi-auto (i.e. trajectories with manual overlays).  
