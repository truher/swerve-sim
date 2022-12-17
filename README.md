# Swerve Sim

This is the simplest possible simulation of a swerve drive, using forward kinematics based on the feedforwards of each motor, with no
attention to momentum, friction, etc. If you're interested in a more realistic approach, check out what Chris Gerth
has going with 6391, [here](https://github.com/6391-Ursuline-Bearbotics/BearSwerve/blob/master/src/main/java/frc/wpiClasses/SwerveModuleSim.java).

If you load the project and click "Simulate Robot Code" from the WPI menu, and choose "sim gui," you'll get the field with the
robot starting in the lower-left corner, and you can use the keyboard or a real controller to roam around.  The autonomous
mode also works, making a sort of circle in the middle of the field.

Caveats:

* There's also something wrong with the rotational control in autonomous: it oscillates; maybe the encoder is backwards or something.

I'll try to fix the issues, hopefully by Christmas.

For now, it might be good enough to play around with waypoint commands and/or semi-auto (i.e. trajectories with manual overlays).  
