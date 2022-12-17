// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final Drivetrain m_swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  @Override
  public void autonomousPeriodic() {
    // driveWithJoystick(false);
    // m_swerve.updateOdometry();

    // System.out.printf("scheduled %s\n", autoc.isScheduled()?"yes":"no");

  }

  Command autoc;

  public void autonomousInit() {
    autoc = auto();
    autoc.schedule();
  }

  public void autonomousExit() {
    if (autoc != null)
      autoc.cancel();
  }

  public Command auto() {
    TrajectoryConfig config = new TrajectoryConfig(1, 1).setKinematics(m_swerve.m_kinematics);
    // make a square, kinda
    Trajectory e = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(6, 6), // first go to the upper left
            new Translation2d(10, 6), // then upper right
            new Translation2d(10, 2), // lower right
            new Translation2d(6, 2) // lower left
        ),
        new Pose2d(6, 6, new Rotation2d(0)),  // works with no rotation.  rotation is not working well.
         config);
    ProfiledPIDController c = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(1, 2));
    SwerveControllerCommand s = new SwerveControllerCommand(e, m_swerve::getPose, m_swerve.m_kinematics,
        new PIDController(0.1, 0, 0), new PIDController(0.1, 0, 0), c, m_swerve::setModuleStates, m_swerve);
    System.out.printf("trajectory %s\n", e);
    m_swerve.resetOdometry(e.getInitialPose());
    return s.andThen(() -> m_swerve.drive(0, 0, 0, true));
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed = -m_xspeedLimiter.calculate(m_controller.getLeftY()) * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = -m_yspeedLimiter.calculate(m_controller.getLeftX()) * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = -m_rotLimiter.calculate(m_controller.getRightX()) * Drivetrain.kMaxAngularSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }

  @Override
  public void simulationInit() {
    m_swerve.simulationInit();
  }

  @Override
  public void simulationPeriodic() {
    m_swerve.simulationPeriodic();
    m_swerve.updateOdometry();
  }

  @Override
  public void robotPeriodic() {
    // m_swerve.updateOdometry();
    // my god if you forget this you are doomed
    CommandScheduler.getInstance().run();
  }

  @Override
  public void testPeriodic() {
    m_swerve.test(m_controller.getLeftX(), m_controller.getRightX());
  }
}
