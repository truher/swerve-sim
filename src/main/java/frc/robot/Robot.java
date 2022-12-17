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
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    m_rotationPositionError.set(m_rotationController.getPositionError());
    m_rotationVelocityError.set(m_rotationController.getVelocityError());
    m_rotationSetpointPosition.set(m_rotationController.getSetpoint().position);
    m_rotationSetpointVelocity.set(m_rotationController.getSetpoint().velocity);


  }

  Command autoc;
  ProfiledPIDController m_rotationController;
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable m_table = inst.getTable("robot");
  private final  DoublePublisher m_rotationSetpointPosition = m_table.getDoubleTopic("rotationSetpointPosition").publish();
  private final  DoublePublisher m_rotationSetpointVelocity = m_table.getDoubleTopic("rotationSetpointVelocity").publish();

  private final  DoublePublisher m_rotationPositionError = m_table.getDoubleTopic("rotationPositionError").publish();
  private final  DoublePublisher m_rotationVelocityError = m_table.getDoubleTopic("rotationVelocityError").publish();


  public void autonomousInit() {
    autoc = auto();
    autoc.schedule();
  }

  public void autonomousExit() {
    if (autoc != null)
      autoc.cancel();
  }


  /**
   * make a zigzag path, imagine these were task stations
   */
  public Command auto() {
    // because we're moving and turning at the same time, max speed needs to be less than actual max
    // to make room for the turning speed
    TrajectoryConfig translationConfig = new TrajectoryConfig(0.5 * Drivetrain.kMaxSpeed, 8.0 * Drivetrain.kMaxSpeed)
        .setKinematics(m_swerve.m_kinematics);
    // starts in the corner, make an S to target 0
    Trajectory target0 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(1, 2),
            new Translation2d(4, 6)),
        new Pose2d(4, 7, new Rotation2d(0.5 * Math.PI)), translationConfig);

    // make an S to target 1
    Trajectory target1 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(4, 7, new Rotation2d(0)),
        List.of(
            new Translation2d(4, 6),
            new Translation2d(8, 2)),
        new Pose2d(8, 1, new Rotation2d( Math.PI)),
        translationConfig);

    // make an S to target 2
    Trajectory target2 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(8, 1, new Rotation2d(0)),
        List.of(
            new Translation2d(7, 2),
            new Translation2d(12, 6)),
        new Pose2d(12, 7, new Rotation2d(0.5 * Math.PI)),
        translationConfig);

    // make an S to target 3
    Trajectory target3 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(12, 7, new Rotation2d(0)),
        List.of(
            new Translation2d(12, 6),
            new Translation2d(15, 2)),
        new Pose2d(15, 1, new Rotation2d(-0.5 * Math.PI)),
        translationConfig);

    // because the controller only looks at rotation at the start and at the end,
    // you can't just concatenate the trajectories, you have to run separate
    // commands.
    // TODO: make a trajectory-like thing that includes rotation; a few other teams
    // have done so.

    PIDController xController = new PIDController(0.5, 0, 0);
    PIDController yController = new PIDController(0.5, 0, 0);
    TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(
        1.0 * Drivetrain.kMaxAngularSpeed, 8.0 * Drivetrain.kMaxAngularSpeed);
    m_rotationController = new ProfiledPIDController(0.3, 0, 0, rotationConstraints);
    SmartDashboard.putData("rotation controler", m_rotationController);

    m_swerve.resetOdometry(target0.getInitialPose()); // TODO: remove this?

    SwerveControllerCommand swerveControllerCommand0 = new SwerveControllerCommand(target0,
        m_swerve::getPose, m_swerve.m_kinematics, xController, yController, m_rotationController,
        m_swerve::setModuleStates, m_swerve);
    SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(target1,
        m_swerve::getPose, m_swerve.m_kinematics, xController, yController, m_rotationController,
        m_swerve::setModuleStates, m_swerve);
    SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(target2,
        m_swerve::getPose, m_swerve.m_kinematics, xController, yController, m_rotationController,
        m_swerve::setModuleStates, m_swerve);
    SwerveControllerCommand swerveControllerCommand3 = new SwerveControllerCommand(target3,
        m_swerve::getPose, m_swerve.m_kinematics, xController, yController, m_rotationController,
        m_swerve::setModuleStates, m_swerve);

    // run the sequence and stop at the end.
    return swerveControllerCommand0.andThen(swerveControllerCommand1).andThen(swerveControllerCommand2)
        .andThen(swerveControllerCommand3).andThen(() -> m_swerve.drive(0, 0, 0, true));
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
