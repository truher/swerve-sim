// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;

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

        // observe the controllers

        m_XErrorPub.set(xController.getPositionError());
        m_YErrorPub.set(yController.getPositionError());

    }

    Command autoc;
    ProfiledPIDController m_rotationController;
    PIDController xController;
    PIDController yController;

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable m_table = inst.getTable("robot");
    private final DoublePublisher m_rotationSetpointPosition = m_table.getDoubleTopic("rotationSetpointPosition")
            .publish();
    private final DoublePublisher m_rotationSetpointVelocity = m_table.getDoubleTopic("rotationSetpointVelocity")
            .publish();

    private final DoublePublisher m_rotationPositionError = m_table.getDoubleTopic("rotationPositionError").publish();
    private final DoublePublisher m_rotationVelocityError = m_table.getDoubleTopic("rotationVelocityError").publish();

    // controller errors
    private final DoublePublisher m_XErrorPub = m_table.getDoubleTopic("xError").publish();
    private final DoublePublisher m_YErrorPub = m_table.getDoubleTopic("yError").publish();

    public void autonomousInit() {
        // autoc = auto();
        autoc = circle();
        autoc.schedule();
    }

    public void autonomousExit() {
        if (autoc != null)
            autoc.cancel();
    }

    /**
     * Make a circular path with a single spline
     */
    public Command circle() {
        TrajectoryConfig translationConfig = new TrajectoryConfig(Drivetrain.kMaxSpeed / 2, Drivetrain.kMaxSpeed)
                .setKinematics(m_swerve.m_kinematics);
        Trajectory target0 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(Math.PI / 4)), // start ~diagonally
                List.of( // make three loops
                        new Translation2d(5, 5), // kind of an insertion here
                        new Translation2d(8, 6),
                        new Translation2d(10, 4),
                        new Translation2d(8, 2),
                        new Translation2d(6, 4),
                        new Translation2d(8, 6),
                        new Translation2d(10, 4),
                        new Translation2d(8, 2),
                        new Translation2d(6, 4),
                        new Translation2d(8, 6),
                        new Translation2d(10, 4),
                        new Translation2d(8, 2)),
                new Pose2d(6, 4, new Rotation2d(Math.PI / 2)), // end +y
                translationConfig);
        xController = new PIDController(1.5, 0, 0);
        yController = new PIDController(1.5, 0, 0);
        TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(
                Drivetrain.kMaxAngularSpeed / 2, Drivetrain.kMaxAngularSpeed / 2);
        m_rotationController = new ProfiledPIDController(1.5, 0, 0, rotationConstraints);
        SmartDashboard.putData("rotation controller", m_rotationController);

        SwerveControllerCommand swerveControllerCommand0 = new SwerveControllerCommand(target0,
                m_swerve::getPose, m_swerve.m_kinematics, xController, yController, m_rotationController,
                () -> new Rotation2d(0), // no robot rotation at all
                m_swerve::setModuleStates, m_swerve);

        return swerveControllerCommand0.andThen(() -> m_swerve.drive(0, 0, 0, true));
    }

    /**
     * Make a zigzag path, imagine these were task stations
     */
    public Command auto() {
        TrajectoryConfig translationConfig = new TrajectoryConfig(Drivetrain.kMaxSpeed / 2, Drivetrain.kMaxSpeed)
                .setKinematics(m_swerve.m_kinematics);

        // for now just make straight lines.
        // remember the endpoint poses are not robot poses, they are **spline controls**
        Trajectory target0 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                new ArrayList<Translation2d>(),
                new Pose2d(4, 7, new Rotation2d(Math.PI / 2)), translationConfig);

        Trajectory target1 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(4, 7, new Rotation2d(-Math.PI / 2)),
                new ArrayList<Translation2d>(),
                new Pose2d(8, 1, new Rotation2d(-Math.PI / 2)),
                translationConfig);

        Trajectory target2 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(8, 1, new Rotation2d(Math.PI / 2)),
                new ArrayList<Translation2d>(),
                new Pose2d(12, 7, new Rotation2d(Math.PI / 2)),
                translationConfig);

        Trajectory target3 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(12, 7, new Rotation2d(-Math.PI / 2)),
                new ArrayList<Translation2d>(),
                new Pose2d(15, 1, new Rotation2d(-Math.PI / 2)),
                translationConfig);

        xController = new PIDController(1.5, 0, 0);
        yController = new PIDController(1.5, 0, 0);
        TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(
                Drivetrain.kMaxAngularSpeed / 2, Drivetrain.kMaxAngularSpeed / 2);
        m_rotationController = new ProfiledPIDController(1.5, 0, 0, rotationConstraints);
        SmartDashboard.putData("rotation controler", m_rotationController);

        // if you don't reset the pose, it will kinda do the right thing, trying to get
        // to the correct place no matter where it starts.
        // m_swerve.resetOdometry(target0.getInitialPose());

        // specify the rotations here, otherwise it takes the last spline control,
        // which only makes sense for tank drive.
        SwerveControllerCommand swerveControllerCommand0 = new SwerveControllerCommand(target0,
                m_swerve::getPose, m_swerve.m_kinematics, xController, yController, m_rotationController,
                () -> new Rotation2d(Math.PI / 2),
                m_swerve::setModuleStates, m_swerve);
        SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(target1,
                m_swerve::getPose, m_swerve.m_kinematics, xController, yController, m_rotationController,
                () -> new Rotation2d(-Math.PI / 2),
                m_swerve::setModuleStates, m_swerve);
        SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(target2,
                m_swerve::getPose, m_swerve.m_kinematics, xController, yController, m_rotationController,
                () -> new Rotation2d(Math.PI / 2),
                m_swerve::setModuleStates, m_swerve);
        SwerveControllerCommand swerveControllerCommand3 = new SwerveControllerCommand(target3,
                m_swerve::getPose, m_swerve.m_kinematics, xController, yController, m_rotationController,
                () -> new Rotation2d(-Math.PI / 2),
                m_swerve::setModuleStates, m_swerve);

        // run the sequence and stop at the end. added pauses for some realism, e.g.
        // like the robot is doing something.
        return swerveControllerCommand0.andThen(new WaitCommand(0.1))
                .andThen(swerveControllerCommand1).andThen(new WaitCommand(0.1))
                .andThen(swerveControllerCommand2).andThen(new WaitCommand(0.1))
                .andThen(swerveControllerCommand3).andThen(new WaitCommand(0.1))
                .andThen(() -> m_swerve.drive(0, 0, 0, true));
    }

    @Override
    public void teleopPeriodic() {
        driveWithJoystick(true);
    }

    private void driveWithJoystick(boolean fieldRelative) {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final var xSpeed = -m_xspeedLimiter.calculate(getXSpeedInput1_1()) * Drivetrain.kMaxSpeed;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        final var ySpeed = -m_yspeedLimiter.calculate(getYSpeedInput1_1()) * Drivetrain.kMaxSpeed;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        final var rot = -m_rotLimiter.calculate(getRotSpeedInput1_1()) * Drivetrain.kMaxAngularSpeed;

        m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
    }

    /**
     * Mix in the cube of the input; this feature is generally called "expo"
     * in the RC community even though it's not an exponential function.
     * 
     * @param fraction how much cubic to add, [0,1]
     */
    private double expoInput(double input, double fraction) {
        return (1 - fraction) * input + fraction * input * input * input;
    }

    private double deadband(double input, double threshold) {
        return MathUtil.applyDeadband(input, threshold, Double.MAX_VALUE);
    }

    // more expo works well with less deadband.

    private double getRotSpeedInput1_1() {
        return expoInput(deadband(m_controller.getRightX(), 0.01), 0.5);
    }

    private double getYSpeedInput1_1() {
        return expoInput(deadband(m_controller.getLeftX(), 0.01), 0.5);
    }

    private double getXSpeedInput1_1() {
        return expoInput(deadband(m_controller.getLeftY(), 0.01), 0.5);
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
        // if you forget this scheduler thing then nothing will happen
        CommandScheduler.getInstance().run();
    }

    @Override
    public void testPeriodic() {
        m_swerve.test(getYSpeedInput1_1(), getRotSpeedInput1_1());
    }
}
