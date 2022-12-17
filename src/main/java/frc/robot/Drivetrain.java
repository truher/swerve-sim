// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.CallbackStore;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  /** 3 m/s */
  public static final double kMaxSpeed = 6.0;
  /** pi rad/s */
  public static final double kMaxAngularSpeed = 15 * Math.PI;

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModule m_frontLeft = new SwerveModule("FrontLeft", 1, 2, 0, 1, 2, 3);
  private final SwerveModule m_frontRight = new SwerveModule("FrontRight", 3, 4, 4, 5, 6, 7);
  private final SwerveModule m_backLeft = new SwerveModule("BackLeft", 5, 6, 8, 9, 10, 11);
  private final SwerveModule m_backRight = new SwerveModule("BackRight", 7, 8, 12, 13, 14, 15);

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  Pose2d robotPose = new Pose2d();
  private double m_prevTimeSeconds = Timer.getFPGATimestamp();
  private final double m_nominalDtS = 0.02; // Seconds

  /*
   * Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings.
   * The numbers used below are robot specific, and should be tuned.
   */
  private final SwerveDrivePoseEstimator<N7, N7, N5> m_poseEstimator = new SwerveDrivePoseEstimator<N7, N7, N5>(
      Nat.N7(),
      Nat.N7(),
      Nat.N5(),
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
      },
      new Pose2d(),
      m_kinematics,
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.05, 0.05, 0.05, 0.05),
      VecBuilder.fill(Units.degreesToRadians(0.01), 0.01, 0.01, 0.01, 0.01),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(3)));

  NetworkTableInstance inst = NetworkTableInstance.getDefault();

  DoublePublisher xSpeedPubM_s = inst.getTable("desired").getDoubleTopic("xspeed m_s").publish();
  DoublePublisher ySpeedPubM_s = inst.getTable("desired").getDoubleTopic("yspeed m_s").publish();
  DoublePublisher thetaSpeedPubRad_s = inst.getTable("desired").getDoubleTopic("thetaspeed rad_s").publish();

  DoubleArrayPublisher fieldPub;
  StringPublisher fieldTypePub;

  List<CallbackStore> cbs = new ArrayList<CallbackStore>();

  // TODO: gyro is NED, robot is NWU, need to invert somewhere.
  AnalogGyroSim gyroSim = new AnalogGyroSim(0);

  public Drivetrain() {
    m_gyro.reset();
    inst.startClient4("blarg");
    NetworkTable fieldTable = inst.getTable("field");
    fieldPub = fieldTable.getDoubleArrayTopic("robotPose").publish();
    fieldTypePub = fieldTable.getStringTopic(".type").publish();
    fieldTypePub.set("Field2d");
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(m_gyro.getRotation2d(), new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
    }, pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeedM_s        Speed of the robot in the x direction (forward).
   * @param ySpeedM_s        Speed of the robot in the y direction (sideways).
   * @param rotRad_s           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeedM_s, double ySpeedM_s, double rotRad_s, boolean fieldRelative) {
    xSpeedPubM_s.set(xSpeedM_s);
    ySpeedPubM_s.set(ySpeedM_s);
    thetaSpeedPubRad_s.set(rotRad_s);

    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedM_s, ySpeedM_s, rotRad_s, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeedM_s, ySpeedM_s, rotRad_s));

    setModuleStates(swerveModuleStates);
  }

  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    m_frontLeft.publishState(swerveModuleStates[0]);
    m_frontRight.publishState(swerveModuleStates[1]);
    m_backLeft.publishState(swerveModuleStates[2]);
    m_backRight.publishState(swerveModuleStates[3]);

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed); // 3m/s max

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * drive one module directly
   * 
   * @param drive desired speed m/s
   * @param turn  desired rotation rad
   */
  public void test(double drive, double turn) {
    m_frontLeft.setDesiredState(new SwerveModuleState(drive, new Rotation2d(turn)));
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {

    m_poseEstimator.update(
        m_gyro.getRotation2d(),
        new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState()
        },
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });

    // Also apply vision measurements. We use 0.3 seconds in the past as an example
    // -- on a real robot, this must be calculated based either on latency or
    // timestamps.

    // m_poseEstimator.addVisionMeasurement(
    // ExampleGlobalMeasurementSensor.getEstimatedGlobalPose(
    // m_poseEstimator.getEstimatedPosition()),
    // Timer.getFPGATimestamp() - 0.3);

  }

  public void simulationInit() {
    m_frontLeft.simulationInit();
    m_frontRight.simulationInit();
    m_backLeft.simulationInit();
    m_backRight.simulationInit();
  }

  public void simulationPeriodic() {
    m_frontLeft.simulationPeriodic();
    m_frontRight.simulationPeriodic();
    m_backLeft.simulationPeriodic();
    m_backRight.simulationPeriodic();

    double currentTimeSeconds = Timer.getFPGATimestamp();
    double dtS = m_prevTimeSeconds >= 0 ? currentTimeSeconds - m_prevTimeSeconds : m_nominalDtS;
    m_prevTimeSeconds = currentTimeSeconds;

    // in simulation these should be the values we just set
    // in SwerveModule.simulationPeriodic(), so we don't need
    // to adjust them *again*, just use them to update the gyro.
    SwerveModuleState[] states = new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState()
    };

    ChassisSpeeds speeds = m_kinematics.toChassisSpeeds(states);

    // finally adjust the simulator gyro.
    Pose2d newPose = new Pose2d(robotPose.getX() + speeds.vxMetersPerSecond * dtS,
        robotPose.getY() + speeds.vyMetersPerSecond * dtS,
        robotPose.getRotation().plus(new Rotation2d(speeds.omegaRadiansPerSecond * dtS)));
    robotPose = newPose;
    gyroSim.setAngle(robotPose.getRotation().getRadians());

    // cheat, tell the pose estimator to use this pose.
    // doesn't seem to do much.
    // m_poseEstimator.addVisionMeasurement(robotPose, Timer.getFPGATimestamp());

    fieldPub.set(new double[] {
        newPose.getX(),
        newPose.getY(),
        newPose.getRotation().getDegrees()
    });
  }
}
