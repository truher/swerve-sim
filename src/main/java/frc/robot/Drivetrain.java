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
import edu.wpi.first.wpilibj.simulation.EncoderSim;
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

  private final SwerveModule m_frontLeft = new SwerveModule(1, 2, 0, 1, 2, 3);
  private final SwerveModule m_frontRight = new SwerveModule(3, 4, 4, 5, 6, 7);
  private final SwerveModule m_backLeft = new SwerveModule(5, 6, 8, 9, 10, 11);
  private final SwerveModule m_backRight = new SwerveModule(7, 8, 12, 13, 14, 15);

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  Pose2d robotPose = new Pose2d();
  private double m_prevTimeSeconds = Timer.getFPGATimestamp();
  private final double m_nominalDtS = 0.02; // Seconds

  /*
   * Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings.
   * The numbers used
   * below are robot specific, and should be tuned.
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

  NetworkTable frontLeft = inst.getTable("FrontLeft");
  NetworkTable frontRight = inst.getTable("FrontRight");
  NetworkTable backLeft = inst.getTable("BackLeft");
  NetworkTable backRight = inst.getTable("BackRight");

  // distance
  DoublePublisher frontLeftDriveEncoderPubM = frontLeft.getDoubleTopic("driveEncoderDistanceM").publish();
  DoublePublisher frontLeftTurnEncoderPubRad = frontLeft.getDoubleTopic("turnEncoderDistanceRad").publish();
  DoublePublisher frontRightDriveEncoderPubM = frontRight.getDoubleTopic("driveEncoderDistanceM").publish();
  DoublePublisher frontRightTurnEncoderPubRad = frontRight.getDoubleTopic("turnEncoderDistanceRad").publish();
  DoublePublisher backLeftDriveEncoderPubM = backLeft.getDoubleTopic("driveEncoderDistanceM").publish();
  DoublePublisher backLeftTurnEncoderPubRad = backLeft.getDoubleTopic("turnEncoderDistanceRad").publish();
  DoublePublisher backRightDriveEncoderPubM = backRight.getDoubleTopic("driveEncoderDistanceM").publish();
  DoublePublisher backRightTurnEncoderPubRad = backRight.getDoubleTopic("turnEncoderDistanceRad").publish();

  // drive rate only; turn rate is ignored
  DoublePublisher frontLeftDriveEncoderRatePubM_s = frontLeft.getDoubleTopic("driveEncoderRateM_s").publish();
  DoublePublisher frontRightDriveEncoderRatePubM_s = frontRight.getDoubleTopic("driveEncoderRateM_s").publish();
  DoublePublisher backLeftDriveEncoderRatePubM_s = backLeft.getDoubleTopic("driveEncoderRateM_s").publish();
  DoublePublisher backRightDriveEncoderRatePubM_s = backRight.getDoubleTopic("driveEncoderRateM_s").publish();

  // motor output should be [-1,1]
  DoublePublisher frontLeftDrivePWMPub1_1 = frontLeft.getDoubleTopic("drivePWMOutput1_1").publish();
  DoublePublisher frontLeftTurnPWMPub1_1 = frontLeft.getDoubleTopic("turnPWMOutput1_1").publish();
  DoublePublisher frontRightDrivePWMPub1_1 = frontRight.getDoubleTopic("drivePWMOutput1_1").publish();
  DoublePublisher frontRightTurnPWMPub1_1 = frontRight.getDoubleTopic("turnPWMOutput1_1").publish();
  DoublePublisher backLeftDrivePWMPub1_1 = backLeft.getDoubleTopic("drivePWMOutput1_1").publish();
  DoublePublisher backLeftTurnPWMPub1_1 = backLeft.getDoubleTopic("turnPWMOutput1_1").publish();
  DoublePublisher backRightDrivePWMPub1_1 = backRight.getDoubleTopic("drivePWMOutput1_1").publish();
  DoublePublisher backRightTurnPWMPub1_1 = backRight.getDoubleTopic("turnPWMOutput1_1").publish();

  // desired velocity or position from input.
  DoublePublisher frontLeftDriveVInPubM_s = frontLeft.getDoubleTopic("driveInputSpeedM_s").publish();
  DoublePublisher frontLeftTurnPInPubRad = frontLeft.getDoubleTopic("turnInputRad").publish();
  DoublePublisher frontRightDriveVInPubM_s = frontRight.getDoubleTopic("driveInputSpeedM_s").publish();
  DoublePublisher frontRightTurnPInPubRad = frontRight.getDoubleTopic("turnInputRad").publish();
  DoublePublisher backLeftDriveVInPubM_s = backLeft.getDoubleTopic("driveInputSpeedM_s").publish();
  DoublePublisher backLeftTurnPInPubRad = backLeft.getDoubleTopic("turnDInputRad").publish();
  DoublePublisher backRightDriveVInPubM_s = backRight.getDoubleTopic("driveInputSpeedM_s").publish();
  DoublePublisher backRightTurnPInPubRad = backRight.getDoubleTopic("turnInputSRad").publish();

  // desired velocity from "inverse feed forward", should be m/s or rad/s.
  DoublePublisher frontLeftDriveVPubM_s = frontLeft.getDoubleTopic("driveDesiredSpeedM_s").publish();
  DoublePublisher frontLeftTurnVPubRad_s = frontLeft.getDoubleTopic("turnDesiredSpeedRad_s").publish();
  DoublePublisher frontRightDriveVPubM_s = frontRight.getDoubleTopic("driveDesiredSpeedM_s").publish();
  DoublePublisher frontRightTurnVPubRad_s = frontRight.getDoubleTopic("turnDesiredSpeedRad_s").publish();
  DoublePublisher backLeftDriveVPubM_s = backLeft.getDoubleTopic("driveDesiredSpeedM_s").publish();
  DoublePublisher backLeftTurnVPubRad_s = backLeft.getDoubleTopic("turnDesiredSpeedRad_s").publish();
  DoublePublisher backRightDriveVPubM_s = backRight.getDoubleTopic("driveDesiredSpeedM_s").publish();
  DoublePublisher backRightTurnVPubRad_s = backRight.getDoubleTopic("turnDesiredSpeedRad_s").publish();

  DoubleArrayPublisher fieldPub;
  StringPublisher fieldTypePub;

  List<CallbackStore> cbs = new ArrayList<CallbackStore>();

  EncoderSim frontLeftDriveEncoderSim = EncoderSim.createForChannel(0);
  EncoderSim frontLeftTurnEncoderSim = EncoderSim.createForChannel(2);
  EncoderSim frontRightDriveEncoderSim = EncoderSim.createForChannel(4);
  EncoderSim frontRightTurnEncoderSim = EncoderSim.createForChannel(6);
  EncoderSim backLeftDriveEncoderSim = EncoderSim.createForChannel(8);
  EncoderSim backLeftTurnEncoderSim = EncoderSim.createForChannel(10);
  EncoderSim backRightDriveEncoderSim = EncoderSim.createForChannel(12);
  EncoderSim backRightTurnEncoderSim = EncoderSim.createForChannel(14);

  PWMSim frontLeftDrivePWMSim = new PWMSim(1);
  PWMSim frontLeftTurnPWMSim = new PWMSim(2);
  PWMSim frontRightDrivePWMSim = new PWMSim(3);
  PWMSim frontRightTurnPWMSim = new PWMSim(4);
  PWMSim backLeftDrivePWMSim = new PWMSim(5);
  PWMSim backLeftTurnPWMSim = new PWMSim(6);
  PWMSim backRightDrivePWMSim = new PWMSim(7);
  PWMSim backRightTurnPWMSim = new PWMSim(8);
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
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // requestedXSpeed = xSpeed m/s
    // requestedYSpeed = ySpeed m/s
    // requestedThetaSpeed = rot rad/s
    xSpeedPubM_s.set(xSpeed);
    ySpeedPubM_s.set(ySpeed);
    thetaSpeedPubRad_s.set(rot);

    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));

    setModuleStates(swerveModuleStates);
  }

  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    frontLeftDriveVInPubM_s.set(swerveModuleStates[0].speedMetersPerSecond);
    frontLeftTurnPInPubRad.set(swerveModuleStates[0].angle.getRadians());
    frontRightDriveVInPubM_s.set(swerveModuleStates[1].speedMetersPerSecond);
    frontRightTurnPInPubRad.set(swerveModuleStates[1].angle.getRadians());
    backLeftDriveVInPubM_s.set(swerveModuleStates[2].speedMetersPerSecond);
    backLeftTurnPInPubRad.set(swerveModuleStates[2].angle.getRadians());
    backRightDriveVInPubM_s.set(swerveModuleStates[3].speedMetersPerSecond);
    backRightTurnPInPubRad.set(swerveModuleStates[3].angle.getRadians());

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

  public void pubSim(PWMSim sim, DoublePublisher pub) {
    cbs.add(sim.registerSpeedCallback((name, value) -> pub.set(value.getDouble()), true));
  }

  public void simulationInit() {
    pubSim(frontLeftDrivePWMSim, frontLeftDrivePWMPub1_1);
    pubSim(frontLeftTurnPWMSim, frontLeftTurnPWMPub1_1);
    pubSim(frontRightDrivePWMSim, frontRightDrivePWMPub1_1);
    pubSim(frontRightTurnPWMSim, frontRightTurnPWMPub1_1);
    pubSim(backLeftDrivePWMSim, backLeftDrivePWMPub1_1);
    pubSim(backLeftTurnPWMSim, backLeftTurnPWMPub1_1);
    pubSim(backRightDrivePWMSim, backRightDrivePWMPub1_1);
    pubSim(backRightTurnPWMSim, backRightTurnPWMPub1_1);
  }

  /**
   * turn motor voltage back into speed
   * 
   * inverting the feedforward is surely wrong but it should work.
   * 
   * feedforward is
   * output = ks * signum(v) + kv * v.
   * so,
   * v = (output - ks*signum(output))/kv
   * 
   * to slow things down, divide by ... two?
   * 
   * @param output [-1,1]
   */
  public double vFromOutput(double output, double ks, double kv) {
    // System.out.printf("output %f\n", output);
    double result = (output - ks * Math.signum(output)) / kv;
    // System.out.printf("result %f\n", result);
    return result;
  }

  public double vFromTurnOutput(double output, double ks, double kv) {
    return (output - ks * Math.signum(output)) / kv;
  }

  public void simulationPeriodic() {
    double currentTimeSeconds = Timer.getFPGATimestamp();
    double dtS = m_prevTimeSeconds >= 0 ? currentTimeSeconds - m_prevTimeSeconds : m_nominalDtS;
    m_prevTimeSeconds = currentTimeSeconds;

    // drive velocities are meters per second
    // turn velocities are radians per second.
    double frontLeftDriveVM_s = vFromOutput(frontLeftDrivePWMSim.getSpeed(), SwerveModule.DRIVE_KS,
        SwerveModule.DRIVE_KV);
    double frontLeftTurnVRad_s = vFromTurnOutput(frontLeftTurnPWMSim.getSpeed(), SwerveModule.TURN_KS,
        SwerveModule.TURN_KV);
    double frontRightDriveVM_s = vFromOutput(frontRightDrivePWMSim.getSpeed(), SwerveModule.DRIVE_KS,
        SwerveModule.DRIVE_KV);
    double frontRightTurnVRad_s = vFromTurnOutput(frontRightTurnPWMSim.getSpeed(), SwerveModule.TURN_KS,
        SwerveModule.TURN_KV);
    double backLeftDriveVM_s = vFromOutput(backLeftDrivePWMSim.getSpeed(), SwerveModule.DRIVE_KS,
        SwerveModule.DRIVE_KV);
    double backLeftTurnVRad_s = vFromTurnOutput(backLeftTurnPWMSim.getSpeed(), SwerveModule.TURN_KS,
        SwerveModule.TURN_KV);
    double backRightDriveVM_s = vFromOutput(backRightDrivePWMSim.getSpeed(), SwerveModule.DRIVE_KS,
        SwerveModule.DRIVE_KV);
    double backRightTurnVRad_s = vFromTurnOutput(backRightTurnPWMSim.getSpeed(), SwerveModule.TURN_KS,
        SwerveModule.TURN_KV);

    frontLeftDriveVPubM_s.set(frontLeftDriveVM_s);
    frontLeftTurnVPubRad_s.set(frontLeftTurnVRad_s);
    frontRightDriveVPubM_s.set(frontRightDriveVM_s);
    frontRightTurnVPubRad_s.set(frontRightTurnVRad_s);
    backLeftDriveVPubM_s.set(backLeftDriveVM_s);
    backLeftTurnVPubRad_s.set(backLeftTurnVRad_s);
    backRightDriveVPubM_s.set(backRightDriveVM_s);
    backRightTurnVPubRad_s.set(backRightTurnVRad_s);

    // set the encoders
    frontLeftDriveEncoderSim.setRate(frontLeftDriveVM_s);
    frontLeftDriveEncoderSim.setDistance(frontLeftDriveEncoderSim.getDistance() + frontLeftDriveVM_s * dtS);
    frontLeftTurnEncoderSim.setDistance(frontLeftTurnEncoderSim.getDistance() + frontLeftTurnVRad_s * dtS);

    frontRightDriveEncoderSim.setRate(frontRightDriveVM_s);
    frontRightDriveEncoderSim.setDistance(frontRightDriveEncoderSim.getDistance() + frontRightDriveVM_s * dtS);
    frontRightTurnEncoderSim.setDistance(frontRightTurnEncoderSim.getDistance() + frontRightTurnVRad_s * dtS);

    backLeftDriveEncoderSim.setRate(backLeftDriveVM_s);
    backLeftDriveEncoderSim.setDistance(backLeftDriveEncoderSim.getDistance() + backLeftDriveVM_s * dtS);
    backLeftTurnEncoderSim.setDistance(backLeftTurnEncoderSim.getDistance() + backLeftTurnVRad_s * dtS);

    backRightDriveEncoderSim.setRate(backRightDriveVM_s);
    backRightDriveEncoderSim.setDistance(backRightDriveEncoderSim.getDistance() + backRightDriveVM_s * dtS);
    backRightTurnEncoderSim.setDistance(backRightTurnEncoderSim.getDistance() + backRightTurnVRad_s * dtS);

    frontLeftDriveEncoderPubM.set(frontLeftDriveEncoderSim.getDistance());
    frontLeftTurnEncoderPubRad.set(frontLeftTurnEncoderSim.getDistance());
    frontRightDriveEncoderPubM.set(frontRightDriveEncoderSim.getDistance());
    frontRightTurnEncoderPubRad.set(frontRightTurnEncoderSim.getDistance());
    backLeftDriveEncoderPubM.set(backLeftDriveEncoderSim.getDistance());
    backLeftTurnEncoderPubRad.set(backLeftTurnEncoderSim.getDistance());
    backRightDriveEncoderPubM.set(backRightDriveEncoderSim.getDistance());
    backRightTurnEncoderPubRad.set(backRightTurnEncoderSim.getDistance());

    frontLeftDriveEncoderRatePubM_s.set(frontLeftDriveEncoderSim.getRate());
    frontRightDriveEncoderRatePubM_s.set(frontRightDriveEncoderSim.getRate());
    backLeftDriveEncoderRatePubM_s.set(backLeftDriveEncoderSim.getRate());
    backRightDriveEncoderRatePubM_s.set(backRightDriveEncoderSim.getRate());

    SwerveModuleState[] states = new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState()
    };
    states[0].angle = states[0].angle.plus(new Rotation2d(frontLeftTurnVRad_s));
    states[1].angle = states[1].angle.plus(new Rotation2d(frontRightTurnVRad_s));
    states[2].angle = states[2].angle.plus(new Rotation2d(backLeftTurnVRad_s));
    states[3].angle = states[3].angle.plus(new Rotation2d(backRightTurnVRad_s));

    states[0].speedMetersPerSecond = frontLeftDriveVM_s;
    states[1].speedMetersPerSecond = frontRightDriveVM_s;
    states[2].speedMetersPerSecond = backLeftDriveVM_s;
    states[3].speedMetersPerSecond = backRightDriveVM_s;

    ChassisSpeeds speeds = m_kinematics.toChassisSpeeds(states);

    // finally adjust the simulator gyro.
    Pose2d newPose = new Pose2d(robotPose.getX() + speeds.vxMetersPerSecond * dtS,
        robotPose.getY() + speeds.vyMetersPerSecond * dtS,
        robotPose.getRotation().plus(new Rotation2d(speeds.omegaRadiansPerSecond * dtS)));
    robotPose = newPose;
    // gyroSim.setAngle(gyroSim.getAngle() + speeds.omegaRadiansPerSecond * dtS);
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
