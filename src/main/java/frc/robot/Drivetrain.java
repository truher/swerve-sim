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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  /** 3 m/s */
  public static final double kMaxSpeed = 6.0;
  /** pi rad/s */
  public static final double kMaxAngularSpeed = 6 * Math.PI;

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  // package visibility for testing
  final SwerveModule m_frontLeft = new SwerveModule("FrontLeft", 1, 2, 0, 1, 2, 3);
  final SwerveModule m_frontRight = new SwerveModule("FrontRight", 3, 4, 4, 5, 6, 7);
  final SwerveModule m_backLeft = new SwerveModule("BackLeft", 5, 6, 8, 9, 10, 11);
  final SwerveModule m_backRight = new SwerveModule("BackRight", 7, 8, 12, 13, 14, 15);

  final AnalogGyro m_gyro = new AnalogGyro(0);
  // note gyro is NED, robot is NWU, see inversion below.
  final AnalogGyroSim gyroSim = new AnalogGyroSim(m_gyro);

  final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  // Pose2d robotPose = new Pose2d();
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
      m_gyro.getRotation2d(), // NWU
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

  DoublePublisher actualXSpeedPubM_s = inst.getTable("actual").getDoubleTopic("xspeed m_s").publish();
  DoublePublisher actualYSpeedPubM_s = inst.getTable("actual").getDoubleTopic("yspeed m_s").publish();
  DoublePublisher actualThetaSpeedPubRad_s = inst.getTable("actual").getDoubleTopic("thetaspeed rad_s").publish();

  DoubleArrayPublisher fieldPub;
  StringPublisher fieldTypePub;

  List<CallbackStore> cbs = new ArrayList<CallbackStore>();

  ChassisSpeeds speeds;

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
    m_poseEstimator.resetPosition(m_gyro.getRotation2d(), // NWU
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        }, pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeedM_s     Speed of the robot in the x direction (forward).
   * @param ySpeedM_s     Speed of the robot in the y direction (sideways).
   * @param rotRad_s      Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeedM_s, double ySpeedM_s, double rotRad_s, boolean fieldRelative) {
    xSpeedPubM_s.set(xSpeedM_s);
    ySpeedPubM_s.set(ySpeedM_s);
    thetaSpeedPubRad_s.set(rotRad_s);

    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedM_s, ySpeedM_s, rotRad_s,
                m_gyro.getRotation2d() // NWU
            )
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
    m_frontLeft.setDesiredState(new SwerveModuleState(drive, new Rotation2d(turn) // NWU
    ));
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {

    m_poseEstimator.update(
        m_gyro.getRotation2d(), // NWU
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

    Pose2d newEstimate = m_poseEstimator.getEstimatedPosition();
    fieldPub.set(new double[] {
        newEstimate.getX(),
        newEstimate.getY(),
        newEstimate.getRotation().getDegrees()
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
    double currentTimeSeconds = Timer.getFPGATimestamp();
    double dtS = m_prevTimeSeconds >= 0 ? currentTimeSeconds - m_prevTimeSeconds : m_nominalDtS;
    m_prevTimeSeconds = currentTimeSeconds;
    simulationPeriodic(dtS);
  }



  public void simulationPeriodic(final double dtS) {
    m_frontLeft.simulationPeriodic(dtS);
    m_frontRight.simulationPeriodic(dtS);
    m_backLeft.simulationPeriodic(dtS);
    m_backRight.simulationPeriodic(dtS);

    // in simulation these should be the values we just set
    // in SwerveModule.simulationPeriodic(), so we don't need
    // to adjust them *again*, just use them to update the gyro.
    SwerveModuleState[] states = new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState()
    };

    // rotational velocity is correct here.
    speeds = m_kinematics.toChassisSpeeds(states);

    // finally adjust the simulator gyro.
    // the pose estimator figures out the X/Y part but it depends on the gyro.
    // since omega is the same in both coordinate schemes, just use that.
    double oldAngleDeg = gyroSim.getAngle();
    double dThetaDeg = -1.0 * new Rotation2d(speeds.omegaRadiansPerSecond * dtS).getDegrees();
    double newAngleDeg = oldAngleDeg + dThetaDeg;
    // note that the "angle" in a gyro is NED, but everything else (e.g robot pose)
    // is NWU, so invert here.
    gyroSim.setAngle(newAngleDeg);

    xSpeedPubM_s.set(speeds.vxMetersPerSecond);
    ySpeedPubM_s.set(speeds.vyMetersPerSecond);
    thetaSpeedPubRad_s.set(-1.0 * speeds.omegaRadiansPerSecond);
  }

  public void close() {
    m_frontLeft.close();
    m_frontRight.close();
    m_backLeft.close();
    m_backRight.close();
    m_gyro.close();
  }
}
