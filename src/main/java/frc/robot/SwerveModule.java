// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.CallbackStore;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;

public class SwerveModule {
  public static final double TURN_KV = 0.05;
  public static final double DRIVE_KV = 0.15;
  public static final double TURN_KS = 0.001;
  public static final double DRIVE_KS = 0.001;
  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = 4096;

  private static final double kModuleMaxAngularVelocity = 10 * Math.PI; // rad/s, fast
  private static final double kModuleMaxAngularAcceleration = 50 * Math.PI; // rad/s^2

  private final PWMMotorController m_driveMotor;
  private final PWMSim m_DrivePWMSim;
  // these are out here so i can observe them for testing.
  double driveOutput;
  double driveFeedforward;

  private final PWMMotorController m_turningMotor;
  private final PWMSim m_TurnPWMSim;

  final Encoder m_driveEncoder;
  private final EncoderSim m_DriveEncoderSim;
  private final Encoder m_turningEncoder; // NWU
  final EncoderSim m_TurnEncoderSim;

  final PIDController m_drivePIDController = new PIDController(0.1, 0, 0);

  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      0.2,
      0,
      0,
      new TrapezoidProfile.Constraints(
          kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(DRIVE_KS, DRIVE_KV);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(TURN_KS, TURN_KV);

  // ######## network tables ########
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  // private final String m_name;
  private final NetworkTable m_table;
  // distance, m
  private final DoublePublisher m_DriveEncoderPubM;
  // distance, rad
  private final DoublePublisher m_TurnEncoderPubRad;
  // drive rate only, m/s; turn rate is ignored
  private final DoublePublisher m_DriveEncoderRatePubM_s;
  // motor output, [-1,1]
  private final DoublePublisher m_DrivePWMPub1_1;
  private final DoublePublisher m_TurnPWMPub1_1;
  // desired velocity from "inverse feed forward", m/s
  private final DoublePublisher m_DriveVPubM_s;
  // desired velocity from "inverse feed forward", rad/s
  private final DoublePublisher m_TurnVPubRad_s;
  // desired velocity from input.
  private final DoublePublisher m_DriveVInPubM_s;
  // desired position from input.
  private final DoublePublisher m_TurnPInPubRad;

  List<CallbackStore> cbs = new ArrayList<CallbackStore>();
  private double m_prevTimeSeconds = Timer.getFPGATimestamp();
  private final double m_nominalDtS = 0.02; // Seconds

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   *
   * @param driveMotorChannel      PWM output for the drive motor.
   * @param turningMotorChannel    PWM output for the turning motor.
   * @param driveEncoderChannelA   DIO input for the drive encoder channel A
   * @param driveEncoderChannelB   DIO input for the drive encoder channel B
   * @param turningEncoderChannelA DIO input for the turning encoder channel A
   * @param turningEncoderChannelB DIO input for the turning encoder channel B
   */
  public SwerveModule(
      String name,
      int driveMotorChannel,
      int turningMotorChannel,
      int driveEncoderChannelA,
      int driveEncoderChannelB,
      int turningEncoderChannelA,
      int turningEncoderChannelB) {
    // m_name = name;
    m_table = inst.getTable(name);
    m_DriveEncoderPubM = m_table.getDoubleTopic("driveEncoderDistanceM").publish();
    m_TurnEncoderPubRad = m_table.getDoubleTopic("turnEncoderDistanceRad").publish();
    m_DriveEncoderRatePubM_s = m_table.getDoubleTopic("driveEncoderRateM_s").publish();
    m_DrivePWMPub1_1 = m_table.getDoubleTopic("drivePWMOutput1_1").publish();
    m_TurnPWMPub1_1 = m_table.getDoubleTopic("turnPWMOutput1_1").publish();
    m_DriveVPubM_s = m_table.getDoubleTopic("driveDesiredSpeedM_s").publish();
    m_TurnVPubRad_s = m_table.getDoubleTopic("turnDesiredSpeedRad_s").publish();
    m_DriveVInPubM_s = m_table.getDoubleTopic("driveInputSpeedM_s").publish();
    m_TurnPInPubRad = m_table.getDoubleTopic("turnInputRad").publish();

    m_driveMotor = new PWMSparkMax(driveMotorChannel);
    m_DrivePWMSim = new PWMSim(m_driveMotor);

    m_turningMotor = new PWMSparkMax(turningMotorChannel);
    m_TurnPWMSim = new PWMSim(m_turningMotor);

    m_driveEncoder = new Encoder(driveEncoderChannelA, driveEncoderChannelB);
    m_DriveEncoderSim = new EncoderSim(m_driveEncoder);

    m_turningEncoder = new Encoder(turningEncoderChannelA, turningEncoderChannelB);
    m_TurnEncoderSim = new EncoderSim(m_turningEncoder);

    pubSim(m_DrivePWMSim, m_DrivePWMPub1_1);
    pubSim(m_TurnPWMSim, m_TurnPWMPub1_1);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    // Set the distance (in this case, angle) in radians per pulse for the turning
    // encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void pubSim(PWMSim sim, DoublePublisher pub) {
    cbs.add(sim.registerSpeedCallback((name, value) -> pub.set(value.getDouble()), true));
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
   * @param output [-1,1]
   */
  public double vFromOutput(double output, double ks, double kv) {
    double result = (output - ks * Math.signum(output)) / kv;
    return result;
  }

  public void simulationPeriodic() {
    double currentTimeSeconds = Timer.getFPGATimestamp();
    double dtS = m_prevTimeSeconds >= 0 ? currentTimeSeconds - m_prevTimeSeconds : m_nominalDtS;
    m_prevTimeSeconds = currentTimeSeconds;
    simulationPeriodic(dtS);
  }

  public void simulationPeriodic(double dtS) {

    // derive velocity from motor output
    double driveVM_s = vFromOutput(m_DrivePWMSim.getSpeed(), DRIVE_KS, DRIVE_KV);
    double turnVRad_s = vFromOutput(m_TurnPWMSim.getSpeed(), TURN_KS, TURN_KV);

    // observe the derived velocity
    m_DriveVPubM_s.set(driveVM_s);
    m_TurnVPubRad_s.set(turnVRad_s);

    // set the encoders using the derived velocity
    m_DriveEncoderSim.setRate(driveVM_s);
    m_DriveEncoderSim.setDistance(m_DriveEncoderSim.getDistance() + driveVM_s * dtS);
    m_TurnEncoderSim.setDistance(m_TurnEncoderSim.getDistance() + turnVRad_s * dtS);

    // observe the encoders
    m_DriveEncoderPubM.set(m_DriveEncoderSim.getDistance());
    m_TurnEncoderPubRad.set(m_TurnEncoderSim.getDistance());
    m_DriveEncoderRatePubM_s.set(m_DriveEncoderSim.getRate());

  }

  public void simulationInit() {
    // nothing to do
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.getDistance()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getDistance(), new Rotation2d(m_turningEncoder.getDistance()));
  }

  // just to see it, has no effect
  public void publishState(SwerveModuleState state) {
    m_DriveVInPubM_s.set(state.speedMetersPerSecond);
    m_TurnPInPubRad.set(state.angle.getRadians());
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState,
        new Rotation2d(m_turningEncoder.getDistance()));
    // state.speedMetersPerSecond max is correct at 3.
    // Calculate the drive output from the drive PID controller.
    driveOutput = m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);

    driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.getDistance(),
        state.angle.getRadians());

    final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    // m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_driveMotor.set(driveOutput + driveFeedforward);

    // m_turningMotor.setVoltage(turnOutput + turnFeedforward);
    m_turningMotor.set(turnOutput + turnFeedforward);
  }

  public double getDriveOutput() {
    return m_driveMotor.get();
  }

  public double getTurnOutput() {
    return m_turningMotor.get();
  }

  /** This is required to keep test cases separate. */
  public void close() {
    m_driveMotor.close();
    m_turningMotor.close();

    // m_DrivePWMSim.close();
    // m_TurnPWMSim.close();

    m_driveEncoder.close();
    m_turningEncoder.close();

    // m_DriveEncoderSim.close();
    // m_TurnEncoderSim.close();
  }
}
