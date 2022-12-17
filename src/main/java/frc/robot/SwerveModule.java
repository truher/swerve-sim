// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class SwerveModule {
  public static final double TURN_KV = 0.05;
  public static final double DRIVE_KV = 0.15;
  public static final double TURN_KS = 0.001;
  public static final double DRIVE_KS = 0.001;
  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = 4096;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 50 * Math.PI; // radians per second squared

  private final MotorController m_driveMotor;
  private final MotorController m_turningMotor;

  private final Encoder m_driveEncoder;
  private final Encoder m_turningEncoder;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(0.1, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      0.2,
      0,
      0,
      new TrapezoidProfile.Constraints(
          kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(DRIVE_KS, DRIVE_KV);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(TURN_KS, TURN_KV);

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
      int driveMotorChannel,
      int turningMotorChannel,
      int driveEncoderChannelA,
      int driveEncoderChannelB,
      int turningEncoderChannelA,
      int turningEncoderChannelB) {
    m_driveMotor = new PWMSparkMax(driveMotorChannel);
    m_turningMotor = new PWMSparkMax(turningMotorChannel);

    m_driveEncoder = new Encoder(driveEncoderChannelA, driveEncoderChannelB);
    m_turningEncoder = new Encoder(turningEncoderChannelA, turningEncoderChannelB);

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

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getDistance()));
    // state.speedMetersPerSecond max is correct at 3.
    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.getDistance(),
        state.angle.getRadians());

    final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    // m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_driveMotor.set(driveOutput + driveFeedforward);

    // m_turningMotor.setVoltage(turnOutput + turnFeedforward);
    m_turningMotor.set(turnOutput + turnFeedforward);

    // System.out.printf("drive desired %5.3f encoder %5.3f output %5.3f ff %5.3f motor
    // %5.3f\n",
    // desiredState.speedMetersPerSecond, m_driveEncoder.getRate(), driveOutput,
    // driveFeedforward, m_driveMotor.get());
   // System.out.printf("turn desired %5.3f encoder %5.3f output %5.3f ff %5.3f motor %5.3f\n",
    //    desiredState.angle.getRadians(), m_turningEncoder.getDistance(), turnOutput, turnFeedforward, m_turningMotor.get());
  }
}
