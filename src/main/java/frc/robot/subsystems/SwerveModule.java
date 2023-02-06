// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonFX m_turningMotor;
  private final CANCoder m_turningEncoder;
  // private PIDController mRotorPID; 
  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel The channel of the drive motor.
   * @param turningMotorChannel The channel of the turning motor.
   * @param driveEncoderChannels The channels of the drive encoder.
   * @param turningEncoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningMotorEncoderChannel,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed,
      boolean turningMotorReversed,
      boolean driveReversed,
      double cancoderOffset) {

    // initialize
    m_driveMotor = new WPI_TalonFX(driveMotorChannel);
    m_turningMotor = new WPI_TalonFX(turningMotorChannel);
    
    m_turningEncoder = new CANCoder(turningMotorEncoderChannel);

    // setting
    m_driveMotor.configFactoryDefault();
    m_turningMotor.configFactoryDefault();
    m_turningEncoder.configFactoryDefault();

    // voltage compensation
    m_driveMotor.enableVoltageCompensation(true);
    m_driveMotor.configVoltageCompSaturation(9);
    m_turningMotor.enableVoltageCompensation(true);
    m_turningMotor.configVoltageCompSaturation(9);

    // sensor
    m_turningEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    m_turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_turningEncoder.configFeedbackCoefficient(0.087890625, "deg", SensorTimeBase.PerSecond);
    m_turningEncoder.configSensorDirection(turningEncoderReversed);
    m_turningEncoder.configMagnetOffset(cancoderOffset);
    m_turningEncoder.setPositionToAbsolute();

    // set remote sensor
    m_turningMotor.configRemoteFeedbackFilter(m_turningEncoder, 0);
    m_turningMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, 0, 10);
    m_turningMotor.configSelectedFeedbackSensor(FeedbackDevice.None, 1, 10);
    
    m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    // m_driveMotor.setSelectedSensorPosition(0);
    // reverse
    m_driveMotor.setInverted(driveReversed);
    m_driveMotor.setSensorPhase(driveEncoderReversed);

    m_turningMotor.setInverted(turningMotorReversed);

    // deadband
    m_driveMotor.configNeutralDeadband(0.05);
    m_turningMotor.configNeutralDeadband(0.1);

    // PIDF
    m_driveMotor.config_kF(0, 0.065);
    m_driveMotor.config_kP(0, 0.15);
    m_driveMotor.config_kI(0, 0);
    m_driveMotor.config_kD(0, 0);

    m_turningMotor.config_kF(0, 0.14);
    m_turningMotor.config_kP(0, 1.2);
    m_turningMotor.config_kI(0, 0);
    m_turningMotor.config_kD(0, 0);

    m_turningMotor.configAllowableClosedloopError(0, 0);
    m_turningMotor.configNominalOutputForward(0.095);
    m_turningMotor.configNominalOutputReverse(-0.095);

    m_turningMotor.configMotionAcceleration(4096);
    m_turningMotor.configMotionCruiseVelocity(5108);
    m_turningMotor.setNeutralMode(NeutralMode.Brake);
    m_driveMotor.setNeutralMode(NeutralMode.Coast);

    var voltage = new SupplyCurrentLimitConfiguration(true, 30, 50, 0.1);
    m_driveMotor.configSupplyCurrentLimit(voltage);
  }
  // CANcoder to talon
  private double deg2raw(double deg) {
    return ((deg)) / (360.0 / 4096.0);
  }

  // m
  public double getDriveEncoderPosition() {
    return m_driveMotor.getSelectedSensorPosition() * ModuleConstants.kDriveCoefficient;
  }

  // m/s
  public double getDriveEncoderVelocity() {
    return m_driveMotor.getSelectedSensorVelocity() * ModuleConstants.kDriveCoefficient * 10;
  }

  // deg
  public double getTurningEncoderAngle() {
    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    return m_turningMotor.getSelectedSensorPosition() / 4096.0 * 360.0;
  }

  public double getTurningEncoderRadian() {
    return getTurningEncoderAngle() / 180.0 * Math.PI;
  }
  
  public double getTurningEncoderRaw() {
    return deg2raw(getTurningEncoderAngle());
  }
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveEncoderVelocity(), new Rotation2d(getTurningEncoderRadian()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDriveEncoderPosition(), Rotation2d.fromDegrees(getTurningEncoderAngle()));
  }
  public double getError() {
    return m_driveMotor.getClosedLoopError();
  }
  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  int x = 0;
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = 
        SwerveModuleState.optimize(desiredState, new Rotation2d(getTurningEncoderRadian()));

    x++;
    if(x >= 10) {
      x=0;
      SmartDashboard.putNumber("Velocity", state.speedMetersPerSecond / ModuleConstants.kDriveCoefficient);
      SmartDashboard.putNumber("Velocity(sensor)", m_driveMotor.getSelectedSensorVelocity());
    }
    // m -> raw    
    // 補償CTRE是以每0.1s
    m_driveMotor.set(ControlMode.Velocity, state.speedMetersPerSecond / ModuleConstants.kDriveCoefficient / 10.0);
    // deg -> raw
    m_turningMotor.set(ControlMode.MotionMagic, deg2raw(state.angle.getDegrees()));
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveMotor.setSelectedSensorPosition(0);
    m_turningMotor.setSelectedSensorPosition(0);
  }
}
