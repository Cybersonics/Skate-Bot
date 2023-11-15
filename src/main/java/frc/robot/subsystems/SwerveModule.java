/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController; //Use for Roborio PID
import edu.wpi.first.math.MathUtil; // Use for RoboRio PID
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule extends SubsystemBase {

  private String moduleName;
  public double currentPosition;
  private CANSparkMax steerMotor;
  private CANSparkMax driveMotor;
  private AnalogInput steerMotorInput; // Set up analog input for Roborio
  private PIDController steerPIDController; // Setup PID in RoboRio
  private static final double RAMP_RATE = 0.5;

  private RelativeEncoder driveMotorEncoder; // Set up integrated Drive motor encoder in Spark Max/Neo

  public double encoderCountPerRotation = 1024;

  private boolean _driveCorrect;

  public SwerveModule(String name, int steerNum, int driveNum, int steerInput, PIDController steerController,
      boolean invertDrive, boolean invertSteer) {
    moduleName = name;
    // Create and configure a new Drive motor
    driveMotor = new CANSparkMax(driveNum, MotorType.kBrushless);
    driveMotor.restoreFactoryDefaults();
    driveMotor.setInverted(invertDrive);// setInverted reverses the both the motor and the encoder direction.
    driveMotor.setOpenLoopRampRate(RAMP_RATE);// This provides a motor ramp up time to prevent brown outs.
    driveMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setSmartCurrentLimit(55);

    // Create and configure an analog input on a roborio port
    steerMotorInput = new AnalogInput(steerInput);

    // Create and configure a new Steering motor
    steerMotor = new CANSparkMax(steerNum, MotorType.kBrushless);
    steerMotor.restoreFactoryDefaults();
    steerMotor.setInverted(invertSteer); // setInverted reverses the both the motor and the encoder direction.
    steerMotor.setOpenLoopRampRate(RAMP_RATE);

    // Set steerPIDController
    steerPIDController = steerController;

    // Create the built in motor encoder
    driveMotorEncoder = driveMotor.getEncoder();
    driveMotorEncoder.setPositionConversionFactor(DriveConstants.ModuleConstants.kDriveEncoderRot2Meter);
    driveMotorEncoder.setVelocityConversionFactor(DriveConstants.ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    driveMotor.burnFlash();// Set configuration values to flash memory in Spark Max to prevent errors.
    resetEncoders();
  }

  public void setSwerve(double angle, double speed, boolean driveCorrect) {
    double currentPosition = steerMotorInput.getValue();

    double currentAngle = (currentPosition * 360.0 / DriveConstants.ModuleConstants.ENCODER_COUNT_PER_ROTATION) % 360.0;
    // The angle from the encoder is in the range [0, 360], but the swerve
    // computations
    // return angles in the range [-180, 180], so transform the encoder angle to
    // this range
    if (currentAngle > 180.0) {
      currentAngle -= 360.0;
    }
    // TODO: Properly invert the steering motors so this isn't necessary
    // This is because the steering encoders are inverted
    double targetAngle = -angle;
    double deltaDegrees = targetAngle - currentAngle;
    // If we need to turn more than 180 degrees, it's faster to turn in the opposite
    // direction
    if (Math.abs(deltaDegrees) > 180.0) {
      deltaDegrees -= 360.0 * Math.signum(deltaDegrees);
    }
    // If we need to turn more than 90 degrees, we can reverse the wheel direction
    // instead and
    // only rotate by the complement

    // if (Math.abs(speed) <= MAX_SPEED){
    // if (Math.abs(deltaDegrees) > 90.0) {
    // deltaDegrees -= 180.0 * Math.signum(deltaDegrees);
    // speed = -speed;
    // }
    // }

    double targetPosition = (currentPosition
        + ((deltaDegrees * DriveConstants.ModuleConstants.ENCODER_COUNT_PER_ROTATION) / 360.0))
        % DriveConstants.ModuleConstants.ENCODER_COUNT_PER_ROTATION;
    double pidOut = steerPIDController.calculate(currentPosition, targetPosition);
    double outVal = pidOut / DriveConstants.ModuleConstants.ENCODER_COUNT_PER_ROTATION;

    steerMotor.set(outVal);
    driveMotor.set(speed);

    SmartDashboard.putNumber((moduleName + "currentPosition"), currentPosition);
    SmartDashboard.putNumber((moduleName + "Setpoint"), targetPosition);

    SmartDashboard.putNumber((moduleName + "PID Out: "), pidOut);
    SmartDashboard.putNumber((moduleName + "Out: "), outVal);
    SmartDashboard.putNumber((moduleName + "speed: "), speed);
  }

  /*
   * Get the built in Spark/Neo Drive motor encoder position. Value is in motor
   * revolutions.
   */
  public double getDriveEncoder() {
    return driveMotorEncoder.getPosition();
  }

  /*
   * Set the position value of the Spark/Neo Drive motor encoder position.
   * Position is in
   * motor revolutions.
   */
  public void setDriveEncoder(double position) {
    driveMotorEncoder.setPosition(position);
  }

  public double getDriveVelocity() {
    return driveMotorEncoder.getVelocity();
  }

  /*
   * Set the drive motor speed from -1 to 1
   */
  public void setDriveSpeed(double speed) {
    driveMotor.set(speed);
  }

  /*
   * Get the drive motor speed.
   */
  public double getDriveSpeed() {
    return driveMotor.get();
  }

  public void stopDriveMotor() {
    driveMotor.stopMotor();
  }

  public double getSteerEncoder() {
    return steerMotorInput.getValue();
  }

  public double getSteerEncDeg() {
    double angle = (1.0 - steerMotorInput.getValue() / DriveConstants.ModuleConstants.ENCODER_COUNT_PER_ROTATION) * 2.0
        * Math.PI;
    // angle += offset;
    angle %= 2.0 * Math.PI;
    if (angle < 0.0) {
      angle += 2.0 * Math.PI;
    }

    return angle;
  }

  public double getTurningPosition() {
    double steerEncoderRaw = getSteerEncoder();
    double turningEncoder = (steerEncoderRaw / this.encoderCountPerRotation) * 2 * Math.PI;
    return -turningEncoder; // Invert Encoder for odometry as wpilib treats encoders backwards.
  }

  public void resetEncoders() {
    driveMotorEncoder.setPosition(0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDriveEncoder(), new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    double driveMotorSpeed = state.speedMetersPerSecond
        / DriveConstants.FrameConstants.kPhysicalMaxSpeedMetersPerSecond;
    double steerMotorAngle = state.angle.getDegrees();
    setSwerve(steerMotorAngle, driveMotorSpeed, false);
  }

  public void stop() {
    driveMotor.set(0);
  }

  public void driveMotorRamp(boolean enableRamp) {
    if (enableRamp) {
      driveMotor.setOpenLoopRampRate(RAMP_RATE);
    } else {
      driveMotor.setOpenLoopRampRate(0);
    }
  }

  // Set Drive Mode
  public void setDriveMode(IdleMode idleMode) {
    driveMotor.setIdleMode(idleMode);
  }

  // Get Drvie Mode
  public IdleMode getDriveMode() {
    return driveMotor.getIdleMode();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
