// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

public class Drivetrain extends SubsystemBase {
  CANSparkMax m_leftDrive1 = new CANSparkMax(1, MotorType.kBrushed);
  CANSparkMax m_leftDrive2 = new CANSparkMax(2, MotorType.kBrushed);
  CANSparkMax m_rightDrive1 = new CANSparkMax(3 , MotorType.kBrushed);
  CANSparkMax m_rightDrive2 = new CANSparkMax(4, MotorType.kBrushed);

  MotorControllerGroup m_leftDrive = new MotorControllerGroup(m_leftDrive1, m_leftDrive2);
  MotorControllerGroup m_rightDrive = new MotorControllerGroup(m_rightDrive1, m_rightDrive2);

  DifferentialDrive m_drivetrain = new DifferentialDrive(m_leftDrive, m_rightDrive);
  XboxController xboxController = new XboxController(0);

  Encoder leftEncoder = new Encoder(9, 8);
  Encoder rightEncoder = new Encoder(7, 6);
  static AHRS navX = new AHRS(SPI.Port.kMXP);
  DifferentialDriveOdometry m_odometry;

  boolean rBPressedOnce;
  boolean rBPressedTwice;
  boolean bPressed;

  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {

    leftEncoder.reset();
    rightEncoder.reset();;

    rightEncoder.setDistancePerPulse(DriveTrainConstants.kLinearDistanceConversionFactor);
    leftEncoder.setDistancePerPulse(DriveTrainConstants.kLinearDistanceConversionFactor);
    //rightEncoder.setVelocityConversionFactor(DriveTrainConstants.kLinearDistanceConversionFactor / 60);
    //leftEncoder.setVelocityConversionFactor(DriveTrainConstants.kLinearDistanceConversionFactor / 60);

    m_leftDrive.setInverted(false);
    m_rightDrive.setInverted(true);

    navX.reset();
    navX.calibrate();
    resetEncoders();

    m_odometry = new DifferentialDriveOdometry(navX.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    m_odometry.resetPosition(navX.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance(), getPose());
    setBreakMode();

    rBPressedOnce = false;
    rBPressedTwice = false;
    bPressed = false;

    m_leftDrive1.setSmartCurrentLimit(50, 60, 100);
    m_leftDrive2.setSmartCurrentLimit(50,60,100);
    m_rightDrive1.setSmartCurrentLimit(50, 60, 100);
    m_rightDrive2.setSmartCurrentLimit(50,60,100);
  }

  public void setBreakMode() {
    m_leftDrive1.setIdleMode(IdleMode.kBrake);
    m_rightDrive2.setIdleMode(IdleMode.kBrake);
    m_rightDrive1.setIdleMode(IdleMode.kBrake);
    m_rightDrive2.setIdleMode(IdleMode.kBrake);
  }

  public void setCoastMode() {
    m_leftDrive1.setIdleMode(IdleMode.kCoast);
    m_rightDrive2.setIdleMode(IdleMode.kCoast);
    m_rightDrive1.setIdleMode(IdleMode.kCoast);
    m_rightDrive2.setIdleMode(IdleMode.kCoast);
  }

  public void resetEncoders() {
    rightEncoder.reset();
    leftEncoder.reset();
  }

  public void driveTrigger(double yPositiveSpeed, double yNegativeSpeed, double xRotation) {
    if (yNegativeSpeed > 0.1) {
      m_drivetrain.arcadeDrive(-xboxController.getLeftTriggerAxis(), -xRotation);
    }
    else {
      m_drivetrain.arcadeDrive(yPositiveSpeed, -xRotation);
    }

    if (yPositiveSpeed < 0.1 || yNegativeSpeed < 0.1) {
      m_drivetrain.stopMotor();
    }

  }

  public void drive(double ySpeed, double xRotation) {
    m_drivetrain.arcadeDrive(ySpeed, -xRotation);

    if (ySpeed < 0.1 && ySpeed > -0.1) {
      m_drivetrain.stopMotor();
    }
  }

  public void limitSpeed(double maxSpeedLimit, double mediumSpeedLimit, double lowerSpeedLimit) {
    if (xboxController.getRightBumperReleased()) {
      if (rBPressedTwice) {
        m_drivetrain.setMaxOutput(maxSpeedLimit);
        rBPressedTwice = false;
      }
      else {
        if (rBPressedOnce) {
          m_drivetrain.setMaxOutput(mediumSpeedLimit);
          rBPressedOnce = false;
          rBPressedTwice = true;
        }
        else {
          m_drivetrain.setMaxOutput(lowerSpeedLimit);
          rBPressedOnce = true;
        }
      }
    }  
    
  }

  public double getRightEncoderPosition() {
    return -rightEncoder.getDistance();
  }

  public double getLeftEncoderPosition() {
    return leftEncoder.getDistance();
  }

  public double getRightEncoderVelocity() {
    return -rightEncoder.getRate();
  }

  public double getLeftEncoderVelocity() {
    return leftEncoder.getRate();
  }

  public double getTurnRate() {
    return -navX.getRate();
  }

  public static double getHeading() {
    return navX.getRotation2d().getDegrees();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(navX.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance(), getPose());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftDrive.setVoltage(leftVolts);
    m_rightDrive.setVoltage(rightVolts);
    m_drivetrain.feed();
  }

  public double getAverageEncoderDistance() {
    return ((getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0);
  }

  public Encoder getLeftEncoder() {
    return leftEncoder;
  }

  public Encoder getRightEncoder() {
    return rightEncoder;
  }

  public void setMaxOutput(double maxOutput) {
    m_drivetrain.setMaxOutput(maxOutput);
  }

  public static void zeroHeading() {
    navX.calibrate();
    navX.reset();
  }

  public Gyro getGyro() {
    return navX;
  }

  public DifferentialDriveOdometry getOdometry() {
    return m_odometry;
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    m_odometry.update(navX.getRotation2d(), leftEncoder.getDistance(),
        rightEncoder.getDistance());

    SmartDashboard.putNumber("Left encoder value meters", getLeftEncoderPosition());
    SmartDashboard.putNumber("RIGHT encoder value meters", getRightEncoderPosition());
    SmartDashboard.putNumber("Gyro heading", getHeading());
  }
}
