// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.OperatorConstants.DrivetrainConstants.*;

public class DriveTrain extends SubsystemBase {

  // creates the motors
  SparkMax FLMotor = new SparkMax(FLMotorID, MotorType.kBrushless);
  SparkMax FRMotor = new SparkMax(FRMotorID, MotorType.kBrushless);
  SparkMax BLMotor = new SparkMax(BLMotorID, MotorType.kBrushless);
  SparkMax BRMotor = new SparkMax(BRMotorID, MotorType.kBrushless);

  // creates the encoders
  RelativeEncoder FLEncoder;
  RelativeEncoder FREncoder;
  RelativeEncoder BLEncoder;
  RelativeEncoder BREncoder;

  // creates the configurations for the motors
  SparkMaxConfig FLConfig = new SparkMaxConfig();
  SparkMaxConfig FRConfig = new SparkMaxConfig();
  SparkMaxConfig BLConfig = new SparkMaxConfig();
  SparkMaxConfig BRConfig = new SparkMaxConfig();

  // creates the groups of motors
  @SuppressWarnings("removal")
  MotorControllerGroup LeftMotors = new MotorControllerGroup(FLMotor, BLMotor);
  @SuppressWarnings("removal")
  MotorControllerGroup RightMotors = new MotorControllerGroup(FRMotor, BRMotor);

  // creates the diff drive
  DifferentialDrive tankDrive = new DifferentialDrive(LeftMotors, RightMotors);

  /** Creates a new DriveTrain.*/
  public DriveTrain() {

    // applies the configs to the motors
    //FLConfig.inverted(FLInvert).idleMode(IdleMode.kBrake).smartCurrentLimit(stallCurrentLimit, freeCurrentLimit);
    //FLConfig.encoder.positionConversionFactor(EncoderPositionConversion).velocityConversionFactor(EncoderSpeedConversion);
    //FLMotor.configure(FLConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    //FRConfig.inverted(FRInvert).idleMode(IdleMode.kBrake).smartCurrentLimit(stallCurrentLimit, freeCurrentLimit);
    //FRConfig.encoder.positionConversionFactor(EncoderPositionConversion).velocityConversionFactor(EncoderSpeedConversion);
    //FRMotor.configure(FRConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    //BLConfig.inverted(BLInvert).idleMode(IdleMode.kBrake).smartCurrentLimit(stallCurrentLimit, freeCurrentLimit);
    //BLConfig.encoder.positionConversionFactor(EncoderPositionConversion).velocityConversionFactor(EncoderSpeedConversion);
    //BLMotor.configure(BLConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    //BRConfig.inverted(BRInvert).idleMode(IdleMode.kBrake).smartCurrentLimit(stallCurrentLimit, freeCurrentLimit);
    //BRConfig.encoder.positionConversionFactor(EncoderPositionConversion).velocityConversionFactor(EncoderSpeedConversion);
    //BRMotor.configure(BRConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // retrieves encoders from speed controllers
    FLEncoder = FLMotor.getEncoder();
    FREncoder = FLMotor.getEncoder();
    BLEncoder = FLMotor.getEncoder();
    BREncoder = FLMotor.getEncoder();
  }


  /**
   * @param leftSpeed The left motor throttle
   * @param rightSpeed The right motor throttle
   */
  public void TankDrive(double leftSpeed, double rightSpeed){
    tankDrive.tankDrive(leftSpeed, rightSpeed, true);

    SmartDashboard.putNumber("Drivetrain LeftWheelThrottle", leftSpeed);
    SmartDashboard.putNumber("Drivetrain RightWheelThrottle", rightSpeed);
  }


  enum EncoderRetriaval {
    GetSpeed,
    GetDistance,
    GetLeftSpeed,
    GetRightSpeed,
    GetLeftDistance,
    GetRightDistance
  }

  /**
   * @param returnType the type of data to be returned when the function is called
   * @return the desired encoder reading
   */
  public double getEncoderValues(EncoderRetriaval returnType) {
    switch (returnType) {
      case GetSpeed:
        double avgSpeed = FLEncoder.getVelocity() + FREncoder.getVelocity() + BLEncoder.getVelocity() + BREncoder.getVelocity();
        avgSpeed /= 4.0;
        return avgSpeed;
      case GetDistance:
        double avgDist = FLEncoder.getPosition() + FREncoder.getPosition() + BLEncoder.getPosition() + BREncoder.getPosition();
        avgDist /= 4.0;
        return avgDist;
      case GetLeftDistance:
        double avgLeftDist = FLEncoder.getPosition() + BLEncoder.getPosition();
        avgLeftDist /= 2.0;
        return avgLeftDist;
      case GetLeftSpeed:
        double avgLeftSpeed = FLEncoder.getVelocity() + BLEncoder.getVelocity();
        avgLeftSpeed /= 2.0;
        return avgLeftSpeed;
      case GetRightDistance:
        double avgRightdist = FREncoder.getPosition() + BREncoder.getPosition();
        avgRightdist /= 2.0;
        return avgRightdist;
      case GetRightSpeed:
        double avgRightSpeed = FREncoder.getVelocity() + BREncoder.getVelocity();
        avgRightSpeed /= 2.0;
        return avgRightSpeed;
      default:
        return 0;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // posts drivetrain encoder data to driverstation
    SmartDashboard.putNumber("DriveTrain Speed", getEncoderValues(EncoderRetriaval.GetSpeed));
    SmartDashboard.putNumber("DriveTrain Distance", getEncoderValues(EncoderRetriaval.GetDistance));

    SmartDashboard.putNumber("DriveTrain LeftSpeed", getEncoderValues(EncoderRetriaval.GetLeftSpeed));
    SmartDashboard.putNumber("DriveTrain LeftDistance", getEncoderValues(EncoderRetriaval.GetLeftDistance));

    SmartDashboard.putNumber("DriveTrain RightSpeed", getEncoderValues(EncoderRetriaval.GetRightSpeed));
    SmartDashboard.putNumber("DriveTrain RightDistance", getEncoderValues(EncoderRetriaval.GetRightDistance));
  }
}
