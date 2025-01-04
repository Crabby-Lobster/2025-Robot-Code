// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.OperatorConstants.DrivetrainConstants.*;

public class DriveTrain extends SubsystemBase {

  // creates the motors
  SparkMax FLMotor = new SparkMax(FLMotorID, MotorType.kBrushless);
  SparkMax FRMotor = new SparkMax(FRMotorID, MotorType.kBrushless);
  SparkMax BLMotor = new SparkMax(BLMotorID, MotorType.kBrushless);
  SparkMax BRMotor = new SparkMax(BRMotorID, MotorType.kBrushless);

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
    FLConfig.inverted(FLInvert).idleMode(IdleMode.kBrake).smartCurrentLimit(stallCurrentLimit, freeCurrentLimit);
    FLConfig.encoder.positionConversionFactor(0).velocityConversionFactor(0);
    FLMotor.configure(FLConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    FRConfig.inverted(FRInvert).idleMode(IdleMode.kBrake).smartCurrentLimit(stallCurrentLimit, freeCurrentLimit);
    FRConfig.encoder.positionConversionFactor(0).velocityConversionFactor(0);
    FRMotor.configure(FRConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    BLConfig.inverted(BLInvert).idleMode(IdleMode.kBrake).smartCurrentLimit(stallCurrentLimit, freeCurrentLimit);
    BLConfig.encoder.positionConversionFactor(0).velocityConversionFactor(0);
    BLMotor.configure(BLConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    BRConfig.inverted(BRInvert).idleMode(IdleMode.kBrake).smartCurrentLimit(stallCurrentLimit, freeCurrentLimit);
    BRConfig.encoder.positionConversionFactor(0).velocityConversionFactor(0);
    BRMotor.configure(BRConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }


  /**
   * @param leftSpeed The left motor throttle
   * @param rightSpeed The right motor throttle
   */
  public void TankDrive(double leftSpeed, double rightSpeed){
    tankDrive.tankDrive(leftSpeed, rightSpeed, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
