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
import frc.robot.Constants.OperatorConstants.DrivetrainConstants;

public class DriveTrain extends SubsystemBase {

  SparkMax FLMotor = new SparkMax(DrivetrainConstants.FLMotorID, MotorType.kBrushless);
  SparkMax FRMotor = new SparkMax(DrivetrainConstants.FRMotorID, MotorType.kBrushless);
  SparkMax BLMotor = new SparkMax(DrivetrainConstants.BLMotorID, MotorType.kBrushless);
  SparkMax BRMotor = new SparkMax(DrivetrainConstants.BRMotorID, MotorType.kBrushless);

  SparkMaxConfig FLConfig = new SparkMaxConfig();
  SparkMaxConfig FRConfig = new SparkMaxConfig();
  SparkMaxConfig BLConfig = new SparkMaxConfig();
  SparkMaxConfig BRConfig = new SparkMaxConfig();

  @SuppressWarnings("removal")
  MotorControllerGroup LeftMotors = new MotorControllerGroup(FLMotor, BLMotor);
  @SuppressWarnings("removal")
  MotorControllerGroup RightMotors = new MotorControllerGroup(FRMotor, BRMotor);

  DifferentialDrive tankDrive = new DifferentialDrive(LeftMotors, RightMotors);

  /** Creates a new DriveTrain.*/
  public DriveTrain() {

    FLConfig.inverted(DrivetrainConstants.FLInvert).idleMode(IdleMode.kBrake);
    FLConfig.encoder.positionConversionFactor(0).velocityConversionFactor(0);
    FLMotor.configure(FLConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    FRConfig.inverted(DrivetrainConstants.FRInvert).idleMode(IdleMode.kBrake);
    FRConfig.encoder.positionConversionFactor(0).velocityConversionFactor(0);
    FRMotor.configure(FRConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    BLConfig.inverted(DrivetrainConstants.BLInvert).idleMode(IdleMode.kBrake);
    BLConfig.encoder.positionConversionFactor(0).velocityConversionFactor(0);
    BLMotor.configure(BLConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    BRConfig.inverted(DrivetrainConstants.BRInvert).idleMode(IdleMode.kBrake);
    BRConfig.encoder.positionConversionFactor(0).velocityConversionFactor(0);
    BRMotor.configure(BRConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void TankDrive(double leftSpeed, double rightSpeed){
    tankDrive.tankDrive(leftSpeed, rightSpeed, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
