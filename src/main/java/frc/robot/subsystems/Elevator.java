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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ElevatorConstants.*;

public class Elevator extends SubsystemBase {
  SparkMax leftM = new SparkMax(leftMID, MotorType.kBrushless);
  SparkMax rightM = new SparkMax(rightMID, MotorType.kBrushless);

  RelativeEncoder leftEnc;
  RelativeEncoder rightEnc;

  DigitalInput elevatorLimit = new DigitalInput(elevatorLimitswitch);

  PIDController elevatorposPID = new PIDController(elevatorPID[0], elevatorPID[1], elevatorPID[2]);

  /** Creates a new Elevator. */
  public Elevator() {
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();

    leftConfig.inverted(leftInvert).idleMode(IdleMode.kBrake);
    rightConfig.inverted(rightInvert).idleMode(IdleMode.kBrake);

    leftConfig.encoder.positionConversionFactor(positionConversion).velocityConversionFactor(velocityConversion);
    rightConfig.encoder.positionConversionFactor(positionConversion).velocityConversionFactor(velocityConversion);

    leftM.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightM.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftEnc = leftM.getEncoder();
    rightEnc = rightM.getEncoder();
  }

  public boolean getLimitSwitch() {
    return elevatorLimit.get();
  }

  public double getHeight(){
    return (leftEnc.getPosition() + rightEnc.getPosition()) / 2.0;
  }

  public void setSpeed(double speed) {
    leftM.set(speed);
    rightM.set(speed);
  }

  public void setPosition(double position) {
    double throttle = elevatorposPID.calculate(getHeight(), position);
    leftM.set(throttle);
    rightM.set(throttle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
