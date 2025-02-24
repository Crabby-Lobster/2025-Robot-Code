// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ElevatorConstants.*;

/**
 * The elevator for the robot
 */
public class Elevator extends SubsystemBase {
  // Motors
  SparkMax leftM = new SparkMax(leftMID, MotorType.kBrushless);
  SparkMax rightM = new SparkMax(rightMID, MotorType.kBrushless);

  // Encoders
  RelativeEncoder leftEnc;
  RelativeEncoder rightEnc;

  // Limit switch
  DigitalInput elevatorLimit = new DigitalInput(elevatorLimitswitch);

  // PIDs
  SparkClosedLoopController leftController;
  SparkClosedLoopController rightController;

  /** Creates a new Elevator. */
  public Elevator() {
    // motor configs
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();

    leftConfig.inverted(leftInvert).idleMode(IdleMode.kBrake);
    rightConfig.inverted(rightInvert).idleMode(IdleMode.kBrake);

    leftConfig.encoder.positionConversionFactor(positionConversion).velocityConversionFactor(velocityConversion);
    rightConfig.encoder.positionConversionFactor(positionConversion).velocityConversionFactor(velocityConversion);

    leftConfig.closedLoop.pid(elevatorPID[0], elevatorPID[1], elevatorPID[2]);
    rightConfig.closedLoop.pid(elevatorPID[0], elevatorPID[1], elevatorPID[2]);

    leftM.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightM.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftEnc = leftM.getEncoder();
    rightEnc = rightM.getEncoder();

    leftController = leftM.getClosedLoopController();
    rightController = rightM.getClosedLoopController();
  }

  /**
   * Returns whether the limit switch is pressed
   * @return the value of the limit switch
   */
  public boolean getLimitSwitch() {
    return !elevatorLimit.get();
  }

  /**
   * gets the position of the elevator in inches
   * @return elevator height in inches
   */
  public double getHeight(){
    return (leftEnc.getPosition() + rightEnc.getPosition()) / 2.0;
  }

  /**
   * sets the speed of the elevator motors
   * @param speed the speed the elevator moves
   */
  public void setSpeed(double speed) {
    leftM.set(speed);
    rightM.set(speed);
  }

  /**
   * sets the position of the elevator
   * @param position the position the elevator will move to
   */
  public void setPosition(double position) {
    leftController.setReference(position, ControlType.kPosition);
    rightController.setReference(position, ControlType.kPosition);
  }

  /**
   * resets the position of the encoders
   * @param position the position to reset the encoders to
   */
  public void resetPosition(double position) {
    leftEnc.setPosition(position);
    rightEnc.setPosition(position);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
