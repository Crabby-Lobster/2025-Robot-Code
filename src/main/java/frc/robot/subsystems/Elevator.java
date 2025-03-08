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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorPositions;

import static frc.robot.Constants.ElevatorConstants.*;

/**
 * The elevator for the robot
 */
public class Elevator extends SubsystemBase {
  // Motors
  SparkMax leftM = new SparkMax(leftMoterID, MotorType.kBrushless);
  SparkMax rightM = new SparkMax(rightMoterId, MotorType.kBrushless);
  SparkMax stage2M = new SparkMax(stage2MotorId, MotorType.kBrushless);

  // Encoders
  RelativeEncoder leftEnc;
  RelativeEncoder rightEnc;
  RelativeEncoder stage2Enc;

  // Limit switch
  DigitalInput elevatorLimit = new DigitalInput(elevatorLimitswitch);

  // PIDs
  SparkClosedLoopController leftController;
  SparkClosedLoopController rightController;
  SparkClosedLoopController stage2Controller;

  /** Creates a new Elevator. */
  public Elevator() {
    // motor configs
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    SparkMaxConfig stage2Config = new SparkMaxConfig();

    leftConfig.inverted(leftInvert).idleMode(IdleMode.kBrake).smartCurrentLimit(stallCurrentLimit);
    rightConfig.inverted(rightInvert).idleMode(IdleMode.kBrake).smartCurrentLimit(stallCurrentLimit);
    stage2Config.inverted(stage2invert).idleMode(IdleMode.kBrake).smartCurrentLimit(stallCurrentLimit);

    leftConfig.encoder.positionConversionFactor(stage1positionConversion);
    rightConfig.encoder.positionConversionFactor(stage1positionConversion);
    stage2Config.encoder.positionConversionFactor(stage2positionConversion);

    leftConfig.closedLoop.pid(stage1elevatorPID[0], stage1elevatorPID[1], stage1elevatorPID[2]);
    rightConfig.closedLoop.pid(stage1elevatorPID[0], stage1elevatorPID[1], stage1elevatorPID[2]);
    stage2Config.closedLoop.pid(stage2elevatorPID[0], stage2elevatorPID[1], stage2elevatorPID[2]);

    leftM.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightM.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    stage2M.configure(stage2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftEnc = leftM.getEncoder();
    rightEnc = rightM.getEncoder();
    stage2Enc = stage2M.getEncoder();

    leftController = leftM.getClosedLoopController();
    rightController = rightM.getClosedLoopController();
    stage2Controller = stage2M.getClosedLoopController();

    resetPosition(ElevatorPositions.HOME);
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
    return ((leftEnc.getPosition() + rightEnc.getPosition()) / 2.0) + stage2Enc.getPosition();
  }

  /**
   * sets the speed of the elevator motors
   * @param stage1Speed the speed of the first stage
   * @param stage2Speed the speed of the second stage
   */
  public void setSpeed(double stage1Speed, double stage2Speed) {
    leftM.set(stage1Speed);
    rightM.set(stage1Speed);
    stage2M.set(stage2Speed);
  }

  /**
   * sets the position of the elevator
   * @param position the position the elevator will move to
   */
  public void setPosition(double position) {
    double stage1pos = MathUtil.clamp(position, ElevatorPositions.HOME, ElevatorPositions.STAGE1TOP);
    double stage2pos = MathUtil.clamp(position - stage1pos, ElevatorPositions.HOME, ElevatorPositions.STAGE2TOP);

    leftController.setReference(stage1pos, ControlType.kPosition);
    rightController.setReference(stage1pos, ControlType.kPosition);

    stage2Controller.setReference(stage2pos, ControlType.kPosition);
  }

  /**
   * resets the position of the encoders
   * @param position the position to reset the encoders to
   */
  public void resetPosition(double position) {

    leftEnc.setPosition(position);
    rightEnc.setPosition(position);
    
    stage2Enc.setPosition(position);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("ElevatorHeight", getHeight());

    SmartDashboard.putBoolean("elevator home", getLimitSwitch());
  }
}
