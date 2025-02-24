// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.CoralArmConstants.*;

public class CoralArm extends SubsystemBase {
  /** Creates a new CoralArm. */
  public CoralArm() {

  }

  //Motors
  SparkMax coralPivot = new SparkMax(pivotID, MotorType.kBrushless);
  VictorSPX lRoller = new VictorSPX(rollerLID);
  VictorSPX rRoller = new VictorSPX(rollerRID);

  // Limit switches
  DigitalInput homeSwitch = new DigitalInput(HomeSwitchID);
  DigitalInput coralSwitch = new DigitalInput(coralSwitchID);

  // Encoders
  RelativeEncoder PivotEncoder;

  // PID
  SparkClosedLoopController PivotPID;

  public void SetPivotSpeed(double speed) {
    coralPivot.set(speed);
  }

  public void SetRollerSpeed(double speed) {
    coralPivot.set(speed);
  }

  public void setPivotPosition(double angle) {
    PivotPID.setReference(angle, ControlType.kPosition);
  }

  public boolean getHomeSwitch() {
    return homeSwitch.get();
  }

  public boolean getCoralSwitch() {
    return coralSwitch.get();
  }

  public double getPivotPosition(){
    return PivotEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
