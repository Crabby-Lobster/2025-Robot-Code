// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.CoralArmConstants.*;

public class CoralArm extends SubsystemBase {

  //Motors
  SparkMax coralPivot = new SparkMax(pivotID, MotorType.kBrushless);
  VictorSPX lRoller = new VictorSPX(rollerLID);
  VictorSPX rRoller = new VictorSPX(rollerRID);

  // Limit switches
  DigitalInput homeSwitch = new DigitalInput(HomeSwitchID);
  DigitalInput coralSwitch = new DigitalInput(coralSwitchID);

  // Encoders
  RelativeEncoder pivotEncoder;

  // PID
  SparkClosedLoopController pivotPID;

  /** Creates a new CoralArm. */
  public CoralArm() {
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.inverted(pivotInvert).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    pivotConfig.encoder.positionConversionFactor(pivotPosConversion);
    pivotConfig.closedLoop.pid(PIDValues[0], PIDValues[1], PIDValues[2]);

    coralPivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotEncoder = coralPivot.getEncoder();
    pivotPID = coralPivot.getClosedLoopController();
  }

  public void SetPivotSpeed(double speed) {
    coralPivot.set(speed);
  }

  public void SetRollerSpeed(double speed) {
    coralPivot.set(speed);
  }

  public void setPosition(double angle) {
    pivotPID.setReference(angle, ControlType.kPosition);
  }

  public boolean getHomeSwitch() {
    return homeSwitch.get();
  }

  public boolean getCoralSwitch() {
    return coralSwitch.get();
  }

  public double getPivotPosition(){
    return pivotEncoder.getPosition();
  }

  public void resetPivotPosition (double position) {
    pivotEncoder.setPosition(position);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Coral Arm Pos", getPivotPosition());
  }
}
