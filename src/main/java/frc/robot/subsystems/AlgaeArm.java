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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.AlgaeArmConstants.*;

public class AlgaeArm extends SubsystemBase {
  /** Creates a new AlgaeArm. */
  SparkMax algeaPivot = new SparkMax(pivotID, MotorType.kBrushless);

  VictorSPX lRoller = new VictorSPX(rollerLID);
  VictorSPX rRoller = new VictorSPX(rollerRID);

  DigitalInput homeSwitch = new DigitalInput(HomeSwitchID);
  DigitalInput algeaSwitch = new DigitalInput(AlgeaSwitchID);

  RelativeEncoder pivotEnc;

  SparkClosedLoopController pivotPID;

  public AlgaeArm() {}

  public void setPivotSpeed(double speed) {
    algeaPivot.set(speed);
  }

  public void setRollerSpeed(double speed) {
    lRoller.set(VictorSPXControlMode.PercentOutput, speed);
    rRoller.set(VictorSPXControlMode.PercentOutput, speed);
  }

  public void setPivotPosition(double angle) {
    pivotPID.setReference(angle, ControlType.kPosition);
  }

  public double getPivotPosition() {
    return pivotEnc.getPosition();
  }

  public boolean getHomeSwitch() {
    return homeSwitch.get();
  }

  public boolean getAlgeaSwitch() {
    return algeaSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
