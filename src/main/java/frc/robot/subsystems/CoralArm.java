// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.CoralArmConstants.*;

public class CoralArm extends SubsystemBase {
  /** Creates a new CoralArm. */
  VictorSPX spxPivot = new VictorSPX(pivotNum);
  VictorSPX spxRoller = new VictorSPX(rollerNum);
  Encoder pivotEncoder = new Encoder(pEncode[0], pEncode[1]);
  DigitalInput pivotLimitSwitch = new DigitalInput(pivotNum);
  PIDController pivotPID = new PIDController(pivotNum, rollerNum, pivotNum);

  public void SetPivotSpeed(double speed) {
    spxPivot.set(VictorSPXControlMode.PercentOutput, speed);
  }
  public void SetRollerSpeed(double speed) {
    spxRoller.set(VictorSPXControlMode.PercentOutput, speed);
  }
  public void setPivotPosition(double angle) {
    double position = pivotPID.calculate(getPivotPosition(), angle);
    SetPivotSpeed(position);
  }
  public boolean getLimitSwitch() {
    return pivotLimitSwitch.get();
  }
  public double getPivotPosition(){
    return pivotEncoder.getDistance();
  }
  public CoralArm() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
