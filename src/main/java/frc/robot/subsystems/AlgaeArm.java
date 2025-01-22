// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.AlgaeArmConstants.*;

public class AlgaeArm extends SubsystemBase {
  /** Creates a new AlgaeArm. */
  VictorSPX spxPivot = new VictorSPX(pivotNum);
  VictorSPX spxRoller = new VictorSPX(rollerNum);
  DigitalInput pivotLimitSwitch = new DigitalInput(pLimitSwitchNum);
  Encoder pivotEncoder = new Encoder(pEncodeNum[0], pEncodeNum[1]);
  PIDController pivotPosition = new PIDController(rollerNum, pivotNum, pLimitSwitchNum);
  public void setPivotSpeed(double speed) {
    spxPivot.set(VictorSPXControlMode.PercentOutput, speed);
  }
  public void setRollerSpeed(double speed) {
    spxRoller.set(VictorSPXControlMode.PercentOutput, speed);
  }
  public void setPivotPosition(double angle) {
    double position = pivotPosition.calculate(getPivotPosition(),angle);
    spxPivot.set(VictorSPXControlMode.PercentOutput, position);
  }
  public double getPivotPosition() {
    return pivotEncoder.getDistance();
  }
  public boolean getLimitSwitch() {
    return pivotLimitSwitch.get();
  }
  public AlgaeArm() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
