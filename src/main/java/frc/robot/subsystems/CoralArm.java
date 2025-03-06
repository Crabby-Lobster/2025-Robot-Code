// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralArmPositions;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.ScoreSystemState.RollerState;
import frc.robot.ScoreSystemState;

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

    lRoller.setInverted(pivotInvert);
    rRoller.setInverted(!pivotInvert);

    resetPivotPosition(CoralArmPositions.HOME);
  }

  public void SetPivotSpeed(double speed) {
    coralPivot.set(speed);
  }

  public void setRollerSpeed(double speed) {
    lRoller.set(ControlMode.PercentOutput, speed);
    rRoller.set(ControlMode.PercentOutput, speed);
  }

  public void setPosition(double angle) {
    pivotPID.setReference(angle, ControlType.kPosition);
  }

  public boolean getHomeSwitch() {
    return !homeSwitch.get();
  }

  public boolean getCoralSwitch() {
    return !coralSwitch.get();
  }

  public double getPivotPosition(){
    return pivotEncoder.getPosition();
  }

  public void resetPivotPosition (double position) {
    pivotEncoder.setPosition(position);
  }

  public double[] getSafeHeight(double dangerAngle) {
    double dangerAngleStart = CoralArmPositions.dangerAngles[0];
    double dangerAngleEnd = CoralArmPositions.dangerAngles[1];

    double elevatorMin = CoralArmPositions.dangerHeight[0];
    double elevatorMax = CoralArmPositions.dangerHeight[1];

    double lerpVal = ScoreSystemState.remap(dangerAngle, dangerAngleStart, dangerAngleEnd, 0, 1);

    lerpVal = MathUtil.clamp(lerpVal, 0, 1);

    lerpVal = ScoreSystemState.lerp(elevatorMin, elevatorMax, lerpVal);

    double[] returnValues = {
      lerpVal,
      ElevatorPositions.MAXHEIGHT()
    };

    return returnValues;
  }

  public void updateRollers(RollerState rollerState) {
    switch (rollerState) {
      case kIdle:
        setRollerSpeed(HoldSpeed);
        break;
      case kIntake:
        setRollerSpeed(IntakeSpeed);
        break;
      case kScore:
        setRollerSpeed(ScoreSpeed);
        break;
      case kHold:
        if (getCoralSwitch()) {
          setRollerSpeed(0);
        } else {
          setRollerSpeed(HoldSpeed);
        }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Coral Arm Pos", getPivotPosition());

    SmartDashboard.putBoolean("Coral limit", getHomeSwitch());
  }
}
