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
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ScoreSystemState;
import frc.robot.Constants.AlgearArmPositions;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.ScoreSystemState.RollerState;

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


  public AlgaeArm() {
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.inverted(pivotInvert).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    pivotConfig.encoder.positionConversionFactor(pivotPosConversion);
    pivotConfig.closedLoop.pid(PIDValues[0], PIDValues[1], PIDValues[2]);

    algeaPivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotEnc = algeaPivot.getEncoder();
    pivotPID = algeaPivot.getClosedLoopController();

    lRoller.setInverted(pivotInvert);
    rRoller.setInverted(pivotInvert);

    resetPivot(AlgearArmPositions.STORE);
  }

  public void setPivotSpeed(double speed) {
    algeaPivot.set(speed);
  }

  public void setRollerSpeed(double speed) {
    lRoller.set(VictorSPXControlMode.PercentOutput, speed);
    rRoller.set(VictorSPXControlMode.PercentOutput, speed);
  }

  public void setPosition(double angle) {
    pivotPID.setReference(angle, ControlType.kPosition);
  }

  public double getPivotPosition() {
    return pivotEnc.getPosition();
  }

  public boolean getHomeSwitch() {
    return !homeSwitch.get();
  }

  public boolean getAlgeaSwitch() {
    return !algeaSwitch.get();
  }

  public void resetPivot(double position) {
    pivotEnc.setPosition(position);
  }

  public double[] getSafeHeight(double dangerAngle) {
    double dangerAngleStart = AlgearArmPositions.dangerAngles[0];
    double dangerAngleEnd = AlgearArmPositions.dangerAngles[1];

    double elevatorMin = AlgearArmPositions.dangerHeight[0];
    double elevatorMax = AlgearArmPositions.dangerHeight[1];

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
        setRollerSpeed(0);
        break;
      case kIntake:
        setRollerSpeed(IntakeSpeed);
        break;
      case kScore:
        setRollerSpeed(ScoreSpeed);
        break;
      case kHold:
        if (getAlgeaSwitch()) {
          setRollerSpeed(0);
        } else {
          setRollerSpeed(HoldSpeed);
        }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Algae Pivot", getPivotPosition());

    SmartDashboard.putBoolean("Algae", getHomeSwitch());
  }
}
