// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  TalonFX ClimberMotor = new TalonFX(30);
  /** Creates a new Climber. */
  public Climber() {
    TalonFXConfiguration climberConfig = new TalonFXConfiguration();
    climberConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    climberConfig.Feedback.FeedbackRemoteSensorID = ClimberMotor.getDeviceID();
    climberConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    climberConfig.Feedback.SensorToMechanismRatio = 1;
    climberConfig.Feedback.RotorToSensorRatio = 1;

    ClimberMotor.getConfigurator().apply(climberConfig);
  }

  public double getDistance() {
    return ClimberMotor.getPosition().getValueAsDouble();
  }

  public void setSpeed(double speed) {
    ClimberMotor.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Position", ClimberMotor.getPosition().getValueAsDouble());
    // This method will be called once per scheduler run
  }
}
