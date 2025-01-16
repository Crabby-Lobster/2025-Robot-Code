// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;

import static frc.robot.Constants.OperatorConstants.DrivetrainConstants.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static Command testTrajectory() {
    
    // voltage constraint for ramsete controller
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
        ksVolts,
        kvVoltSecondsPerMeter,
        kaVoltSecondsSquaredPerMeter
      ),
      kDriveKinematics,
      ramseteMaxV
    );


  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
