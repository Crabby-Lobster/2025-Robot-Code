// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static Command testTrajectory(AutoFactory autofactory) {
    AutoRoutine routine = autofactory.newRoutine("Test");

    AutoTrajectory testTraj = routine.trajectory("New Path");

    routine.active().onTrue(
      Commands.sequence(
        testTraj.resetOdometry(),
        testTraj.cmd()
      )
    );

    return routine.cmd();
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
