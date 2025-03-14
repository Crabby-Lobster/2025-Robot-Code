// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgearArmPositions;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ScoreSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeHome extends Command {
  AlgaeArm algaeArm;
  Elevator elevator;
  /** Creates a new AlgaeHome. */
  public AlgaeHome(ScoreSystem scoreSystem) {
    algaeArm = scoreSystem.algaeArm;
    elevator = scoreSystem.elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(scoreSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algaeArm.setPivotSpeed(0.25);
    elevator.setSpeed(-0.1, -0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeArm.setPivotSpeed(0);
    elevator.setSpeed(0, 0);
    algaeArm.resetPivot(AlgearArmPositions.HOME);
    elevator.resetPosition(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return algaeArm.getHomeSwitch();
  }
}
