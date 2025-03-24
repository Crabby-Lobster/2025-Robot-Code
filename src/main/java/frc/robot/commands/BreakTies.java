// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgearArmPositions;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.ScoreSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BreakTies extends Command {
  AlgaeArm algaeArm;

  /** Creates a new BreakTies. */
  public BreakTies(ScoreSystem scoreSystem) {
    this.algaeArm = scoreSystem.algaeArm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(scoreSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    algaeArm.setPosition(AlgearArmPositions.STORE - 25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algaeArm.setPosition(AlgearArmPositions.STORE - 25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ( (algaeArm.getPivotPosition() < (AlgearArmPositions.STORE - 20) )) ;
  }
}
