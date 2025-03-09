// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralArmPositions;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.ScoreSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralHome extends Command {
  CoralArm coralArm;
  /** Creates a new CoralHome. */
  public CoralHome(ScoreSystem scoreSystem) {
    this.coralArm = scoreSystem.coralArm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(scoreSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coralArm.SetPivotSpeed(0.25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralArm.SetPivotSpeed(0);
    coralArm.resetPivotPosition(CoralArmPositions.HOME);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coralArm.getHomeSwitch();
  }
}
