// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PositionContainer;
import frc.robot.PositionContainer.States;
import frc.robot.subsystems.ScoreSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoStore extends Command {
  ScoreSystem scoreSystem;
  PositionContainer positionContainer;

  /** Creates a new AutoStore. */
  public AutoStore(ScoreSystem scoreSystem, PositionContainer positionContainer) {
    this.scoreSystem = scoreSystem;
    this.positionContainer = positionContainer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(scoreSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    positionContainer.setState(States.kStore);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    positionContainer.updateLogic();
    scoreSystem.setState(positionContainer.GetState());
    scoreSystem.update();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return positionContainer.moveComplete;
  }
}
