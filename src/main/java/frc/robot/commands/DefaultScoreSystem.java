// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PositionContainer;
import frc.robot.subsystems.ScoreSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultScoreSystem extends Command {
  
  ScoreSystem scoreSystem;

  PositionContainer positionContainer;

  Joystick leftJoy;
  Joystick rightJoy;
  Joystick controller;

  double algaeOFfset = 0;

  /** Creates a new DefaultScoreSystem. */
  public DefaultScoreSystem(ScoreSystem scoreSystem, Joystick leftJoy, Joystick rightJoy, Joystick controller, PositionContainer positionContainer) {
    this.scoreSystem = scoreSystem;
    this.leftJoy = leftJoy;
    this.rightJoy = rightJoy;
    this.controller = controller;
    this.positionContainer = positionContainer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(scoreSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    positionContainer.updateInputs();
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
    return false;
  }
}
