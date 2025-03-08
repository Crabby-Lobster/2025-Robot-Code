// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ScoreSystem;

/**
 * moves the elevator down until it reaches home position
 * command ends once limit switch is triggered
 */
public class ElevatorHome extends Command {

  Elevator elevator;

  /** Creates a new ElevatorHome. */
  public ElevatorHome(ScoreSystem scoreSystem) {
    elevator = scoreSystem.elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(scoreSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("ElevatorHoming", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setSpeed(-0.01,-0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setSpeed(0, -0.1);
    elevator.resetPosition(ElevatorPositions.HOME);
    SmartDashboard.putBoolean("ElevatorHoming", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return elevator.getLimitSwitch();
    return true;
  }
}
