// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/**
 * moves the elevator down until it reaches home position
 * command ends once limit switch is triggered
 */
public class ElevatorHome extends Command {

  Elevator elevator;

  /** Creates a new ElevatorHome. */
  public ElevatorHome(Elevator elevator) {
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setSpeed(-0.25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setSpeed(0);
    elevator.resetPosition(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return elevator.getLimitSwitch();
    return true;
  }
}
