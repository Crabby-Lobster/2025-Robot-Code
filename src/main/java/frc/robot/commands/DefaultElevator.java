// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/**
 * controls the elevator during teleop
 */
public class DefaultElevator extends Command {
  Joystick leftJoystick;
  Joystick rightJoystick;
  XboxController controller;

  Elevator elevator;

  double position = 0;
  /** Creates a new DefaultElevator. */
  public DefaultElevator(Elevator elevator, Joystick leftstick, Joystick rightstick, XboxController controller) {
    this.elevator = elevator;

    this.leftJoystick = leftstick;
    this.rightJoystick = rightstick;
    this.controller = controller;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    position += (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()) * 0.1;
    elevator.setPosition(position);;
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
