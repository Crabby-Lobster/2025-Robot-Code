// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultAlgaeArm extends Command {
  Joystick lJoystick;
  Joystick rJoystick;
  XboxController controller;

  AlgaeArm algaeArm;
  /** Creates a new DefaultAlgaeArm. */
  public DefaultAlgaeArm(AlgaeArm algaeArm, Joystick leftstick, Joystick rightstick, XboxController controller) {
    this.algaeArm = algaeArm;

    this.lJoystick = leftstick;
    this.rJoystick = rightstick;
    this.controller = controller;

    addRequirements(algaeArm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algaeArm.setRollerSpeed(.25);
    algaeArm.setPivotSpeed(.25);
    
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
