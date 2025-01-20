// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PathSelector;
import frc.robot.subsystems.DriveTrain;
import static frc.robot.Constants.DrivetrainConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultDrive extends Command {
  
  //The joysticks used to control the drivetrain
  Joystick LeftJoystick;
  Joystick rightJoystick;
  XboxController controller;
  DriveTrain drivetrain;
  PathSelector pathSelector;

  /** Creates a new DefaultDrive. */
  public DefaultDrive(Joystick leftJoystick, Joystick rightJoystick, DriveTrain driveTrain, XboxController controller, PathSelector pathSelector) {
    // assigns the inputs from the constructor to the variables to be used in this class
    this.LeftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;
    this.drivetrain = driveTrain;
    this.controller = controller;
    this.pathSelector = pathSelector;
    // Use addRequirements() here to declare subsystem dependencies.
    // this tell the drivetrain that it is required for this command to run
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // this calls the function from the drivetrain to set the speed of the motors
    double left_speed = -controller.getLeftY() * DriveSpeed;
    double right_speed = -controller.getRightY() * DriveSpeed;
    drivetrain.TankDrive(left_speed, right_speed, true);
    pathSelector.GetButtonPresses();
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
