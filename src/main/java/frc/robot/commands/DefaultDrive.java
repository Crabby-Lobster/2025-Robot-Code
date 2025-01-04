// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants.DrivetrainConstants;
import frc.robot.subsystems.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultDrive extends Command {
  
  Joystick LeftJoystick;
  Joystick rightJoystick;
  DriveTrain drivetrain;

  /** Creates a new DefaultDrive. */
  public DefaultDrive(Joystick leftJoystick, Joystick rightJoystick, DriveTrain driveTrain) {
    this.LeftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;
    this.drivetrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left_speed = LeftJoystick.getY() * DrivetrainConstants.DriveSpeed;
    double right_speed = rightJoystick.getY() * DrivetrainConstants.DriveSpeed;
    drivetrain.TankDrive(left_speed, right_speed);
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
