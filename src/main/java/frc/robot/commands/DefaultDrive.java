// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import static frc.robot.Constants.DrivetrainConstants.*;

/**
 * controls drivetrain during teleop
 */
public class DefaultDrive extends Command {
  
  //The joysticks used to control the drivetrain
  XboxController driver;
  Joystick operator;
  DriveTrain drivetrain;

  /** Creates a new DefaultDrive. */
  public DefaultDrive(XboxController driver, DriveTrain driveTrain, Joystick operator) {
    // assigns the inputs from the constructor to the variables to be used in this class
    this.driver = driver;
    this.drivetrain = driveTrain;
    this.operator = operator;
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
    double foward = -driver.getLeftY() * DriveSpeed;
    double turn = -driver.getRightX() * DriveSpeed * 1.5;
    drivetrain.ArcadeDrive(foward, turn);
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
