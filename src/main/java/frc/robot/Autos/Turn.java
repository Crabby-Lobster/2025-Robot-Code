// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Turn extends Command {
  DriveTrain driveTrain;
  double angle;

  PIDController steeController = new PIDController(0.1, 0, 0);

  /** Creates a new Turn. */
  public Turn(DriveTrain driveTrain, double angle) {
    this.driveTrain = driveTrain;
    this.angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    steeController.setTolerance(5);
    steeController.setSetpoint(driveTrain.getHeading() + angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double steer = steeController.calculate(driveTrain.getHeading());
    driveTrain.ArcadeDrive(0, steer);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.ArcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return steeController.atSetpoint();
  }
}
