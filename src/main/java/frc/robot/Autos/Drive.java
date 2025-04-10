// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.EncoderRetriaval;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Drive extends Command {
  DriveTrain driveTrain;
  double distance;
  double direction;

  PIDController drivController = new PIDController(5, 0, 0);
  PIDController steercontroller = new PIDController(0.1, 0, 0);
  /** Creates a new Drive. */
  public Drive(DriveTrain driveTrain, double distance) {
    this.driveTrain = driveTrain;
    this.distance = distance / 39.37;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetEncoder(0);

    drivController.setTolerance(1 / 39.37);

    drivController.setSetpoint(distance);
    steercontroller.setSetpoint(driveTrain.getHeading());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double driveval = drivController.calculate(driveTrain.getEncoderValues(EncoderRetriaval.GetDistance));
    double steerVal = steercontroller.calculate(driveTrain.getHeading());

    driveTrain.ArcadeDrive(MathUtil.clamp(driveval, -0.5, 0.5), steerVal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.ArcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivController.atSetpoint();
  }
}
