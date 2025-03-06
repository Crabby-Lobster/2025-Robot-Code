// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ScoreSystemState;
import frc.robot.Constants.AlgearArmPositions;
import frc.robot.Constants.CoralArmPositions;
import frc.robot.ScoreSystemState.RollerState;
import frc.robot.subsystems.ScoreSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultScoreSystem extends Command {
  
  ScoreSystem scoreSystem;

  Joystick leftJoy;
  Joystick rightJoy;
  XboxController controller;


  //temp
  double position = CoralArmPositions.HOME;

  /** Creates a new DefaultScoreSystem. */
  public DefaultScoreSystem(ScoreSystem scoreSystem, Joystick leftJoy, Joystick rightJoy, XboxController controller) {
    this.scoreSystem = scoreSystem;
    this.leftJoy = leftJoy;
    this.rightJoy = rightJoy;
    this.controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(scoreSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    position = position + (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis());

    SmartDashboard.putNumber("Desired position", position);

    // state
    ScoreSystemState desiredState = new ScoreSystemState();

    desiredState.setElevator(0);
    desiredState.setAlgaeArm(AlgearArmPositions.HOME, RollerState.kIdle);
    desiredState.setCoralArm(position, RollerState.kIdle);

    scoreSystem.setState(desiredState);
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
