// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ScoreSystemState;
import frc.robot.ScoreSystemState.RollerState;

public class ScoreSystem extends SubsystemBase {

  // parts
  Elevator elevator;

  //States
  private ScoreSystemState desiredState = new ScoreSystemState();
  private ScoreSystemState safeState = new ScoreSystemState();
  private ScoreSystemState currentState = new ScoreSystemState();

  /** Creates a new ScoreSystem. */
  public ScoreSystem(Elevator elevator) {
    this.elevator = elevator;
  }


  /**
   * sets the desired state for the score system
   * @param state the desired state
   */
  public void setState(ScoreSystemState state) {
    desiredState = state;
  }

  /**
   * updates the score system.
   * has to be called for the system to work
   */
  public void update() {
    // check safties
    checkElevatorSaftey();
    checkCoralSaftey();
    checkAlgaeSaftey();

    // updates subsystems
    elevator.setPosition(safeState.elevatorPos);

    // updates current state
    currentState.setElevator(elevator.getHeight());
    currentState.setCoralArm(0, safeState.algaeMode, false);
    currentState.setAlgaeArm(0, safeState.algaeMode, false);
  }


  /**
   * checks to make sure the elevator wont cause any collisions
   */
  private void checkElevatorSaftey() {
    double desiredPosition = desiredState.elevatorPos;
    
    safeState.setElevator(desiredPosition);
  }

  /**
   * checks to make sure the coral arm wont cause any collisions
   */
  private void checkCoralSaftey() {
    double desiredPosition = desiredState.coralArmPos;
    RollerState desiredMode = desiredState.coralMode;
    
    safeState.setCoralArm(desiredPosition, desiredMode);
  }

  /**
   * checks to make sure the algae arm wont cause any collisions
   */
  private void checkAlgaeSaftey() {
    double desiredPosition = desiredState.coralArmPos;
    RollerState desiredMode = desiredState.coralMode;
    
    safeState.setCoralArm(desiredPosition, desiredMode);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentState.setElevator(elevator.getHeight());
  }
}
