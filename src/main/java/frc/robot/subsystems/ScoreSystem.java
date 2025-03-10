// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ScoreSystemState;
import frc.robot.ScoreSystemState.RollerState;
import frc.robot.commands.AlgaeHome;
import frc.robot.commands.BreakTies;
import frc.robot.commands.ElevatorHome;
import frc.robot.Constants.AlgearArmPositions;
import frc.robot.Constants.ElevatorPositions;

public class ScoreSystem extends SubsystemBase {

  // parts
  public Elevator elevator;
  public AlgaeArm algaeArm;

  //States
  private ScoreSystemState desiredState = new ScoreSystemState();
  private ScoreSystemState safeState = new ScoreSystemState();
  public ScoreSystemState currentState = new ScoreSystemState();

  /** Creates a new ScoreSystem. */
  public ScoreSystem(Elevator elevator, AlgaeArm algaeArm) {
    this.elevator = elevator;
    this.algaeArm = algaeArm;
  }


  /**
   * sets the desired state for the score system
   * @param state the desired state
   */
  public void setState(ScoreSystemState state, double algaeOffset) {
    state.algeaArmPos += algaeOffset;
    desiredState = state;
  }

  /**
   * updates the score system.
   * has to be called for the system to work
   */
  public void update() {
    // check safties
    checkElevatorSaftey();
    checkAlgaeSaftey();

    // updates subsystems
    elevator.setPosition(safeState.elevatorPos);
    algaeArm.setPosition(safeState.algeaArmPos);

    algaeArm.updateRollers(safeState.algaeMode);

  }


  /**
   * checks to make sure the elevator wont cause any collisions
   */
  private void checkElevatorSaftey() {
    double desiredPosition = desiredState.elevatorPos;
    
    // Clamps to elevator position
    desiredPosition = MathUtil.clamp(desiredPosition - ElevatorPositions.OFFSET, ElevatorPositions.HOME, ElevatorPositions.MAXHEIGHT());
    
    //keeps elevator clear of algae arm
    double algaeAngle = Math. min(desiredState.algeaArmPos, currentState.algeaArmPos);
    double[] algaeClearence = algaeArm.getSafeHeight(algaeAngle);

    desiredPosition = MathUtil.clamp(desiredPosition, algaeClearence[0], algaeClearence[1]);


    safeState.setElevator(desiredPosition);
  }


  /**
   * checks to make sure the algae arm wont cause any collisions
   */
  private void checkAlgaeSaftey() {
    double desiredPosition = desiredState.algeaArmPos;
    RollerState desiredMode = desiredState.algaeMode;

    desiredPosition = MathUtil.clamp(desiredPosition, AlgearArmPositions.MINANGLE, AlgearArmPositions.HOME);
    
    safeState.setAlgaeArm(desiredPosition, desiredMode);
  }

  public SequentialCommandGroup HomeSystems(ScoreSystem scoresystem) {
    return new SequentialCommandGroup(
      new BreakTies(scoresystem),
      new AlgaeHome(scoresystem),
      new ElevatorHome(scoresystem)
    );
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentState.setElevator(elevator.getHeight());
    currentState.setAlgaeArm(algaeArm.getPivotPosition(), safeState.algaeMode, algaeArm.getAlgeaSwitch());

    SmartDashboard.putNumber("algae", currentState.algeaArmPos);
    SmartDashboard.putNumber("Elevator", currentState.elevatorPos);
    SmartDashboard.putNumber("Desired algae", desiredState.algeaArmPos);
    SmartDashboard.putNumber("Desired Elavator", desiredState.elevatorPos);
  }
}
