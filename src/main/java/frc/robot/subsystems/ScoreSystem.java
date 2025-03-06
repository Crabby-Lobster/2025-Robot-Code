// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ScoreSystemState;
import frc.robot.ScoreSystemState.RollerState;
import frc.robot.commands.AlgaeHome;
import frc.robot.commands.CoralHome;
import frc.robot.commands.ElevatorHome;
import frc.robot.Constants.AlgearArmPositions;
import frc.robot.Constants.ElevatorPositions;

public class ScoreSystem extends SubsystemBase {

  // parts
  public Elevator elevator;
  public CoralArm coralArm;
  public AlgaeArm algaeArm;

  //States
  private ScoreSystemState desiredState = new ScoreSystemState();
  private ScoreSystemState safeState = new ScoreSystemState();
  private ScoreSystemState currentState = new ScoreSystemState();

  /** Creates a new ScoreSystem. */
  public ScoreSystem(Elevator elevator, CoralArm coralArm, AlgaeArm algaeArm) {
    this.elevator = elevator;
    this.coralArm = coralArm;
    this.algaeArm = algaeArm;
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
    coralArm.setPosition(safeState.coralArmPos);
    algaeArm.setPosition(safeState.algeaArmPos);

    coralArm.updateRollers(safeState.coralMode);
    algaeArm.updateRollers(safeState.algaeMode);

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
    
    desiredPosition = MathUtil.clamp(desiredPosition - ElevatorPositions.OFFSET, ElevatorPositions.HOME, ElevatorPositions.MAXHEIGHT());
    
    //keeps elevator clear of algae arm
    double algaeAngle = Math.max(desiredState.algeaArmPos, currentState.algeaArmPos);
    double[] algaeClearence = algaeArm.getSafeHeight(algaeAngle);

    // keeps elevator clear of coral arm
    double coralAngle = Math.max(desiredState.coralArmPos, currentState.coralArmPos);
    double[] coralClearence = coralArm.getSafeHeight(coralAngle);

    desiredPosition = MathUtil.clamp(desiredPosition, algaeClearence[0], algaeClearence[1]);
    desiredPosition = MathUtil.clamp(desiredPosition, coralClearence[0], coralClearence[1]);

    safeState.setElevator(desiredPosition);
  }

  /**
   * checks to make sure the coral arm wont cause any collisions
   */
  private void checkCoralSaftey() {
    double desiredPosition = desiredState.coralArmPos;
    RollerState desiredMode = desiredState.coralMode;

    desiredPosition = MathUtil.clamp(desiredPosition, AlgearArmPositions.MINANGLE, AlgearArmPositions.HOME);
    
    safeState.setCoralArm(desiredPosition, desiredMode);
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
      new CoralHome(scoresystem),
      new AlgaeHome(scoresystem),
      new ElevatorHome(scoresystem)
    );
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentState.setElevator(elevator.getHeight());
    currentState.setCoralArm(coralArm.getPivotPosition(), safeState.coralMode, coralArm.getCoralSwitch());
    currentState.setAlgaeArm(algaeArm.getPivotPosition(), safeState.algaeMode, algaeArm.getAlgeaSwitch());
  }
}
