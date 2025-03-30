// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AlgearArmPositions;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.ScoreSystemState.RollerState;
import frc.robot.subsystems.ScoreSystem;

/** Holds positions manual offsets for use during gameplay */
public class PositionContainer {

    //Scoresystem
    ScoreSystem scoreSystem;

    //Controllers
    XboxController driver;
    Joystick operator;

    // Scoresystem states
    ScoreSystemState currentState = new ScoreSystemState();
    ScoreSystemState rawState = new ScoreSystemState();
    ScoreSystemState desiredState = new ScoreSystemState();

    // The state identifier
    States activeSystemState = States.kStore;

    //Offsets
    double elevatorOffset = 0;
    double algaeArmOffset = 0;

    // booleans
    public boolean moveComplete = false;

    public boolean intake = false;
    public boolean output = false;

    private boolean hold = false;

    public enum States {
        kStore,
        kGround,
        kGroundHigh,
        kReefLow,
        kReefHigh,
        kBarge
    }
    
    /** Creates new positionContainer */
    public PositionContainer(XboxController driver, Joystick operator, ScoreSystem scoreSystem) {
        this.driver = driver;
        this.operator = operator;
        this.scoreSystem = scoreSystem;
    }

    public void setState(States state) {
        activeSystemState = state;
    }
    /**
     * updates the internal inputs for position changing
     */
    public void updateInputs() {
        //Store
        if (operator.getRawButtonPressed(2)) {
            activeSystemState = States.kStore;
        }
        //Ground
        else if (operator.getRawButtonPressed(11)) {
            activeSystemState = States.kGround;
        }
        //GroundHigh
        else if (operator.getRawButtonPressed(1)) {
            activeSystemState = States.kGroundHigh;
        }
        //ReefLow
        else if (operator.getRawButtonPressed(9)) {
            activeSystemState = States.kReefLow;
        }
        //ReefHigh
        else if (operator.getRawButtonPressed(10)) {
            activeSystemState = States.kReefHigh;
        }
        //Barge
        else if (operator.getRawButtonPressed(7)) {
            activeSystemState = States.kBarge;
        }

        //intake
        intake = operator.getRawButton(3);
        output = operator.getRawButton(4);

        //Manual
        algaeArmOffset = operator.getY() * 20;
    }

    /**
     * Updates the internal logic for the path positions
     */
    public void updateLogic() {
        moveComplete = false;
        currentState = scoreSystem.currentState;

        switch (activeSystemState) {
            case kGround:
                updateGround();
                break;
            case kGroundHigh:
                updateGroundHigh();
                break;
            case kStore:
                updateStore();
                break;
            case kReefLow:
                updateReefLow();
                break;
            case kReefHigh:
                updateReefHigh();
                break;
            case kBarge:
                updateBarge();
                break;
        }

        //intake control
        if (intake) {
            desiredState.algaeMode = RollerState.kIntake;
            hold = true;
        } else if (output) {
            desiredState.algaeMode = RollerState.kScore;
            hold = false;
        } else if (hold) {
            desiredState.algaeMode = RollerState.kHold;
        } else {
            desiredState.algaeMode = RollerState.kIdle;
        }
    }

    /**
     * returns the state after modifications and offsets
     * @return the desired state of the arm
     */
    public ScoreSystemState GetState() {
        desiredState.elevatorPos = rawState.elevatorPos + elevatorOffset;
        desiredState.algeaArmPos = rawState.algeaArmPos + algaeArmOffset;

        return desiredState;
    }


    private void updateGround() {
        if (currentState.algeaArmFull) {
            rawState.algeaArmPos = AlgearArmPositions.Ground + 5;
            rawState.elevatorPos = ElevatorPositions.Ground;   
        } else {
            rawState.algeaArmPos = AlgearArmPositions.Ground;
            rawState.elevatorPos = ElevatorPositions.Ground;
        }
        moveComplete = true;
    }

    private void updateGroundHigh() {
        rawState.algeaArmPos = AlgearArmPositions.GroundHigh;
        rawState.elevatorPos = ElevatorPositions.GroundHigh;
        moveComplete = true;
    }

    private void updateStore() {
        if (currentState.algeaArmFull) {
            rawState.algeaArmPos = AlgearArmPositions.STORE - 5;
            rawState.elevatorPos = ElevatorPositions.STORE;
        } else {
            rawState.algeaArmPos = AlgearArmPositions.STORE;
            rawState.elevatorPos = ElevatorPositions.STORE;
        }

        moveComplete = true;
    }

    private void updateReefLow() {
        if (currentState.algeaArmFull) {
            rawState.algeaArmPos = AlgearArmPositions.ReefLow + 10;
            rawState.elevatorPos = ElevatorPositions.ReefLow;
            moveComplete = true;
        } else {
            rawState.algeaArmPos = AlgearArmPositions.ReefLow;
            rawState.elevatorPos = ElevatorPositions.ReefLow;
        }
    }

    private void updateReefHigh() {
        if (currentState.algeaArmFull) {
            rawState.algeaArmPos = AlgearArmPositions.ReefHigh + 10;
            rawState.elevatorPos = ElevatorPositions.ReefHigh;
            moveComplete = true;
        } else {
            rawState.algeaArmPos = AlgearArmPositions.ReefHigh;
            rawState.elevatorPos = ElevatorPositions.ReefHigh;
        }
    }

    private void updateBarge() {
        rawState.algeaArmPos = AlgearArmPositions.Barge;
        rawState.elevatorPos = ElevatorPositions.Barge;
        moveComplete = true;
    }


    public boolean hasBall() {
        return scoreSystem.algaeArm.getAlgeaSwitch();
    }
}
