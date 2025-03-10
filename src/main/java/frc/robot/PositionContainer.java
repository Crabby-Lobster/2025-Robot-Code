// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.AlgearArmPositions;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.ScoreSystemState.RollerState;
import frc.robot.subsystems.ScoreSystem;

/** Holds positions manual offsets for use during gameplay */
public class PositionContainer {

    //Scoresystem
    ScoreSystem scoreSystem;

    //Controllers
    Joystick leftJoy;
    Joystick rightJoy;
    Joystick Controller;

    // Scoresystem states
    ScoreSystemState currentState = new ScoreSystemState();
    ScoreSystemState rawState = new ScoreSystemState();
    ScoreSystemState desiredState = new ScoreSystemState();

    // The state identifier
    States activeSystemState = States.Store;

    //Offsets
    double elevatorOffset = 0;
    double algaeArmOffset = 0;

    // booleans
    public boolean moveComplete = false;

    public boolean intake = false;
    public boolean output = false;

    private boolean hold = false;

    public enum States {
        Store,
        Ground,
        Proccesor,
        GroundHigh,
        ReefLow,
        ReefHigh,
        Barge
    }
    
    /** Creates new positionContainer */
    public PositionContainer(Joystick leftJoy, Joystick rightJoy, Joystick controller, ScoreSystem scoreSystem) {
        this.leftJoy = leftJoy;
        this.rightJoy = rightJoy;
        this.Controller = controller;
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
        if (Controller.getRawButtonPressed(1)) {
            activeSystemState = States.Store;
        }
        //Ground
        else if (Controller.getRawButtonPressed(2)) {
            activeSystemState = States.Ground;
        }
        //Proccesor
        else if (Controller.getRawButtonPressed(3)) {
            activeSystemState = States.Proccesor;
        }
        //GroundHigh
        else if (Controller.getRawButtonPressed(4)) {
            activeSystemState = States.GroundHigh;
        }
        //ReefLow
        else if (Controller.getRawButtonPressed(5)) {
            activeSystemState = States.ReefLow;
        }
        //ReefHigh
        else if (Controller.getRawButtonPressed(6)) {
            activeSystemState = States.ReefHigh;
        }
        //Barge
        else if (Controller.getRawButtonPressed(7)) {
            activeSystemState = States.Barge;
        }

        //intake
        intake = leftJoy.getRawButton(1);
        output = leftJoy.getRawButton(6);

        //Manual
        algaeArmOffset = Controller.getY() * 10;
    }

    /**
     * Updates the internal logic for the path positions
     */
    public void updateLogic() {
        moveComplete = false;
        currentState = scoreSystem.currentState;

        switch (activeSystemState) {
            case Ground:
                updateGround();
                break;
            case Proccesor:
                updateProccesor();
                break;
            case GroundHigh:
                updateGroundHigh();
                break;
            case Store:
                updateStore();
                break;
            case ReefLow:
                updateReefLow();
                break;
            case ReefHigh:
                updateReefHigh();
                break;
            case Barge:
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
        rawState.algeaArmPos = AlgearArmPositions.Ground;
        rawState.elevatorPos = ElevatorPositions.Ground;
        moveComplete = true;
    }

    private void updateProccesor() {
        rawState.algeaArmPos = AlgearArmPositions.Proccesor;
        rawState.elevatorPos = ElevatorPositions.Proccesor;
        moveComplete = true;
    }

    private void updateGroundHigh() {
        rawState.algeaArmPos = AlgearArmPositions.GroundHigh;
        rawState.elevatorPos = ElevatorPositions.GroundHigh;
        moveComplete = true;
    }

    private void updateStore() {
        rawState.algeaArmPos = AlgearArmPositions.STORE;
        rawState.elevatorPos = ElevatorPositions.STORE;
        moveComplete = true;
    }

    private void updateReefLow() {
        rawState.algeaArmPos = AlgearArmPositions.ReefLow;
        rawState.elevatorPos = ElevatorPositions.ReefLow;
        moveComplete = true;
    }

    private void updateReefHigh() {
        rawState.algeaArmPos = AlgearArmPositions.ReefHigh;
        rawState.elevatorPos = ElevatorPositions.ReefHigh;
        moveComplete = true;
    }

    private void updateBarge() {
        rawState.algeaArmPos = AlgearArmPositions.Barge;
        rawState.elevatorPos = ElevatorPositions.Barge;
        moveComplete = true;
    }
}
