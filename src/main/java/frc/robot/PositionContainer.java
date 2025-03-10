// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.AlgearArmPositions;
import frc.robot.Constants.ElevatorPositions;
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
    }

    /**
     * Updates the internal logic for the path positions
     */
    public void updateLogic() {
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
    }

    private void updateProccesor() {
        rawState.algeaArmPos = AlgearArmPositions.Proccesor;
        rawState.elevatorPos = ElevatorPositions.Proccesor;
    }

    private void updateGroundHigh() {
        rawState.algeaArmPos = AlgearArmPositions.GroundHigh;
        rawState.elevatorPos = ElevatorPositions.GroundHigh;
    }

    private void updateStore() {
        rawState.algeaArmPos = AlgearArmPositions.STORE;
        rawState.elevatorPos = ElevatorPositions.STORE;
    }

    private void updateReefLow() {
        rawState.algeaArmPos = AlgearArmPositions.ReefLow;
        rawState.elevatorPos = ElevatorPositions.ReefLow;
    }

    private void updateReefHigh() {
        rawState.algeaArmPos = AlgearArmPositions.ReefHigh;
        rawState.elevatorPos = ElevatorPositions.ReefHigh;
    }

    private void updateBarge() {
        rawState.algeaArmPos = AlgearArmPositions.Barge;
        rawState.elevatorPos = ElevatorPositions.Barge;
    }
}
