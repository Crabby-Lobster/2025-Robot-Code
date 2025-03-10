// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

/** Holds positions manual offsets for use during gameplay */
public class PositionContainer {

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
    public PositionContainer(Joystick leftJoy, Joystick rightJoy, Joystick controller) {
        this.leftJoy = leftJoy;
        this.rightJoy = rightJoy;
        this.Controller = controller;
    }

    /**
     * supplies the current state of the scoresystem
     * @param currenState the current state of the scoresystem
     */
    public void updateCurrentState(ScoreSystemState currenState) {
        this.currentState = currenState;
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
        else if (Controller.getRawButtonPressed(1)) {
            activeSystemState = States.Ground;
        }
        //Proccesor
        else if (Controller.getRawButtonPressed(1)) {
            activeSystemState = States.Proccesor;
        }
        //GroundHigh
        else if (Controller.getRawButtonPressed(1)) {
            activeSystemState = States.GroundHigh;
        }
        //ReefLow
        else if (Controller.getRawButtonPressed(1)) {
            activeSystemState = States.ReefLow;
        }
        //ReefHigh
        else if (Controller.getRawButtonPressed(1)) {
            activeSystemState = States.ReefHigh;
        }
        //Barge
        else if (Controller.getRawButtonPressed(1)) {
            activeSystemState = States.Barge;
        }
    }

    /**
     * Updates the internal logic for the path positions
     */
    public void updateLogic() {
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
        return desiredState;
    }


    private void updateGround() {
    }

    private void updateProccesor() {
    }

    private void updateGroundHigh() {
    }

    private void updateStore() {
    }

    private void updateReefLow() {
    }

    private void updateReefHigh() {
    }

    private void updateBarge() {
    }
}
