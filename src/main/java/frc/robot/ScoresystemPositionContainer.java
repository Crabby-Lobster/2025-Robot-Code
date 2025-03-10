// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.AlgearArmPositions;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.ScoreSystemState.RollerState;

/** The container for Saved positions and the logic to switch between them */
public class ScoresystemPositionContainer {
    Joystick leftJoy;
    Joystick righJoy;
    Joystick controller;

    boolean algaeHold = false;
    boolean AlgaeHigh = true;

    ScoreSystemState DesiredState = new ScoreSystemState();

    AlgaePosition algaePositions = new AlgaePosition();

    /**Creates the position container */
    public ScoresystemPositionContainer(Joystick leftJoy, Joystick rightJoy, Joystick controller) {
        this.leftJoy = leftJoy;
        this.righJoy = rightJoy;
        this.controller = controller;
    }

    /**
     * Contains the arm position and height for a specific saved position
     * @param armPos The saved arm position
     * @param elevatorHeight The saved height for the elevator
     */
    private class ArmPositions {
        public double armPos;
        public double elevatorHeight;
        
        /**
         * Creates new ArmPosition
         * @param arm the arm position
         * @param elevator the arm height
         */
        public ArmPositions(double arm, double elevator) {
            this.armPos = arm;
            this.elevatorHeight = elevator;
        }
    }

    /**
     * contains the saved positions for the algae arm
     * @param Store The stored position of the arm
     * @param Ground the ground intake position
     * @param Intake The reef intake position
     * @param Score the verticale scoring position
     */
    private class AlgaePosition {
        public ArmPositions Store = new ArmPositions(AlgearArmPositions.STORE, ElevatorPositions.HOME);
        public ArmPositions Ground = new ArmPositions(AlgearArmPositions.GROUNDINTAKE, ElevatorPositions.AlgaeGround);
        public ArmPositions Intake = new ArmPositions(AlgearArmPositions.INTAKE, ElevatorPositions.AlgaeIntake);
        public ArmPositions IntakeHigh = new ArmPositions(AlgearArmPositions.INTAKEHigh, ElevatorPositions.AlgaeIntakeHigh);
        public ArmPositions Score = new ArmPositions(AlgearArmPositions.SCORE, ElevatorPositions.AlgaeScore);
    }


    /**
     * contains the internal logic for selecting arm position
     * @param currentState The currant scoresystem state
     * @return the desired score system state
     */
    public ScoreSystemState getState(ScoreSystemState currentState) {
        // POSITIONS
        // THE STORED POSITION
        if (controller.getRawButtonPressed(2)) {
            DesiredState.elevatorPos = ElevatorPositions.OFFSET;

            if (currentState.algeaArmFull) {
                DesiredState.algeaArmPos = algaePositions.Score.armPos;
            } else {
                DesiredState.algeaArmPos = algaePositions.Store.armPos;
            }
        }
        // ALGAE GROUND
        else if (controller.getRawButtonPressed(7)) {
            DesiredState.elevatorPos = algaePositions.Ground.elevatorHeight;
            DesiredState.algeaArmPos = algaePositions.Ground.armPos;
        }
        // ALGAE INTAKE
        else if (controller.getRawButtonPressed(9)) {
            AlgaeHigh = !AlgaeHigh;
            if (AlgaeHigh) {
                DesiredState.elevatorPos = algaePositions.IntakeHigh.elevatorHeight;
                DesiredState.algeaArmPos = algaePositions.IntakeHigh.armPos;
            } else {
                DesiredState.elevatorPos = algaePositions.Intake.elevatorHeight;
                DesiredState.algeaArmPos = algaePositions.Intake.armPos;
            }
        }
        // ALGAE SCORE
        else if (controller.getRawButtonPressed(11)) {
            DesiredState.elevatorPos = algaePositions.Score.elevatorHeight;
            DesiredState.algeaArmPos = algaePositions.Score.armPos;
        }

        //Intakes Algae
        if (leftJoy.getRawButton(1)) {
            DesiredState.algaeMode = RollerState.kIntake;
            algaeHold = true;
        } else if (leftJoy.getRawButton(6)) {
            DesiredState.algaeMode = RollerState.kScore;
            algaeHold = false;
        } else if (algaeHold) {
            DesiredState.algaeMode = RollerState.kHold;
        } else {
            DesiredState.algaeMode = RollerState.kIdle;
        }

        return DesiredState;
    }
}
