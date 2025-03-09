// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.AlgearArmPositions;
import frc.robot.Constants.CoralArmPositions;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.ScoreSystemState.RollerState;

/** The container for Saved positions and the logic to switch between them */
public class ScoresystemPositionContainer {
    Joystick leftJoy;
    Joystick righJoy;
    Joystick controller;

    boolean algaeHold = false;
    boolean AlgaeHigh = true;
    boolean coralHold = false;

    ScoreSystemState DesiredState = new ScoreSystemState();

    AlgaePosition algaePositions = new AlgaePosition();
    CoralPosition coralPositions = new CoralPosition();

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
     * Contains the saved positions for the coral arm
     * @param Intake The Coral feeder station intake position
     * @param Store The stored position
     * @param L1 The Level 1 Coral
     * @param L2 The level 2 Coral
     * @param L3 The level 3 Coral
    */
    private class CoralPosition {
        public ArmPositions Intake = new ArmPositions(CoralArmPositions.INTAKE, ElevatorPositions.CoralIntake);
        public ArmPositions Store = new ArmPositions(CoralArmPositions.STORE, ElevatorPositions.HOME);
        public ArmPositions L1 = new ArmPositions(CoralArmPositions.SCORE, ElevatorPositions.L1Coral);
        public ArmPositions L2 = new ArmPositions(CoralArmPositions.SCORE, ElevatorPositions.L2Coral);
        public ArmPositions L3 = new ArmPositions(CoralArmPositions.SCORE, ElevatorPositions.L3Coral);
        public ArmPositions AlgaeIntake = new ArmPositions(CoralArmPositions.AlgaeIntakeGround, ElevatorPositions.HOME);
        public ArmPositions AlgaeIntakeOther = new ArmPositions(CoralArmPositions.AlgaeIntakeOther, ElevatorPositions.HOME);
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
                DesiredState.coralArmPos = coralPositions.AlgaeIntakeOther.armPos;
                DesiredState.algeaArmPos = algaePositions.Score.armPos;
            } else {
                DesiredState.coralArmPos = coralPositions.Store.armPos;
                DesiredState.algeaArmPos = algaePositions.Store.armPos;
            }
        }
        // ALGAE GROUND
        else if (controller.getRawButtonPressed(7)) {
            DesiredState.elevatorPos = algaePositions.Ground.elevatorHeight;
            DesiredState.coralArmPos = coralPositions.AlgaeIntake.armPos;
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
            DesiredState.coralArmPos = coralPositions.AlgaeIntakeOther.armPos;
        }
        // ALGAE SCORE
        else if (controller.getRawButtonPressed(11)) {
            DesiredState.elevatorPos = algaePositions.Score.elevatorHeight;
            DesiredState.coralArmPos = coralPositions.AlgaeIntakeOther.armPos;
            DesiredState.algeaArmPos = algaePositions.Score.armPos;
        }
        // Coral L1
        else if (controller.getRawButtonPressed(8)) {
            DesiredState.elevatorPos = coralPositions.L1.elevatorHeight;
            DesiredState.coralArmPos = coralPositions.L1.armPos;
            DesiredState.algeaArmPos = algaePositions.Store.armPos;
        }
        // Coral L2
        else if (controller.getRawButtonPressed(10)) {
            DesiredState.elevatorPos = coralPositions.L2.elevatorHeight;
            DesiredState.coralArmPos = coralPositions.L2.armPos;
            DesiredState.algeaArmPos = algaePositions.Store.armPos;
        }
        // Coral L3
        else if (controller.getRawButtonPressed(12)) {
            DesiredState.elevatorPos = coralPositions.L3.elevatorHeight;
            DesiredState.coralArmPos = coralPositions.L3.armPos;
            DesiredState.algeaArmPos = algaePositions.Store.armPos;
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

        //Intakes Coral
        if (righJoy.getRawButton(1)) {
            // sets arm to coral intake positon when intake button is pressed
            DesiredState.elevatorPos = coralPositions.Intake.elevatorHeight;
            DesiredState.coralArmPos = coralPositions.Intake.armPos;
            DesiredState.algeaArmPos = algaePositions.Store.armPos;

            DesiredState.coralMode = RollerState.kIntake;
            coralHold = true;
        } else if (righJoy.getRawButton(6)) {
            DesiredState.coralMode = RollerState.kScore;
            coralHold = false;
        } else if (coralHold) {
            DesiredState.coralMode = RollerState.kHold;
        } else {
            DesiredState.coralMode = RollerState.kIdle;
        }

        return DesiredState;
    }
}
