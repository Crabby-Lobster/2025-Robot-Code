// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.AlgearArmPositions;
import frc.robot.Constants.CoralArmPositions;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.ScoreSystemState.RollerState;
import frc.robot.subsystems.AlgaeArm;

/** Add your docs here. */
public class ScoresystemPositionContainer {
    Joystick leftJoy;
    Joystick righJoy;
    Joystick controller;

    boolean algaeHold = false;
    boolean coralHold = false;

    ScoreSystemState DesiredState = new ScoreSystemState();

    double[][] AlgaePositions = {
        {AlgearArmPositions.STORE, ElevatorPositions.HOME},
        {AlgearArmPositions.GROUNDINTAKE, ElevatorPositions.AlgaeGround},
        {AlgearArmPositions.INTAKE, ElevatorPositions.AlgaeIntake},
        {AlgearArmPositions.SCORE, ElevatorPositions.AlgaeScore}
    };

    double[][] CoralPositions = {
        {CoralArmPositions.STORE, ElevatorPositions.HOME},
        {CoralArmPositions.SCORE, ElevatorPositions.L1Coral},
        {CoralArmPositions.SCORE, ElevatorPositions.L2Coral},
        {CoralArmPositions.SCORE, ElevatorPositions.L3Coral}
    };

    public ScoresystemPositionContainer(Joystick leftJoy, Joystick rightJoy, Joystick controller) {
        this.leftJoy = leftJoy;
        this.righJoy = rightJoy;
        this.controller = controller;
    }


    public ScoreSystemState getState(ScoreSystemState currentState) {
        // POSITIONS
        // THE STORED POSITION
        if (controller.getRawButtonPressed(2)) {
            DesiredState.elevatorPos = ElevatorPositions.HOME;
            DesiredState.coralArmPos = CoralPositions[0][0];
            DesiredState.algeaArmPos = AlgaePositions[0][0];
        }
        // ALGAE GROUND
        else if (controller.getRawButtonPressed(7)) {
            DesiredState.elevatorPos = AlgaePositions[1][1];
            DesiredState.coralArmPos = CoralPositions[0][0];
            DesiredState.algeaArmPos = AlgaePositions[1][0];
        }
        // ALGAE INTAKE
        else if (controller.getRawButtonPressed(9)) {
            DesiredState.elevatorPos = AlgaePositions[2][1];
            DesiredState.coralArmPos = CoralPositions[0][0];
            DesiredState.algeaArmPos = AlgaePositions[2][0];
        }
        // ALGAE SCORE
        else if (controller.getRawButtonPressed(11)) {
            DesiredState.elevatorPos = AlgaePositions[3][1];
            DesiredState.coralArmPos = CoralPositions[0][0];
            DesiredState.algeaArmPos = AlgaePositions[3][0];
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
