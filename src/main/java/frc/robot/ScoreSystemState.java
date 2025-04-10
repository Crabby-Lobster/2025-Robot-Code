// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AlgearArmPositions;
import frc.robot.Constants.ElevatorPositions;

/** A State for the scoresystem */
public class ScoreSystemState {

    public enum RollerState {
        kIntake,
        kScore,
        kIdle,
        kHold
    }

    // elevator
    public double elevatorPos = ElevatorPositions.HOME;

    // algea arm
    public double algeaArmPos = AlgearArmPositions.STORE;
    public RollerState algaeMode = RollerState.kIdle;
    public boolean algeaArmFull = false;

    /** Creates a new ScoreState */
    public ScoreSystemState() {}


    /**
     * sets the values for the elevator
     * @param height the height of the elevator in inches
     */
    public void setElevator(double height) {
        elevatorPos = height;
    }

    /**
     * sets the values for the algae arm
     * @param angle the angle of the arm
     * @param mode the roller mode of the arm
     */
    public void setAlgaeArm(double angle, RollerState mode) {
        setAlgaeArm(angle, mode, false);
    }

    /**
     * sets the values for the algae arm
     * @param angle the angle of the arm
     * @param mode the roller mode of the arm
     * @param full if the arm is holding a peice
     */
    public void setAlgaeArm(double angle, RollerState mode, boolean full) {
        algeaArmPos = angle;
        algaeMode = mode;
        algeaArmFull = full;
    }

    /**
     * lineraly interpolates between from and to by factor
     * @param from the start value
     * @param to the end value
     * @param factor the factor to lerp by
     * @return the lerped value
     */
    public static double lerp(double from, double to, double factor) {
        return from + factor * (to - from);
    }
    
    public static double remap(double in, double inMin, double inMax, double outMin, double outMax) {
        return outMin + (outMax - outMin) * ((in - inMin) / (inMax - inMin));
    }
}
