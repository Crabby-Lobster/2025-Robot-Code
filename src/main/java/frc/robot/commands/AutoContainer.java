// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class AutoContainer {
    // the autofactory
    AutoFactory autofactory;

    // list of subsystems to be used by the auto programs
    DriveTrain drivetrain;

    // creates the auto chooser
    AutoChooser autoChooser;

    public AutoContainer(AutoFactory autofactory, DriveTrain drivetrain) {
        this.autofactory = autofactory;
        this.drivetrain = drivetrain;

        autoChooser = new AutoChooser();

        autoChooser.addRoutine("Test", this::test);

        SmartDashboard.putData("Auto Chooster", autoChooser);
    }

    public AutoRoutine test() {
        // creates the routine
        AutoRoutine testRoutine = autofactory.newRoutine("Test");

        // creates the trajectory
        AutoTrajectory testTrajectory = testRoutine.trajectory("New Path");

        // sets up the commands for the routine
        testRoutine.active().onTrue(
            Commands.sequence(
                testTrajectory.resetOdometry(),
                testTrajectory.cmd()
            )
        );

        return testRoutine;
    }

    public Command getAutoCommand() {
        return autoChooser.selectedCommandScheduler();
    }
}
