// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.PositionContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ScoreSystem;

/** Container for holding auto programs */
public class AutoContainer {

    //subsytems
    DriveTrain drivetrain;
    ScoreSystem scoreSystem;
    PositionContainer positionContainer;

    //Autofactory
    AutoFactory autofactory;

    //Helper Commands
    AutoReefLow m_reefLow;


    public AutoContainer(DriveTrain driveTrain, ScoreSystem scoreSystem, PositionContainer positionContainer) {
        this.drivetrain = driveTrain;
        this.scoreSystem = scoreSystem;
        this.positionContainer = positionContainer;

        //Sets up auto factory
        autofactory = new AutoFactory(
        driveTrain::getPose,
        driveTrain::resetOdometry,
        driveTrain::followTrajectory,
        true,
        driveTrain
        );

        // sets up helper commands
        m_reefLow = new AutoReefLow(scoreSystem, positionContainer);
    }

    public AutoRoutine Taxi() {
        AutoRoutine TaxiRoutine = autofactory.newRoutine("Taxi");
        AutoTrajectory MoveTraj = TaxiRoutine.trajectory("Taxi");

        //Starts the trajectory
        TaxiRoutine.active().onTrue(Commands.sequence(
            Commands.print("Started Taxi"),
            MoveTraj.resetOdometry(),
            MoveTraj.cmd()
        ));

        // moves to reef loow at the end of that trajectory
        MoveTraj.done().onTrue(m_reefLow);

        return TaxiRoutine;
    }
}
