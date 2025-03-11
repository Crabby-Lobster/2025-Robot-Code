// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
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
    AutoStore m_Store;
    AutoReefLow m_reefLow;
    AutoReefHigh m_ReefHigh;
    AutoBarge m_Barge;

    AutoIntake m_Intake;
    AutoIntakeOff m_IntakeOff;
    AutoOutput m_Output;
    AutoOutputOff m_OutputOff;


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
        m_Store = new AutoStore(scoreSystem, positionContainer);
        m_reefLow = new AutoReefLow(scoreSystem, positionContainer);
        m_ReefHigh = new AutoReefHigh(scoreSystem, positionContainer);
        m_Barge = new AutoBarge(scoreSystem, positionContainer);

        m_Intake = new AutoIntake(positionContainer);
        m_IntakeOff = new AutoIntakeOff(positionContainer);
        m_Output = new AutoOutput(positionContainer);
        m_OutputOff = new AutoOutputOff(positionContainer);

        //Binds commands
            //Move commands
        autofactory.bind("Store", m_Store);
        autofactory.bind("MoveLow", m_reefLow);
        autofactory.bind("MoveHigh", m_ReefHigh);
        autofactory.bind("MoveBarge", m_Barge);
            //Intake Commands
        autofactory.bind("StopIntake", m_Intake);
        autofactory.bind("IntakeLow", Commands.parallel(m_reefLow, m_Intake));
        autofactory.bind("IntakeHigh", Commands.parallel(m_ReefHigh, m_Intake));
            //Output Commands
        autofactory.bind("StopOutput", m_OutputOff);
        autofactory.bind("Output", Commands.parallel(m_Barge, m_Output));

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
