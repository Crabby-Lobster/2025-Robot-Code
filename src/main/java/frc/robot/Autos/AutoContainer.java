// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PositionContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ScoreSystem;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.deadline;

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
        m_Store = new AutoStore(positionContainer);
        m_reefLow = new AutoReefLow(positionContainer);
        m_ReefHigh = new AutoReefHigh(positionContainer);
        m_Barge = new AutoBarge(positionContainer);

        m_Intake = new AutoIntake(positionContainer);
        m_IntakeOff = new AutoIntakeOff(positionContainer);
        m_Output = new AutoOutput(positionContainer);
        m_OutputOff = new AutoOutputOff(positionContainer);


    }

    public Command Test() {
        return new Drive(drivetrain, 12);
    }
}
