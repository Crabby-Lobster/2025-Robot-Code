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

    WaitUntilBall m_WaitUntilBall;
    Wait m_Wait;


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

        m_WaitUntilBall = new WaitUntilBall(positionContainer);
        m_Wait = new Wait();


    }

    public Command Test() {
        return sequence(
            //drives foward 12 inches
            new Drive(drivetrain, 12),

            //turns on intake
            m_Intake,

            // waits until ball is in wheels
            m_WaitUntilBall,

            // turns around and drives 12 inches
            new Turn(drivetrain, 180),
            new Drive(drivetrain, 12),

            //turns 180 and shuts the intake off
            new Turn(drivetrain, 180),
            m_IntakeOff
        );
    }

    public Command AlgaeScoreAuto() {
        return sequence(
            
            //Turns Toward reef
            new Turn(drivetrain, 32).withTimeout(5),

            //Drives toward reef
            new Drive(drivetrain, 80),

            //Lines up with ball
            new Turn(drivetrain, -32).withTimeout(5),

            //raises arm and collects
            parallel(m_reefLow, m_Intake),

            // picks up ball and reverses
            new Drive(drivetrain, 12),
            new Drive(drivetrain, -12),

            //lowers arm
            parallel(m_Store, m_IntakeOff),

            //turns toward barge
            new Turn(drivetrain, 151).withTimeout(5),

            //drives toward barge
            new Drive(drivetrain, 60),

            //lines up for barge and raises elevator
            parallel(new Turn(drivetrain, 30).withTimeout(5), m_Barge),

            //shoots
            deadline(m_Wait.withTimeout(3), m_Output),
            m_OutputOff
        );
    }
}
