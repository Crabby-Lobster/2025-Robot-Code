// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ScoreSystem;

/** Container for holding auto programs */
public class AutoContainer {

    //subsytems
    DriveTrain drivetrain;
    ScoreSystem scoreSystem;

    //Autofactory
    AutoFactory autofactory;

    //Helper Commands


    public AutoContainer(DriveTrain driveTrain, ScoreSystem scoreSystem, AutoFactory autoFactory) {
        this.drivetrain = driveTrain;
        this.scoreSystem = scoreSystem;
        this.autofactory = autoFactory;
    }

    public Command Taxi() {
        return Commands.sequence(
            autofactory.resetOdometry("Taxi"),
            autofactory.trajectoryCmd("Taxi")
        );
    }
}
