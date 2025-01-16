// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;

import static frc.robot.Constants.OperatorConstants.DrivetrainConstants.*;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static Command testTrajectory() {
    
    // voltage constraint for ramsete controller
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
        ksVolts,
        kvVoltSecondsPerMeter,
        kaVoltSecondsSquaredPerMeter
      ),
      kDriveKinematics,
      ramseteMaxV
    );

    // trajectory config
    TrajectoryConfig config =
      new TrajectoryConfig(
        kMaxSpeedMeterPerSeconds,
        kMaxAccelerationMeterPerSecondSquared)
      //add kinematics to ensure max speed is obeyed
      .setKinematics(kDriveKinematics)
      // apply voltage constraint
      .addConstraint(autoVoltageConstraint);
    
    // test trajectory to follow
    Trajectory exampleTrajectory = 
      TrajectoryGenerator.generateTrajectory(
        //start point
        new Pose2d(0,0, new Rotation2d(0)),
        
        // interior points making S-curve
        List.of(new Translation2d(0.5,0.5), new Translation2d(1.5,-0.5)),

        // end point 2 meters infront of start point
        new Pose2d(2,0, new Rotation2d(0)),
        config
      );

  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
