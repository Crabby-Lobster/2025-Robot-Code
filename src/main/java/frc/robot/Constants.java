// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static java.lang.Math.PI;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /**
   * the values for controller inputs
   */
  public static class ControllerConstants {
    public static final int LeftJoystick = 0;
    public static final int rightJoystick = 1;
    public static final int controller = 2;
  }

  /**
   * values for the drivetrain subsystem
   */
  public static final class DrivetrainConstants {
      
    //Motor CAN Ids
    public static final int FLMotorID = 2;
    public static final int FRMotorID = 7;
    public static final int BLMotorID = 3;
    public static final int BRMotorID = 6;

    //gearbox and wheel constants
    private static final double overallGearboxRatio = (1.0 / 10.71);

    /**
     *wheel diameter in meters
      */
    private static final double wheelDiameter = 0.1524;

    //encoder values
    public static final double EncoderPositionConversion = overallGearboxRatio * wheelDiameter * PI;
    public static final double EncoderSpeedConversion = (1.0 / 60.0) * wheelDiameter * PI * overallGearboxRatio;

    // the motors invert status
    public static final boolean FLInvert = false;
    public static final boolean FRInvert = true;
    public static final boolean BLInvert = false;
    public static final boolean BRInvert = false;

    // current limit for motors
    public static final int stallCurrentLimit = 60;
    public static final int freeCurrentLimit = 40;

    // the throttle multiplier for manual driving
    public static final double DriveSpeed = 0.5;

    // diff drive constants
    public static final double kTrackwidthMeter = 0.60325;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeter);
  }

  /**
   * values for the elevator subsytem
   */
  public static final class ElevatorConstants {
    public static final int leftMID = 4;
    public static final int rightMID = 5;

    public static final boolean leftInvert = false;
    public static final boolean rightInvert = true;

    public static final int highCurrentLimit = 40;

    public static final double positionConversion = (1.0 / 12.0) * 1.75 * PI;

    public static final int elevatorLimitswitch = 9;

    public static final double[] elevatorPID = {0.1,0,0};
  }

  /**
   * stored elevator positions
   */
  public static final class ElevatorPositions {
    public static final double TOP = 32;

    public static final double L4Coral = 0;
    public static final double L3Coral = 0;
    public static final double L2Coral = 0;

    public static final double AlgaePickup = 0;

    public static final double CoralPickup = 0;
    
    public static final double HOME = 8.75;


    public static final double minElevatorSafeHeight = 0;
  }
}
