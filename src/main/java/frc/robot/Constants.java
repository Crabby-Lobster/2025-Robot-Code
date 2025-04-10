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
    public static final double DriveSpeed = 1;

    // diff drive constants
    public static final double kTrackwidthMeter = 0.60325;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeter);
  }
  

  /**
   *  saved values for algea arm
   */
  public static final class AlgaeArmConstants {
    // Motors
    public static final int pivotID = 9;
    public static final int rollerLID = 20;
    public static final int rollerRID = 21;
    public static final double pivotPosConversion = (1 / 275.0) * 360;
    public static final double rollerSpeedConversion = 1;

    public static final boolean pivotInvert = true;
    public static final boolean rollerInvert = true;

    // Limit switches
    public static final int HomeSwitchID = 8;
    public static final int AlgeaSwitchID = 5;

    // PID
    public static final double[] PIDValues = {0.1,0,0};

    // Rollers
    public static final double IntakeSpeed = -0.25;
    public static final double ScoreSpeed = 1;
    public static final double HoldSpeed = -0.1;
  }

  /**
   * saved algea arm position
   */
  public static final class AlgearArmPositions {
    public static final double HOME = 75;
    public static final double MINANGLE = -35;

    public static final double[] dangerAngles = {-20, MINANGLE};
    public static final double[] dangerHeight = {0,0};

    public static final double Barge = 70;
    public static final double ReefHigh = 0;
    public static final double ReefLow = -12;
    public static final double STORE = 65;
    public static final double GroundHigh = -10;
    public static final double Ground = -12;
    
  }

  /**
   * values for the elevator subsytem
   */
  public static final class ElevatorConstants {
    public static final int leftMoterID = 4;
    public static final int rightMoterId = 5;
    public static final int stage2MotorId = 14;

    public static final boolean leftInvert = false;
    public static final boolean rightInvert = true;
    public static final boolean stage2invert = true;

    public static final int stallCurrentLimit = 40;

    public static final double stage1positionConversion = (1.0 / 12.0) * 1.75 * PI;
    public static final double stage2positionConversion = (1.0 / 20) * 1.5 * PI;

    public static final double[] stage1elevatorPID = {0.1,0,0};
    public static final double[] stage2elevatorPID = {0.1,0,0};

    public static final int elevatorLimitswitch = 9;
  }

  /**
   * stored elevator positions
   */
  public static final class ElevatorPositions {

    public static final double MAXHEIGHT() {
      return STAGE1TOP + STAGE2TOP;
    }

    public static final double STAGE1TOP = 25;
    public static final double STAGE2TOP = 16;
    public static final double HOME = 0;


    public static final double Barge = MAXHEIGHT();
    public static final double ReefHigh = 38;
    public static final double ReefLow = 26;
    public static final double STORE = 0;
    public static final double GroundHigh = 10;
    public static final double Ground = 0;
    
  }
}
