// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    public static final class DrivetrainConstants {
      
      //Motor CAN Ids
      public static final int FLMotorID = 4;
      public static final int FRMotorID = 2;
      public static final int BLMotorID = 3;
      public static final int BRMotorID = 1;

      //encoder values
      public static final double EncoderPositionConversion = 0;
      public static final double EncoderSpeedConversion = 0;

      // the motors invert status
      public static final boolean FLInvert = true;
      public static final boolean FRInvert = true;
      public static final boolean BLInvert = true;
      public static final boolean BRInvert = true;

      // current limit for motors
      public static final int stallCurrentLimit = 60;
      public static final int freeCurrentLimit = 40;

      // the throttle multiplier for manual driving
      public static final double DriveSpeed = 0.5;
    }
  }
}
