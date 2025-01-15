// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    public static final class DrivetrainConstants {
      
      //Motor CAN Ids
      public static final int FLMotorID = 4;
      public static final int FRMotorID = 2;
      public static final int BLMotorID = 3;
      public static final int BRMotorID = 1;

      //encoder values
      public static final double EncoderPositionConversion = 1;
      public static final double EncoderSpeedConversion = 1;

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

      // constants for trajectory following
      public static final double ksVolts = 0;
      public static final double ksVoltSecondsPerMeter = 0;
      public static final double kaVoltSecondsSquaredPerMeter = 0;
      public static final double kPDriveVel = 0;

      // diff drive constants
      public static final double kTrackwidthMeter = 0;
      public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeter);

      // trajectory constraints
      public static final double kMaxSpeedMeterPerSeconds = 1;
      public static final double kMaxAccelerationMeterPerSecondSquared = 1;

      // RAMSETE constants
      public static final double kRamseteB = 2;
      public static final double kRamseteZeta = 0.7;
    }
  }
}
