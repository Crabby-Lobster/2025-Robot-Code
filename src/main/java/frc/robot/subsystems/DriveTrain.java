// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static frc.robot.Constants.OperatorConstants.DrivetrainConstants.*;
import static java.lang.Math.*;

public class DriveTrain extends SubsystemBase {

  // creates the motors
  SparkMax FLMotor = new SparkMax(FLMotorID, MotorType.kBrushless);
  SparkMax FRMotor = new SparkMax(FRMotorID, MotorType.kBrushless);
  SparkMax BLMotor = new SparkMax(BLMotorID, MotorType.kBrushless);
  SparkMax BRMotor = new SparkMax(BRMotorID, MotorType.kBrushless);

  // creates the encoders
  RelativeEncoder FLEncoder;
  RelativeEncoder FREncoder;
  RelativeEncoder BLEncoder;
  RelativeEncoder BREncoder;

  // creates the configurations for the motors
  SparkMaxConfig FLConfig = new SparkMaxConfig();
  SparkMaxConfig FRConfig = new SparkMaxConfig();
  SparkMaxConfig BLConfig = new SparkMaxConfig();
  SparkMaxConfig BRConfig = new SparkMaxConfig();


  // creates the diff drive
  DifferentialDrive tankDrive = new DifferentialDrive(FLMotor, FRMotor);

  // creates diff drive odometry
  DifferentialDriveOdometry driveOdometry;

  // creates gyro
  ADIS16470_IMU gyro = new ADIS16470_IMU();

  // creates SysIdRoutine
  SysIdRoutine systemId = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(this::voltageDrive, null, this)
  );


  /** Creates a new DriveTrain.*/
  public DriveTrain() {

    // disables drivetrain saftey
    tankDrive.setSafetyEnabled(false);

    //initalizes diff odometry
    driveOdometry = new DifferentialDriveOdometry(
      new Rotation2d(getHeading()),
      getEncoderValues(EncoderRetriaval.GetLeftDistance),
      getEncoderValues(EncoderRetriaval.GetRightDistance)
    );

    // applies the configs to the motors
    FLConfig.inverted(FLInvert).idleMode(IdleMode.kBrake).smartCurrentLimit(stallCurrentLimit, freeCurrentLimit);
    FLConfig.encoder.positionConversionFactor(EncoderPositionConversion).velocityConversionFactor(EncoderSpeedConversion);
    FLMotor.configure(FLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    FRConfig.inverted(FRInvert).idleMode(IdleMode.kBrake).smartCurrentLimit(stallCurrentLimit, freeCurrentLimit);
    FRConfig.encoder.positionConversionFactor(EncoderPositionConversion).velocityConversionFactor(EncoderSpeedConversion);
    FRMotor.configure(FRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // sets the back motors to follow the front motors
    BLConfig.follow(FLMotorID, BLInvert).idleMode(IdleMode.kBrake).smartCurrentLimit(stallCurrentLimit, freeCurrentLimit);
    BLConfig.encoder.positionConversionFactor(EncoderPositionConversion).velocityConversionFactor(EncoderSpeedConversion);
    BLMotor.configure(BLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    BRConfig.follow(FRMotorID, BRInvert).idleMode(IdleMode.kBrake).smartCurrentLimit(stallCurrentLimit, freeCurrentLimit);
    BRConfig.encoder.positionConversionFactor(EncoderPositionConversion).velocityConversionFactor(EncoderSpeedConversion);
    BRMotor.configure(BRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    

    // retrieves encoders from speed controllers
    FLEncoder = FLMotor.getEncoder();
    FREncoder = FRMotor.getEncoder();
    BLEncoder = BLMotor.getEncoder();
    BREncoder = BRMotor.getEncoder();

    resetEncoder(0);

    // calibrates gyro
    gyro.calibrate();
  }


  /**
   * @param leftSpeed The left motor throttle
   * @param rightSpeed The right motor throttle
   * @param square whether the inputs are squared when sent to the motor
   */
  public void TankDrive(double leftSpeed, double rightSpeed, boolean square){
    double[] Speeds = {leftSpeed, rightSpeed};

    // when square is true the values will be multiplied by themselves to make inputs smoother
    if (square) {
      Speeds[0] *= abs(Speeds[0]);
      Speeds[1] *= abs(Speeds[1]);
    }

    tankDrive.tankDrive(leftSpeed, rightSpeed);

    SmartDashboard.putBoolean("Drivetrain SquareThrottle", square);
    SmartDashboard.putNumberArray("Drivetrain Throttles", Speeds);
  }

  /** sets the voltage of the left and right drive motors
   * @param leftVoltage the voltage to send to the left motors
   * @param rightVoltage the voltage to send to the right motors
   */
  public void VTankDrive(double leftVoltage, double rightVoltage) {
    FLMotor.setVoltage(leftVoltage);
    FRMotor.setVoltage(rightVoltage);
  }

  /** voltage drive used by system identification
   * @param voltage the voltage for the motors
   */
  public void voltageDrive(Voltage voltage) {
    FLMotor.setVoltage(voltage);
    FRMotor.setVoltage(voltage);
  }

  enum EncoderRetriaval {
    GetSpeed,
    GetDistance,
    GetLeftSpeed,
    GetRightSpeed,
    GetLeftDistance,
    GetRightDistance
  }

  /**
   * @param returnType the type of data to be returned when the function is called
   * @return the desired encoder reading
   */
  public double getEncoderValues(EncoderRetriaval returnType) {
    switch (returnType) {
      case GetSpeed:
        double avgSpeed = FLEncoder.getVelocity() + FREncoder.getVelocity() + BLEncoder.getVelocity() + BREncoder.getVelocity();
        avgSpeed /= 4.0;
        return avgSpeed;
      case GetDistance:
        double avgDist = FLEncoder.getPosition() + FREncoder.getPosition() + BLEncoder.getPosition() + BREncoder.getPosition();
        avgDist /= 4.0;
        return avgDist;
      case GetLeftDistance:
        double avgLeftDist = FLEncoder.getPosition() + BLEncoder.getPosition();
        avgLeftDist /= 2.0;
        return avgLeftDist;
      case GetLeftSpeed:
        double avgLeftSpeed = FLEncoder.getVelocity() + BLEncoder.getVelocity();
        avgLeftSpeed /= 2.0;
        return avgLeftSpeed;
      case GetRightDistance:
        double avgRightdist = FREncoder.getPosition() + BREncoder.getPosition();
        avgRightdist /= 2.0;
        return avgRightdist;
      case GetRightSpeed:
        double avgRightSpeed = FREncoder.getVelocity() + BREncoder.getVelocity();
        avgRightSpeed /= 2.0;
        return avgRightSpeed;
      default:
        return 0;
    }
  }

  /** function for returning wheel speeds for ramsete controller
   */
  public DifferentialDriveWheelSpeeds getDiffWheelSpeed() {
    return new DifferentialDriveWheelSpeeds(
      getEncoderValues(EncoderRetriaval.GetLeftSpeed),
      getEncoderValues(EncoderRetriaval.GetRightSpeed)
    );
  }

  /**
   * @param position the position to set the encoders to
   */
  public void resetEncoder(double position) {
    FLEncoder.setPosition(position);
    FREncoder.setPosition(position);
    BLEncoder.setPosition(position);
    BREncoder.setPosition(position);
  }

  public double getHeading() {
    return gyro.getAngle(gyro.getYawAxis());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // posts drivetrain encoder data to driverstation
    SmartDashboard.putNumber("DriveTrain Speed", getEncoderValues(EncoderRetriaval.GetSpeed));
    SmartDashboard.putNumber("DriveTrain Distance", getEncoderValues(EncoderRetriaval.GetDistance));

    SmartDashboard.putNumber("DriveTrain LeftSpeed", getEncoderValues(EncoderRetriaval.GetLeftSpeed));
    SmartDashboard.putNumber("DriveTrain LeftDistance", getEncoderValues(EncoderRetriaval.GetLeftDistance));

    SmartDashboard.putNumber("DriveTrain RightSpeed", getEncoderValues(EncoderRetriaval.GetRightSpeed));
    SmartDashboard.putNumber("DriveTrain RightDistance", getEncoderValues(EncoderRetriaval.GetRightDistance));

    SmartDashboard.putNumber("DriveTrain Heading", getHeading());
  }

  /**gets the static function for system id
   */
  public Command systemIdStatic(SysIdRoutine.Direction direction) {
    return systemId.quasistatic(direction);
  }

  /**gets the dynamic function for system id
   */
  public Command systemIdDynamic(SysIdRoutine.Direction direction) {
    return systemId.dynamic(direction);
  }
}
