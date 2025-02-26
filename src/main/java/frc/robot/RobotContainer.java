// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DefaultElevator;
import frc.robot.commands.ElevatorHome;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  AutoFactory autofactory;

  // controllers
  private final Joystick leftStick = new Joystick(ControllerConstants.LeftJoystick);
  private final Joystick rightStick = new Joystick(ControllerConstants.rightJoystick);
  private final XboxController controller = new XboxController(ControllerConstants.controller);

  // subsystems
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final Elevator m_elevator = new Elevator();

  // Default commands
  private final DefaultDrive m_DefaultDrive = new DefaultDrive(leftStick, rightStick, m_driveTrain, controller);
  private final DefaultElevator m_DefaultElevator = new DefaultElevator(m_elevator, leftStick, rightStick, controller);

  // Homing Commands
  private final ElevatorHome m_ElevatorHome = new ElevatorHome(m_elevator);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(ControllerConstants.controller);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autofactory = new AutoFactory(
      m_driveTrain::getPose,
      m_driveTrain::resetOdometry,
      m_driveTrain::followTrajectory,
      true,
      m_driveTrain
    );

    // Configure the trigger bindings
    configureBindings();

    // sets default commands
    m_driveTrain.setDefaultCommand(m_DefaultDrive);
    m_elevator.setDefaultCommand(m_DefaultElevator);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.testTrajectory(autofactory);
  }

  /**
   * Homes the robot subsystems
   */
  public void HomeRobot() {
    //m_ElevatorHome.schedule();
  }
}
