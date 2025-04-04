// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Autos.AutoContainer;
import frc.robot.Autos.AutoReset;
import frc.robot.Autos.AutoUpdateScoreSystem;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.DefaultClimber;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DefaultScoreSystem;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ScoreSystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // controllers
  private final Joystick leftStick = new Joystick(ControllerConstants.LeftJoystick);
  private final Joystick rightStick = new Joystick(ControllerConstants.rightJoystick);
  private final Joystick controller = new Joystick(ControllerConstants.controller);

  
  // subsystems
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final AlgaeArm m_algaeArm = new AlgaeArm();
  private final Elevator m_elevator = new Elevator();
  private final Climber m_climber = new Climber();
  private final ScoreSystem m_ScoreSystem = new ScoreSystem(m_elevator, m_algaeArm);

  private final PositionContainer m_PositionContainer = new PositionContainer(leftStick, rightStick, controller, m_ScoreSystem);
  private final AutoContainer m_AutoContainer = new AutoContainer(m_driveTrain, m_ScoreSystem, m_PositionContainer);

  // Default commands
  private final DefaultDrive m_DefaultDrive = new DefaultDrive(leftStick, rightStick, m_driveTrain, controller);
  private final DefaultScoreSystem m_DefaultScoreSystem = new DefaultScoreSystem(m_ScoreSystem, leftStick, rightStick, controller, m_PositionContainer);
  private final DefaultClimber m_DefaultClimber = new DefaultClimber(m_climber, rightStick);
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandJoystick m_driverController =
      new CommandJoystick(ControllerConstants.controller);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();

    // sets default commands
    m_driveTrain.setDefaultCommand(m_DefaultDrive);
    m_ScoreSystem.setDefaultCommand(m_DefaultScoreSystem);
    m_climber.setDefaultCommand(m_DefaultClimber);
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
    //m_driverController.button(1).onTrue(HomeRobot());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new SequentialCommandGroup(
      HomeRobot(),
      new AutoReset(m_PositionContainer),

      // updates the scoresystem while the auto is running
      //this keeps the scoresystem at the set position
      Commands.deadline(
        m_AutoContainer.AlgaeScoreAuto(),
        new AutoUpdateScoreSystem(m_PositionContainer, m_ScoreSystem)
      )
    );
  }

  /**
   * Homes the robot subsystems
   */
  public SequentialCommandGroup HomeRobot() {
    return m_ScoreSystem.HomeSystems(m_ScoreSystem);
  }
}
