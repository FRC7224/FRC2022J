/*----------------------------------------------------------------------------*/
/* Copyright (c) 2022 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Chassis;
import frc.robot.commands.ChassisTeleop;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {

  private static RobotContainer m_robotContainer = new RobotContainer();

  // The robot's subsystems
  public final Shifter m_shifter = new Shifter();
  // public final Intake m_intake = new Intake();
  public final Chassis m_chassis = new Chassis();
  public final Pneumatics m_pneumatics = new Pneumatics();
  public final Shoot m_shoot = new Shoot();
  public final Climb m_climb = new Climb();

  // Joysticks
  private final Joystick joystick1 = new Joystick(0);

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {

    // Smartdashboard Subsystems

    // SmartDashboard Buttons
    SmartDashboard.putData("Autonomous Command", new AutonomousCommand());
    // SmartDashboard.putData("IntakeAction", new IntakeAction( m_intake ));
    SmartDashboard.putData("PneumaticsControl", new PneumaticsControl(m_pneumatics));
    SmartDashboard.putData("ShifterControl", new ShifterControl(m_shifter));
    SmartDashboard.putData("ShootControl", new ShootControl(m_shoot));
    SmartDashboard.putData("ClimbControl", new ClimbControl(m_climb));
    SmartDashboard.putData("Chassis", new ChassisTeleop(m_chassis));

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_chassis.setDefaultCommand(new ChassisTeleop(m_chassis));
    m_pneumatics.setDefaultCommand(new PneumaticsControl(m_pneumatics));

    // Configure autonomous sendable chooser

    m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());

    SmartDashboard.putData("Auto Mode", m_chooser);
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Create some buttons

  }

  public Joystick getJoystick1() {
    return joystick1;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return m_chooser.getSelected();
  }

}
