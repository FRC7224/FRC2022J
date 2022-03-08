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
import frc.robot.subsystems.DriveSubsystem;
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
  public final ShifterSubsystem m_shiftersubsystem = new ShifterSubsystem();
  public final IntakeSubsystem m_intakesubsystem = new IntakeSubsystem();
  public final DriveSubsystem m_drivesubsystem = new DriveSubsystem();
  public final PneumaticsSubsystem m_pneumaticssubsystem = new PneumaticsSubsystem();
  public final ShootSubsystem m_shootsubsystem = new ShootSubsystem();
  public final ClimbSubsystem m_climbsubsystem = new ClimbSubsystem();
  public final LightSubsystem m_lightsubsystem = new LightSubsystem();

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
    // SmartDashboard.putData("PneumaticsControl", new PneumaticsControl(m_pneumatics));
    // SmartDashboard.putData("ShifterControl", new ShifterControl(m_shifter));
    // SmartDashboard.putData("ShootControl", new ShootControl(m_shoot));
    // SmartDashboard.putData("ClimbControl", new ClimbControl(m_climb));
    // SmartDashboard.putData("DriveTeleop", new DriveTeleop(m_drivesubsystem));

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_drivesubsystem.setDefaultCommand(new DriveTeleop(m_drivesubsystem));
    m_climbsubsystem.setDefaultCommand(new ClimbControl(m_climbsubsystem));
    m_intakesubsystem.setDefaultCommand(new IntakeAction(m_intakesubsystem));
    m_pneumaticssubsystem.setDefaultCommand(new PneumaticsControl(m_pneumaticssubsystem));
    m_shiftersubsystem.setDefaultCommand(new ShifterControl(m_shiftersubsystem));
     m_shootsubsystem.setDefaultCommand(new ShootControl(m_shootsubsystem));
     m_lightsubsystem.setDefaultCommand(new LightOn( m_lightsubsystem));


    // Configure autonomous sendable chooser

    m_chooser.setDefaultOption("Do Nothing", new AutonomousCommand());
    m_chooser.addOption("File Generator", new AutonomousGrpFileGenerator(m_drivesubsystem));
    m_chooser.addOption("Left Start Position", new AutonomousGrpLeft(m_drivesubsystem,m_shootsubsystem,m_intakesubsystem));
    m_chooser.addOption("Center Start Position", new AutonomousGrpCenter(m_drivesubsystem,m_shootsubsystem,m_intakesubsystem));
    m_chooser.addOption("Right Start Position", new AutonomousGrpRight(m_drivesubsystem,m_shootsubsystem,m_intakesubsystem));
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


    ////  ****************************************  Move ????????????????????????????????
    new InstantCommand(m_lightsubsystem::lightOn, m_lightsubsystem);

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
