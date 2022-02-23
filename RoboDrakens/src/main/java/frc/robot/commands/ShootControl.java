/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ShootControl extends CommandBase {

    private final ShootSubsystem m_shootsubsystem;
    private final IntakeSubsystem m_intakesubsystem;
    int zonePosition = 12;
    int buttonDelay ;
    boolean launchInProgress = false;

    // Used for Button Toggle Code
    private final Timer timer = new Timer();

    public ShootControl(ShootSubsystem subsystem, IntakeSubsystem subsystem2) {

        m_shootsubsystem = subsystem;
        addRequirements(m_shootsubsystem);

        m_intakesubsystem = subsystem2;
        addRequirements(m_intakesubsystem);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // m_shootsubsystem.setupShooter();
        Constants.shooterMode = false;
        timer.reset();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        Joystick joystick1 = new Joystick(0);
        boolean ballshot = false;
        boolean launchReady = false;

       

        // Set Lauch ready status
        if (m_intakesubsystem.getballLoadstatus()) {
            launchReady = true;
            SmartDashboard.putBoolean("launchReady", launchReady);
        } ;
    

        // If throttle swicth is pressed
        // Change zones
        if (buttonDelay == 5) { // Runs every x loops
            if ((joystick1.getRawAxis(4) > 0) & (zonePosition < 25)) {
                zonePosition = zonePosition + 1;
            }
            if ((joystick1.getRawAxis(4) < 0) & (zonePosition > 0)) {
                zonePosition = zonePosition - 1;
            }
            buttonDelay = 0;
        } else {
            buttonDelay = buttonDelay + 1;
        }
        SmartDashboard.putNumber("Zone", zonePosition);

        /// shoot botton is pressed
        if (joystick1.getRawButton(Constants.kinitShooter) & launchReady & !launchInProgress ) {
            launchInProgress = true; 
            timer.start();
            timer.reset();
        }   

        SmartDashboard.putNumber("Shoot Timer", timer.get());
        SmartDashboard.putBoolean("launchInProgress", launchInProgress);
        if (launchInProgress) {
            if (timer.get() <= Constants.kshooterTimer_spin) {
                System.out.print("spinup");
                m_shootsubsystem.setShootSpeed(zonePosition);

            } else if (timer.get() <= Constants.kshooterTimer_timer) {
                System.out.print("shoot ing");
                m_shootsubsystem.pushBall();
                m_shootsubsystem.setShootSpeed(zonePosition);
                m_shootsubsystem.setelvSpeed(Constants.kelvspeed);
                ballshot = true;
            } else {
                launchInProgress = false; 
                timer.reset();  
                timer.stop();          
            }
        } else {
            m_shootsubsystem.stopshooter();
            m_shootsubsystem.setelvSpeed(0);
            m_shootsubsystem.resetBallPush();
            if (ballshot) { // first time after a ball has been shot
                new SequentialCommandGroup(new MoveBalltoShooterTimed(m_intakesubsystem));
                ballshot = false;
                launchReady = false;
            }

        }

       

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {

        return false;

    }
}
