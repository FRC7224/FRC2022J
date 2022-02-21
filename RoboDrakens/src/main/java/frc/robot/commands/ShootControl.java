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
        timer.start();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        int zonePosition = 12;
        Joystick joystick1 = new Joystick(0);
        boolean ballshot = false;
        final double timetorun = Constants.shooterTimer_timer;

        // If throttle swicth is pressed
        // Change zones
        if (joystick1.getRawAxis(4) > 0) {
            if (zonePosition < 25) {
                zonePosition = zonePosition + 1;
            }
        }

        if (joystick1.getRawAxis(4) < 0) {
            if (zonePosition > 1) {
                zonePosition = zonePosition - 1;
            }
        }
        SmartDashboard.putNumber("Zone", zonePosition);

        /// Long shoot botton is pressed
        if (joystick1.getRawButtonPressed(Constants.kinitShooter)) {
            SmartDashboard.putNumber("shoot mode ", timetorun);
            if (timer.get() <= timetorun) {
                SmartDashboard.putNumber("shoot mode inside ", timetorun);
                SmartDashboard.putNumber("timer ", timer.get());
                m_shootsubsystem.setShootSpeed(zonePosition);

            } else {
                m_shootsubsystem.pushBall();
                m_shootsubsystem.setShootSpeed(zonePosition);
                m_shootsubsystem.setelvSpeed(Constants.kelvspeed);
                SmartDashboard.putNumber("shoot mode else ", timetorun);
                ballshot = true;
            }
        } else {
            m_shootsubsystem.stopshooter();
            m_shootsubsystem.setelvSpeed(0);
            m_shootsubsystem.resetBallPush();
            timer.reset();
            if (ballshot) { // first time after a ball has been shot
                new SequentialCommandGroup(new MoveBalltoShooterTimed(m_intakesubsystem));
                ballshot = false;
            }
        }

        /// short low shoot botton is pressed
        if (joystick1.getRawButtonPressed(Constants.kshortshootbutton)) {
            SmartDashboard.putNumber("shoot mode ", timetorun);
            if (timer.get() <= timetorun) {
                SmartDashboard.putNumber("shoot mode inside ", timetorun);
                SmartDashboard.putNumber("timer ", timer.get());
                m_shootsubsystem.setShootSpeed(zonePosition);

            } else {
                m_shootsubsystem.pushBall();
                m_shootsubsystem.setShootSpeed(Constants.kshortshootzone);
                m_shootsubsystem.setelvSpeed(Constants.kelvspeed);
                SmartDashboard.putNumber("shoot mode else ", timetorun);
                ballshot = true;
            }
        } else {
            m_shootsubsystem.stopshooter();
            m_shootsubsystem.setelvSpeed(0);
            m_shootsubsystem.resetBallPush();
            timer.reset();
            if (ballshot) { // first time after a ball has been shot
                new SequentialCommandGroup(new MoveBalltoShooterTimed(m_intakesubsystem));
                ballshot = false;
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
