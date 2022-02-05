/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shoot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ShootControl extends CommandBase {

    private final Shoot m_shoot;
    // Used for Button Toggle Code
    private final Timer timer = new Timer();

    public ShootControl(Shoot subsystem) {

        m_shoot = subsystem;
        addRequirements(m_shoot);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_shoot.setupShooter();
        Constants.shooterMode = false;
        timer.start();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double motorspeedTop;
        double motorspeedBottom;
        Joystick joystick1 = new Joystick(0);

        // if button 1 is pressed
        SmartDashboard.putBoolean("shoot mode ", Constants.shooterMode);

        final double timetorun = Constants.shooterTimer_timer;

        // cahnge zones
        if (joystick1.getRawButtonPressed(Constants.zoneup)) {
            if (Constants.zone < 4) {
                Constants.zone = Constants.zone + 1;
            }
        }

        if (joystick1.getRawButtonPressed(Constants.zonedown)) {
            if (Constants.zone > 1) {
                Constants.zone = Constants.zone - 1;
            }
        }

        SmartDashboard.putNumber("Zone", Constants.zone);

        if (joystick1.getRawButton(Constants.kinitShooter)) {
            SmartDashboard.putNumber("shoot mode ", timetorun);

            switch (Constants.zone) {
                case 1: // Zone 1 Green
                    motorspeedTop = Constants.zone1shootertargetspeedTop;
                    motorspeedBottom = Constants.zone1shootertargetspeedBottom;
                    break;
                case 2: // Zone 2 Yellow
                    motorspeedTop = Constants.zone2shootertargetspeedTop;
                    motorspeedBottom = Constants.zone2shootertargetspeedBottom;
                    break;
                case 3: // Zone 3 Blue
                    motorspeedTop = Constants.zone3shootertargetspeedTop;
                    motorspeedBottom = Constants.zone3shootertargetspeedBottom;
                    break;
                case 4: // Zone 4 red
                    motorspeedTop = Constants.zone4shootertargetspeedTop;
                    motorspeedBottom = Constants.zone4shootertargetspeedBottom;
                    break;
                default: // Zone 4
                    motorspeedTop = Constants.zone4shootertargetspeedTop;
                    motorspeedBottom = Constants.zone4shootertargetspeedBottom;
                    break;
            }

            if (timer.get() <= timetorun) {
                SmartDashboard.putNumber("shoot mode inside ", timetorun);
                SmartDashboard.putNumber("timer ", timer.get());
                m_shoot.setShootSpeed(motorspeedTop, motorspeedBottom);

            } else {
                m_shoot.setShootSpeed(motorspeedTop, motorspeedBottom);
                m_shoot.setelvSpeed(Constants.kelvspeed);
                 SmartDashboard.putNumber("shoot mode else ", timetorun);
            }
        } else

        {
            m_shoot.setShootSpeed(0, 0);
            m_shoot.setelvSpeed(0);
            timer.reset();
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
