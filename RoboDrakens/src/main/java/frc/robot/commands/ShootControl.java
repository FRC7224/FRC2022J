/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShootSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ShootControl extends CommandBase {

    private final ShootSubsystem m_shootsubsystem;
    // Used for Button Toggle Code
    private final Timer timer = new Timer();
    

    public ShootControl(ShootSubsystem subsystem) {

        m_shootsubsystem = subsystem;
        addRequirements(m_shootsubsystem);      

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
   //     m_shootsubsystem.setupShooter();
        Constants.shooterMode = false;
        timer.start();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        int zonePosition = 12;
        Joystick joystick1 = new Joystick(0);

        // if button 1 is pressed
        SmartDashboard.putBoolean("shoot mode ", Constants.shooterMode);

        final double timetorun = Constants.shooterTimer_timer;

        // cahnge zones
        if (joystick1.getRawAxis(4) > 0) {  
            if (zonePosition < 25) {
                zonePosition= zonePosition+ 1;
            }
        }

        if (joystick1.getRawAxis(4) < 0) {
            if (zonePosition > 1) {
                zonePosition = zonePosition - 1;
            }
        }

        SmartDashboard.putNumber("Zone", zonePosition);

        if (joystick1.getRawButton(Constants.kinitShooter)) {
            SmartDashboard.putNumber("shoot mode ", timetorun);



            if (timer.get() <= timetorun) {
                SmartDashboard.putNumber("shoot mode inside ", timetorun);
                SmartDashboard.putNumber("timer ", timer.get());
                m_shootsubsystem.setShootSpeed(zonePosition);

            } else {
                m_shootsubsystem.setShootSpeed(zonePosition);
                m_shootsubsystem.setelvSpeed(Constants.kelvspeed);
                 SmartDashboard.putNumber("shoot mode else ", timetorun);
            }
        } else

        {
            m_shootsubsystem.stopshooter();
            m_shootsubsystem.setelvSpeed(0);
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
