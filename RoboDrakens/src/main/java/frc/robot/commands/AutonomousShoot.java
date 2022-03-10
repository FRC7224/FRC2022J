/*----------------------------------------------------------------------------*/
/* Copyright (c) 2022 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShootSubsystem;

/**
 *
 */
public class AutonomousShoot extends CommandBase {

    private final ShootSubsystem m_shootsubsystem;
    private final Timer timer = new Timer();
    int zonePosition = 7;
    int buttonDelay ;
    boolean launchInProgress = false;
    boolean ballshot ;

    public AutonomousShoot(ShootSubsystem subsystem, int zonePosition) {

        m_shootsubsystem = subsystem;
        addRequirements(m_shootsubsystem);

        ;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        launchInProgress = false;
        ballshot = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if ( !launchInProgress ) {
          
            launchInProgress = true; 
     //       SmartDashboard.putBoolean("setting launch readay", launchInProgress);
            timer.start();
            timer.reset();
        }   

   //     SmartDashboard.putNumber("Shoot Timer", timer.get());
   //     SmartDashboard.putBoolean("launchInProgress", launchInProgress);
        if (launchInProgress) {
            if (timer.get() <= Constants.kshooterTimer_spin) {
                System.out.print("spinup");
                m_shootsubsystem.setShootSpeed(zonePosition);

            } else if (timer.get() <= Constants.kshooterTimer_timer) {
                System.out.print("shoot ing");
                m_shootsubsystem.setShootSpeed(zonePosition);
                m_shootsubsystem.setelvSpeed(Constants.kelvspeed);
                if (timer.get() <= (Constants.kshooterTimer_timer+ 0.5)){  // 0.5 second before pushing
                    m_shootsubsystem.pushBall();
                }
            } else {
                launchInProgress = false; 
                timer.reset();  
                timer.stop();       
                m_shootsubsystem.stopshooter();
                m_shootsubsystem.setelvSpeed(0);
                m_shootsubsystem.resetBallPush();   
                ballshot = true;
            }
        } else {
            m_shootsubsystem.stopshooter();
            m_shootsubsystem.setelvSpeed(0);
            m_shootsubsystem.resetBallPush();

            }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shootsubsystem.setShootSpeed(0); // Robot conveyor set ot 0
        m_shootsubsystem.setelvSpeed(0);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return ballshot;
    }

    @Override
    public boolean runsWhenDisabled() {

        return false;

    }
}
