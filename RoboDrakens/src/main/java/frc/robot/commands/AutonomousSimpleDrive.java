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
import frc.robot.subsystems.DriveSubsystem;

/**
 *
 */
public class AutonomousSimpleDrive extends CommandBase {

    private final DriveSubsystem m_drivesubsystem;
    private final Timer timer = new Timer();
    boolean drivecomplete = false;

    public AutonomousSimpleDrive(DriveSubsystem subsystem) {

        m_drivesubsystem = subsystem;
        addRequirements(m_drivesubsystem);

        ;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        timer.start();
        drivecomplete = false;
        double timetorun = 3.0;
         if (timer.get() <= timetorun) {
             SmartDashboard.putNumber("simple run time" , timer.get());
             m_drivesubsystem.arcadeDrive(0.4, 0);

         }else {
             drivecomplete = true;
         }
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivesubsystem.arcadeDrive(0, 0); // Robot  set ot 0
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return drivecomplete ;
    }

    @Override
    public boolean runsWhenDisabled() {

        return false;

    }
}
