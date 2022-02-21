/*----------------------------------------------------------------------------*/
/* Copyright (c) 2022 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

/**
 *
 */
public class MoveBalltoShooterTimed extends CommandBase {

    private final IntakeSubsystem m_intakesubsystem;
    private final Timer timer = new Timer();
    boolean ballLoaded = false;

    public MoveBalltoShooterTimed(IntakeSubsystem subsystem) {

        m_intakesubsystem = subsystem;
        addRequirements(m_intakesubsystem);

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
        double timetorun = Constants.kballIntakeTimer_timer;
        if (timer.get() <= timetorun) {
            if (m_intakesubsystem.getballLoadstatus()) { // is ball loaded ?
                m_intakesubsystem.setConveyorMotor(0); // stop conveyor ball is loaded
                m_intakesubsystem.closeballgate(); // clsoe the ball gate
                timetorun = 0;
                ballLoaded = true;
            } else {
                m_intakesubsystem.setConveyorMotor(Constants.kConveyorSpeed);
                m_intakesubsystem.openballgate(); // open the ball gate
                   }
        } else { // toggle off
            m_intakesubsystem.setConveyorMotor(0); // Robot conveyor set ot 0
            m_intakesubsystem.closeballgate(); // clsoe the ball gate
            ballLoaded = true;
            // Robot intake set ot 0
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intakesubsystem.setConveyorMotor(0); // Robot conveyor set ot 0
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return ballLoaded;
    }

    @Override
    public boolean runsWhenDisabled() {

        return false;

    }
}
