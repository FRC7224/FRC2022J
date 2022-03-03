/*----------------------------------------------------------------------------*/
/* Copyright (c) 2022 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;

/**
 *
 */
public class ClimbControl extends CommandBase {

    private final ClimbSubsystem m_climbsubsystem;

    /** Save the target position to servo to */
    double targetPositionRotations = 0;
    boolean climblock = false;

    public ClimbControl(ClimbSubsystem subsystem) {

        m_climbsubsystem = subsystem;
        addRequirements(m_climbsubsystem);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        targetPositionRotations = 0;
        climblock = false;
        m_climbsubsystem.setclimbrelease(1.0); // full power on iniitalize then use constant
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Joystick joystick1 = new Joystick(0);

        /* Gamepad processing */
        double throttlestick = joystick1.getZ();
        SmartDashboard.putNumber("joy1", joystick1.getZ());

        /* climb lock */
        if (joystick1.getRawButtonPressed(Constants.kclimbLock)) {
            climblock = true;
            m_climbsubsystem.setclimblock();
        }

        /* climb lock release and refreash */
        if (joystick1.getRawButtonPressed(Constants.kclimbrelease)) {
            climblock = false;
            m_climbsubsystem.setclimbrelease(1.0); // full power when button is initially pressed
                                                   // Note: using getRawButtonPressed
        } else {
            if (climblock == false) { /* climb lock refreash */
                m_climbsubsystem.setclimbrelease(Constants.kclimbreleaseP); // set refereash power level
            }
        }
        
        
        /**
         * Climb code
         */
        if (joystick1.getRawButton(Constants.kclimbButton)) {
            /* Position Closed Loop */

            // release lock 
            m_climbsubsystem.setclimbrelease(1.0); // Full lock power during climb 

            /* 10 Rotations * 4296 u/rev in either direction * 12 gear ratio */
            SmartDashboard.putNumber("climb", targetPositionRotations);
            targetPositionRotations = -throttlestick * 2048 * 12 * 10; // 2048 set for Falcon encoder

            // limit height
            if (targetPositionRotations >= Constants.kMaxClimbHeight) {
                targetPositionRotations = Constants.kMaxClimbHeight;
            }

            // limit retun
            if (targetPositionRotations <= Constants.kMinClimbHeight) {
                targetPositionRotations = Constants.kMinClimbHeight;
            }

            m_climbsubsystem.pidClimb(targetPositionRotations);
        }

        /* When button climbOverButto is held, just straight drive */
        if (joystick1.getRawButton(Constants.kclimboverridebutton)) {

            // release lock 
            m_climbsubsystem.setclimbrelease(1.0);  //Full lock power during climb 

            /* Percent Output */
            m_climbsubsystem.straightClimb(throttlestick * 0.5);
        }

        /* When button climbOverButton is released turn off motor */
        if (joystick1.getRawButtonReleased(Constants.kclimboverridebutton)) {
            /* Percent Output */
            m_climbsubsystem.straightClimb(0);
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
