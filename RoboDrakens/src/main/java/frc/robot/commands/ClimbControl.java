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
        m_climbsubsystem.resetClimbPosition();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Joystick joystick1 = new Joystick(0);

        /* Gamepad processing */
        double throttlestick = joystick1.getZ();
     //   SmartDashboard.putNumber("joy1", joystick1.getZ());

        /**
         * Climb code
         * 
         * 
         * Initial hook up
         */
        if (joystick1.getRawButton(Constants.kclimbButtonInital)) {
            /* Position Closed Loop */

            /* 10 Rotations * 4296 u/rev in either direction * 12 gear ratio */

            targetPositionRotations = -throttlestick * 2048 * 12 * 10; // 2048 set for Falcon encoder

            // limit height
            if (targetPositionRotations >= Constants.kMaxClimbHeightInitial) {
                targetPositionRotations = Constants.kMaxClimbHeightInitial;
            }

            // limit retun
            if (targetPositionRotations <= Constants.kMinClimbHeight) {
                targetPositionRotations = Constants.kMinClimbHeight;

            }
        //    SmartDashboard.putNumber("climb initial", targetPositionRotations);
            m_climbsubsystem.pidClimb(targetPositionRotations);
        }

        // Climb

        if (joystick1.getRawButton(Constants.kclimbButton)) {
            /* Position Closed Loop */

            /* 10 Rotations * 4296 u/rev in either direction * 12 gear ratio */

            targetPositionRotations = -throttlestick * 2048 * 12 * 13; // 2048 set for Falcon encoder

            // limit height
            if (targetPositionRotations >= Constants.kMaxClimbHeightFinal) {
                targetPositionRotations = Constants.kMaxClimbHeightFinal;

            }

            // limit retun
            if (targetPositionRotations <= Constants.kMinClimbHeight) {
                targetPositionRotations = Constants.kMinClimbHeight;
            }
        //    SmartDashboard.putNumber("climb final ", targetPositionRotations);
            m_climbsubsystem.pidClimb(targetPositionRotations);
        }

        /* When button climbOverButto is held, just straight drive */
        if (joystick1.getRawButton(Constants.kclimboverridebutton)
                && joystick1.getRawButton(Constants.kclimbButtonInital)
                && joystick1.getRawButton(Constants.kclimbButton)) {
            /* Percent Output */
            if (throttlestick <0 ) {
                throttlestick = 0;  
            }
            m_climbsubsystem.straightClimb(-throttlestick * 0.4);
            // MUST Reboot after using this
        }

        /* When button climbOverButton is released turn off motor */
        if (joystick1.getRawButtonReleased(Constants.kclimboverridebutton)) {
            m_climbsubsystem.straightClimb(0);
            m_climbsubsystem.resetClimbPosition(); // Resets the new zero position
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
