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

    public ClimbControl(ClimbSubsystem subsystem) {

        m_climbsubsystem = subsystem;
        addRequirements(m_climbsubsystem);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        targetPositionRotations = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Joystick joystick1 = new Joystick(0);

        /* Gamepad processing */
        double throttlestick = joystick1.getZ();
        SmartDashboard.putNumber("joy1", joystick1.getZ());
        boolean climbButton = joystick1.getRawButton(Constants.kclimbButton);
        boolean climbOverButton = joystick1.getRawButton(Constants.kclimboverridebutton);

        /* Deadband gamepad */
      //  if (Math.abs(throttlestick) < 0.10) {
      //      /* Within 10% of zero */
      //      throttlestick = 0;
    //    }

        /**
         * When button climbButton is pressed, perform Position Closed Loop to selected
         * position,
         * indicated by Joystick position x10, [-10, 10] rotations
         */
        if (climbButton) {
            /* Position Closed Loop */

            /* 10 Rotations * 4296 u/rev in either direction   * 12 gear ratio */
            SmartDashboard.putNumber("climb", targetPositionRotations);
            targetPositionRotations = -throttlestick * 10.0 * 2048 * 12 ; // 2048 set for Falcon encoder

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
        if (climbOverButton) {
            /* Percent Output */
            m_climbsubsystem.straightClimb(throttlestick);
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
