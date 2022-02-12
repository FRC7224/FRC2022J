/*----------------------------------------------------------------------------*/
/* Copyright (c) 2022 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/



package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;

/**
 *
 */
public class ClimbControl extends CommandBase {

    private final ClimbSubsystem m_climbsubsystem;

    public ClimbControl(ClimbSubsystem subsystem) {

        m_climbsubsystem = subsystem;
        addRequirements(m_climbsubsystem);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Joystick joystick1 = new Joystick(0);
        if (joystick1.getRawButton(Constants.kclimbButton)) { // Robot climb
            m_climbsubsystem.climbMotorControl(Constants.kClimbSpeed);
        } else { // toggle off
            m_climbsubsystem.climbMotorControl(0);
            // Robot cliomb set ot 0
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
