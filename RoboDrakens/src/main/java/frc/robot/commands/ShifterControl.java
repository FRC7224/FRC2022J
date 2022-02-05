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
import frc.robot.subsystems.Shifter;

/**
 *
 */
public class ShifterControl extends CommandBase {

    private final Shifter m_shifter;

    public ShifterControl(Shifter subsystem) {

        m_shifter = subsystem;
        addRequirements(m_shifter);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        Joystick joystick1 = new Joystick(0);  // swicth to high gear
        if (joystick1.getRawButton(Constants.kshiftbutton)) {
            m_shifter.shiftHigh();
            Constants.shiftOpenState = true;
        } else { // toggle off
            m_shifter.shiftLow();
            Constants.shiftOpenState = false;
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
