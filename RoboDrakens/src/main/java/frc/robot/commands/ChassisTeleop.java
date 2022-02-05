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

import frc.robot.subsystems.Chassis;

/**
 *
 */
public class ChassisTeleop extends CommandBase {

    private final Chassis m_chassis;

    public ChassisTeleop(Chassis subsystem) {

        m_chassis = subsystem;
        addRequirements(m_chassis);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_chassis.setupDrive();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Joystick joystick1 = new Joystick(0);
        double forward = -m_chassis.deadZone(joystick1.getY()); // forward
        double turn = -m_chassis.deadZone(joystick1.getX()); // turn
        if (forward >= 0) {
            forward = forward * forward;
        } else {
            forward = -(forward * forward);
        }
        if (turn >= 0) {
            turn = turn * turn;
        } else {
            turn = -(turn * turn);
        }
        SmartDashboard.putNumber("forward", forward);
        SmartDashboard.putNumber("turn", turn);
        SmartDashboard.putNumber("x", joystick1.getX());
        SmartDashboard.putNumber("y", joystick1.getY());
        m_chassis.autoshift();
        m_chassis.arcadeDrive(forward, turn);
        m_chassis.displayChasisData();
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
