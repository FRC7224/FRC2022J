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
import frc.robot.subsystems.IntakeSubsystem;

/**
 *
 */
public class IntakeAction extends CommandBase {

    private final IntakeSubsystem m_intakesubsystem;

    public IntakeAction(IntakeSubsystem subsystem) {

        m_intakesubsystem = subsystem;
        addRequirements(m_intakesubsystem);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Set Lauch ready satus
        // sets state of LAUNCHREADY
        boolean ballloaded;
        ballloaded = m_intakesubsystem.getballLoadstatus();

    //    SmartDashboard.putNumber("intakeeee running", Constants.kIntakeSpeed);
        Joystick joystick1 = new Joystick(0);
        if (joystick1.getRawButton(Constants.kintakebutton)) { // Robot intake
            m_intakesubsystem.lowerIntake();
            m_intakesubsystem.setIntakeMotor(Constants.kIntakeSpeed); // Robot conveyor
            if (ballloaded == false) { // is ball not loaded 
                m_intakesubsystem.setConveyorMotor(Constants.kConveyorSpeed);
                m_intakesubsystem.openballgate();
            } else {
                m_intakesubsystem.setConveyorMotor(0); // stop conveyor ball is loaded
                m_intakesubsystem.closeballgate(); // close the ball gate
            }
        } else  { 
            if (joystick1.getRawButton(Constants.kconveyerbutton)) { // Robot intake
                m_intakesubsystem.setConveyorMotor(Constants.kConveyorSpeed);
                m_intakesubsystem.openballgate(); // open the ball gate
            } else {  // turns off intake and convayor
                m_intakesubsystem.setIntakeMotor(0); // stop conveyor ball is loaded
                m_intakesubsystem.setConveyorMotor(0); // stop conveyor ball is loaded
                m_intakesubsystem.closeballgate(); // clsoe the ball gate
            }
        }

        // Rasie Intake
        if (joystick1.getRawButton(Constants.kintakeupbutton)) { // Robot intake
            m_intakesubsystem.raiseIntake();
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
