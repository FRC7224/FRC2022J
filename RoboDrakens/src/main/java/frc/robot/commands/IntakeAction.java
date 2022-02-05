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
import frc.robot.subsystems.Intake;

/**
 *
 */
public class IntakeAction extends CommandBase {


        private final Intake m_intake;
 

    public IntakeAction(Intake subsystem) {

       m_intake = subsystem;
        addRequirements(m_intake);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Joystick joystick1 = new Joystick(0);
        if (joystick1.getRawButton(Constants.kintakebutton)) { // Robot intake
            m_intake.setIntakeMotor(Constants.kIntakeSpeed);   // Robot conveyor
            m_intake.setConveyorMotor(Constants.kConveyorSpeed);
        } else { // toggle off
            m_intake.setIntakeMotor(0);    // Robot intake set ot 0
            m_intake.setConveyorMotor(0);  // Robot conveyor set ot 0
            // Robot intake set ot 0
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
