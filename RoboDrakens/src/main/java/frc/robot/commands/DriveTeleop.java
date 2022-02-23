/*----------------------------------------------------------------------------*/
/* Copyright (c) 2022 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

/** A command that will turn the robot to the specified angle. */
public class DriveTeleop extends CommandBase

{

    private final DriveSubsystem m_drivesubsystem;

    private boolean firstpassturning = true;
    Timer setPointTimer = new Timer();
    public PIDController drivepid;

    /**
     * 
     *
     * 
     * 
     */
    public DriveTeleop(DriveSubsystem subsystem) {

        drivepid = new PIDController(Constants.kTurnP, Constants.kTurnI, Constants.kTurnD, 20);
        // Set the controller to be continuous (because it is an angle controller)
        drivepid.enableContinuousInput(-180, 180);
        // Set the controller tolerance - the delta tolerance ensures the robot is
        // stationary at the
        // setpoint before it is considered as having reached the reference
        drivepid.setTolerance(Constants.kTurnToleranceDeg);

        SendableRegistry.setName(drivepid, "DriveSubsystem" ,"Drive Stabilize PID");
         
        m_drivesubsystem = subsystem;
        addRequirements(m_drivesubsystem);

    
    }

    @Override
    public void execute() {
        Joystick joystick1 = new Joystick(0);
  
        double forward = -m_drivesubsystem.deadZone(joystick1.getY()); // forward
        double turn = -m_drivesubsystem.deadZone(joystick1.getX()); // turn
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
        SmartDashboard.putNumber("x", joystick1.getX());
        SmartDashboard.putNumber("y", joystick1.getY());
        m_drivesubsystem.autoshift();
        if (Constants.kenablePID == true) { // use PID process
            // **********************************************************
            // * PID Assit Driving Proccessing
            // *
            // **********************************************************
            //
            if (turn == 0.0) { // *** Not turning ****
                if (firstpassturning == true) { // Code is called first time, when we stopped turning
                    setPointTimer.start();
                    m_drivesubsystem.zeroHeading();
                    firstpassturning = false; // Set turning to false, because we are not
                                              // turning any more
                } else if (setPointTimer.get() != 0) {// settle timer reached
                    if (setPointTimer.get() >= 1.0) { // Robot is moving straight
                                                      // wait for timer before turning on PID
                        setPointTimer.stop();
                        setPointTimer.reset();
                    }
                } else { // after settling ** Driving straight using PID
                    // turn = Constants.gyroPIDOutput;
                    turn = drivepid.calculate(m_drivesubsystem.getHeading(), 0);
                    SmartDashboard.putNumber("Drive PID Output",turn);
                }
                // ELSE the user is still commanding
                // User is commanding a turn
            } else { // (turn != 0.0)
                // SmartDashboard.putNumber("turn in elseif",turn );
                setPointTimer.stop();
                setPointTimer.reset();
                firstpassturning = true;
                // Reset angle
            }
       //   m_drivesubsystem.displayDriveData();  
            m_drivesubsystem.arcadeDrive(forward, turn); // PID controlled Drive
        } // End of BasicDrive PID Control
          // ELSE PID is Off
        else { // use standard arcadeDrive
               // * PID off mode *****
        //    m_drivesubsystem.displayDriveData();
            m_drivesubsystem.arcadeDrive(forward, turn);
        }
        // End of PID enable loop\

    }

    @Override
    public boolean isFinished() {
        // End when the controller is at the reference.
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;

    }

}
