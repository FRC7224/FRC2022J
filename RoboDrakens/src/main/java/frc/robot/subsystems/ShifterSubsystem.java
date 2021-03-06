/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

/**
 *
 */
public class ShifterSubsystem extends SubsystemBase {

    private Solenoid shiftSolenoid;

    /**
    *
    */
    public ShifterSubsystem() {

     shiftSolenoid = new Solenoid(0, PneumaticsModuleType.CTREPCM, Constants.kPneumaticsShift );
    addChild("Shifter", shiftSolenoid);
    }
 
    public void shiftHigh() {
    // opens shifter/put in high gear
    shiftSolenoid.set(true);
    } 



    public void shiftLow() {
    // closes shifter/put in low gare
    shiftSolenoid.set(false);
    }

    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}
