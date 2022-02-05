/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

/**
 *
 */
public class Shifter extends SubsystemBase {

    private Solenoid solenoid1;

    /**
    *
    */
    public Shifter() {

     solenoid1 = new Solenoid(0, PneumaticsModuleType.CTREPCM, 0);
    addChild("Solenoid 1", solenoid1);
    }
 
    public void shiftHigh() {
    // opens shifter/put in high gear
    solenoid1.set(true);
    } 



    public void shiftLow() {
    // closes shifter/put in low gare
    solenoid1.set(false);
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
