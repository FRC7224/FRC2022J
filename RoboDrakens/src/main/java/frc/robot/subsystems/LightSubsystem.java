/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

/**
 *
 */
public class LightSubsystem extends SubsystemBase {


    private WPI_VictorSPX light = new WPI_VictorSPX(Constants.kLightPort);

    /**
    *
    */
    public LightSubsystem() {

   addChild("Light", light);
    }
 
    public void lightOn() {
    // turns light on 
    light.set(1.0);
    } 



    public void lightOff() {
    // turns light off
    light.set(0.0);
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
