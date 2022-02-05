/*----------------------------------------------------------------------------*/
/* Copyright (c) 2022 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 *
 */
public class Pneumatics extends SubsystemBase {

    private Compressor compressor;

    /**
    *
    */
    public Pneumatics() {

        compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
        addChild("Compressor 1", compressor);

    }

    public void stopCompressor() {
        compressor.disable();
    }

    public void startCompressor() {
        compressor.enabled();

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
