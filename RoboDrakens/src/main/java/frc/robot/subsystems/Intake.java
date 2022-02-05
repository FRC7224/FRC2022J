/*----------------------------------------------------------------------------*/
/* Copyright (c) 2022 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
//import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import edu.wpi.first.wpilibj.Solenoid;

/**
 *
 */
public class Intake extends SubsystemBase {

    private WPI_VictorSPX intakemotor;
    private WPI_VictorSPX conveyormotor;
  //  private Solenoid solenoidG;
 //   private DigitalInput ballSwitch;

    /**
     * sets the ball intake motor speed -1 to +1
     */
    public void setIntakeMotor(double ispeed) {
        intakemotor = new WPI_VictorSPX(7);
        intakemotor.set(ControlMode.PercentOutput, ispeed);
    }

 /**
     * sets the ball intake motor speed -1 to +1
     */
    public void setConveyorMotor(double conspeed) {
        conveyormotor = new WPI_VictorSPX(8);
        conveyormotor.set(ControlMode.PercentOutput, conspeed);
    }






    // public void placeholder (double temp) {
    // solenoidG = new Solenoid(0, PneumaticsModuleType.CTREPCM, 2);
    // addChild("Solenoid G", solenoidG);
    // ballSwitch = new DigitalInput(0);
    // addChild("Ball Switch", ballSwitch);
    // }
    


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
