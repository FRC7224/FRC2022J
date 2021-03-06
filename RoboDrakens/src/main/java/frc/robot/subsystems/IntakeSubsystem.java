/*----------------------------------------------------------------------------*/
/* Copyright (c) 2022 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class IntakeSubsystem extends SubsystemBase {
   // for debug only 
  //  private WPI_TalonSRX conveyormotor = new WPI_TalonSRX(Constants.kConveyorMotorPort);
  //  private WPI_TalonSRX intakemotor = new WPI_TalonSRX(Constants.kIntakeMotorPort);

  private WPI_VictorSPX conveyormotor = new WPI_VictorSPX(Constants.kConveyorMotorPort);
  private WPI_VictorSPX intakemotor = new WPI_VictorSPX(Constants.kIntakeMotorPort);

    DigitalInput ballLoaded = new DigitalInput(Constants.kballsensorchannel);
    private Solenoid solenoidG = new Solenoid(0, PneumaticsModuleType.CTREPCM, Constants.kPneumaticsShootGate);
    private Solenoid solenoidI = new Solenoid(0, PneumaticsModuleType.CTREPCM, Constants.kPneumaticsIntake);

    public IntakeSubsystem() {

        addChild("Solenoid Gate", solenoidG);
        addChild("Solenoid Intake", solenoidI);
        addChild("Ball Switch", ballLoaded);
        addChild("Intake Motor", intakemotor);
        addChild("Conveyor motor", conveyormotor);
    }

    /**
     * sets the ball intake motor speed -1 to +1
     */
    public void setIntakeMotor(double ispeed) {
        intakemotor.set(ControlMode.PercentOutput, ispeed);
    }

    /**
     * sets the ball intake motor speed -1 to +1
     */
    public void setConveyorMotor(double convayorspeed) {
        conveyormotor.set(ControlMode.PercentOutput, convayorspeed);
    }

    /**
     * return the status if the ball is loaded in the shooter
     * sets gloabl LAUNCHREADY stste
     * 
     * @return
     */
    public boolean getballLoadstatus() {
        Constants.LAUNCHREADY = ballLoaded.get();
        return (ballLoaded.get());
    }

    /**
     * Closes the ball gate
     * 
     * @return
     */
    public void closeballgate() {
        solenoidG.set(true);
    }

    /**
     * Opens the ball gate
     * 
     * @return
     */
    public void openballgate() {
        solenoidG.set(false);
    }

    /**
     * Lower Intake
     * 
     * @return
     */
    public void lowerIntake() {
        solenoidI.set(true);
    }

    /**
     * Opens the ball gate
     * 
     * @return
     */
    public void raiseIntake() {
        solenoidI.set(false);
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
