/*----------------------------------------------------------------------------*/
/* Copyright (c) 2022 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Zone;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

/**
 *
 */
public class ShootSubsystem extends SubsystemBase {


    private WPI_TalonFX shootMotorTop= new WPI_TalonFX(5,"rio");
    private WPI_TalonFX shootMotorBottom = new WPI_TalonFX(6,"rio");
    private WPI_VictorSPX elvMotor = new WPI_VictorSPX(7);
    private Solenoid solenoidP = new Solenoid(0, PneumaticsModuleType.CTREPCM, Constants.kPneumaticsShootPush);
    Zone[] zones = {
    new Zone(Constants.kT0,Constants.kB0),
    new Zone(Constants.kT1,Constants.kB1),
    new Zone(Constants.kT2,Constants.kB2),
    new Zone(Constants.kT3,Constants.kB3),
    new Zone(Constants.kT4,Constants.kB4),
    new Zone(Constants.kT5,Constants.kB5),
    new Zone(Constants.kT6,Constants.kB6),
    new Zone(Constants.kT7,Constants.kB7),
    new Zone(Constants.kT8,Constants.kB8),
    new Zone(Constants.kT9,Constants.kB9),
    new Zone(Constants.kT10,Constants.kB10),
    new Zone(Constants.kT11,Constants.kB11),
    new Zone(Constants.kT12,Constants.kB12),
    new Zone(Constants.kT13,Constants.kB13),
    new Zone(Constants.kT14,Constants.kB14),
    new Zone(Constants.kT15,Constants.kB15),
    new Zone(Constants.kT16,Constants.kB16),
    new Zone(Constants.kT17,Constants.kB17),
    new Zone(Constants.kT18,Constants.kB18),
    new Zone(Constants.kT19,Constants.kB19),
    new Zone(Constants.kT20,Constants.kB20),
    new Zone(Constants.kT21,Constants.kB21),
    new Zone(Constants.kT22,Constants.kB22),
    new Zone(Constants.kT23,Constants.kB23),
    new Zone(Constants.kT24,Constants.kB24),
    new Zone(Constants.kT25,Constants.kB25),
   ///  special short zone
    new Zone(Constants.kT26,Constants.kB26),
};
 

    /**
    *
    */
    public  ShootSubsystem() {
 
        addChild("Shoot Motor Top", shootMotorTop );
        addChild("Shoot Motor Bottom", shootMotorBottom );
        addChild("Solenoid Push", solenoidP);
      
        /**
         * sets up shooter with PID
         */

        shootMotorTop.configFactoryDefault();
        shootMotorTop.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        shootMotorTop.set(ControlMode.Velocity, 0);
        shootMotorTop.setInverted(true);
        shootMotorTop.setSensorPhase(true);

        /* Config the peak and nominal outputs */
        shootMotorTop.configNominalOutputForward(0, Constants.kTimeoutMs);
        shootMotorTop.configNominalOutputReverse(0, Constants.kTimeoutMs);
        shootMotorTop.configPeakOutputForward(100, Constants.kTimeoutMs);
        shootMotorTop.configPeakOutputReverse(0, Constants.kTimeoutMs);

        /* Config the Velocity closed loop gains in slot0 */
        shootMotorTop.config_kF(Constants.kPIDLoopIdx, Constants.kshootTopF, Constants.kTimeoutMs);
        shootMotorTop.config_kP(Constants.kPIDLoopIdx, Constants.kshootTopP, Constants.kTimeoutMs);
        shootMotorTop.config_kI(Constants.kPIDLoopIdx, Constants.kshootTopI, Constants.kTimeoutMs);
        shootMotorTop.config_kD(Constants.kPIDLoopIdx, Constants.kshootTopD, Constants.kTimeoutMs);

        shootMotorBottom.configFactoryDefault();
        shootMotorBottom.set(ControlMode.Velocity, 0);
        shootMotorBottom.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        shootMotorBottom.setSensorPhase(true);
        shootMotorBottom.setInverted(false);

        /* Config the peak and nominal outputs */
        shootMotorBottom.configNominalOutputForward(0, Constants.kTimeoutMs);
        shootMotorBottom.configNominalOutputReverse(0, Constants.kTimeoutMs);
        shootMotorBottom.configPeakOutputForward(100, Constants.kTimeoutMs);
        shootMotorBottom.configPeakOutputReverse(0, Constants.kTimeoutMs);

        /* Config the Velocity closed loop gains in slot0 */
        shootMotorBottom.config_kF(Constants.kPIDLoopIdx, Constants.kshootBottomF, Constants.kTimeoutMs);
        shootMotorBottom.config_kP(Constants.kPIDLoopIdx, Constants.kshootBottomP, Constants.kTimeoutMs);
        shootMotorBottom.config_kI(Constants.kPIDLoopIdx, Constants.kshootBottomI, Constants.kTimeoutMs);
        shootMotorBottom.config_kD(Constants.kPIDLoopIdx, Constants.kshootBottomD, Constants.kTimeoutMs);

    }

    /**
     * sets the shooter speed
     */
    public void setShootSpeed( int zoneposition) {
        // shootMotor1.set(speed);
        shootMotorTop.set(ControlMode.Velocity, zones[zoneposition].getTopMotor());
        shootMotorBottom.set(ControlMode.Velocity, zones[zoneposition].getBottomMotor());
    }


    /**
     * stop the shooter speed
     */
    public void stopshooter() {
        shootMotorTop.set(ControlMode.Velocity, 0);
        shootMotorBottom.set(ControlMode.Velocity,0);
    }



     /**
      * sets the elevator speed
      */
      public void setelvSpeed(final double espeed) {
        elvMotor.set(ControlMode.PercentOutput, espeed);
   }

 /**
     * Pushed the Ball to shooter
     * 
     * @return
     */
    public void pushBall() {
        solenoidP.set(true);
    }

    /**
     * Resets the Ball shooter
     * 
     * @return
     */
    public void resetBallPush() {
        solenoidP.set(false);
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
