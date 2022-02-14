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
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

/**
 *
 */
public class ShootSubsystem extends SubsystemBase {

    private Solenoid solenoidS;
    private WPI_TalonFX shootMotorTop;
    private WPI_TalonFX shootMotorBottom;
    private WPI_VictorSPX elvMotor;

    /**
    *
    */
    public void setupShooter() {
        shootMotorTop = new WPI_TalonFX(5);
        addChild("Shoot Motor Top", shootMotorTop );
        shootMotorBottom = new WPI_TalonFX(6);
        addChild("Shoot Motor Bottom", shootMotorBottom );
        elvMotor = new WPI_VictorSPX(7);
        solenoidS = new Solenoid(0, PneumaticsModuleType.CTREPCM, 1);
        addChild("Solenoid S", solenoidS);
        


        /**
         * sets up shooter with PID
         */

        shootMotorTop.configFactoryDefault();
        shootMotorTop.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        shootMotorTop.set(ControlMode.Velocity, 0);
        shootMotorTop.setInverted(true);
        shootMotorTop.setSensorPhase(true);

        /* Config the peak and nominal outputs */
        shootMotorTop.configNominalOutputForward(0, Constants.kTimeoutMs);
        shootMotorTop.configNominalOutputReverse(0, Constants.kTimeoutMs);
        shootMotorTop.configPeakOutputForward(34000, Constants.kTimeoutMs);
        shootMotorTop.configPeakOutputReverse(0, Constants.kTimeoutMs);

        /* Config the Velocity closed loop gains in slot0 */
        shootMotorTop.config_kF(Constants.kPIDLoopIdx, Constants.kshootTopF, Constants.kTimeoutMs);
        shootMotorTop.config_kP(Constants.kPIDLoopIdx, Constants.kshootTopP, Constants.kTimeoutMs);
        shootMotorTop.config_kI(Constants.kPIDLoopIdx, Constants.kshootTopI, Constants.kTimeoutMs);
        shootMotorTop.config_kD(Constants.kPIDLoopIdx, Constants.kshootTopD, Constants.kTimeoutMs);

        shootMotorBottom.configFactoryDefault();
        shootMotorBottom.set(ControlMode.Velocity, 0);
        shootMotorBottom.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        shootMotorBottom.setSensorPhase(true);
        shootMotorBottom.setInverted(false);

        /* Config the peak and nominal outputs */
        shootMotorBottom.configNominalOutputForward(0, Constants.kTimeoutMs);
        shootMotorBottom.configNominalOutputReverse(0, Constants.kTimeoutMs);
        shootMotorBottom.configPeakOutputForward(34000, Constants.kTimeoutMs);
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
    public void setShootSpeed(final double topspeed, double bottomspeed) {
        // shootMotor1.set(speed);
        shootMotorTop.set(ControlMode.Velocity, topspeed);
        shootMotorBottom.set(ControlMode.Velocity, bottomspeed);
    }

     /**
      * sets the elevator speed
      */
      public void setelvSpeed(final double espeed) {
        elvMotor.set(ControlMode.PercentOutput, espeed);
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
