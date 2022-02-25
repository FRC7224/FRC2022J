/*----------------------------------------------------------------------------*/
/* Copyright (c) 2022 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

/**
 *
 */
public class ClimbSubsystem extends SubsystemBase {

    /** Hardware */
    WPI_TalonFX climbMotor = new WPI_TalonFX(6, "rio");
    Joystick _joy = new Joystick(0);

    /** Used to create string thoughout loop */
    StringBuilder _sb = new StringBuilder();
    int _loops = 0;

    /**
    *
    */
    /**
     * sets the climb motor speed -1 to +1
     */

    public ClimbSubsystem() {

        /**
         * Description:
         * The PositionClosedLoop example demonstrates the Position closed-loop servo.
         * Tested with Logitech F350 USB Gamepad inserted into Driver Station
         * 
         * Be sure to select the correct feedback sensor using
         * configSelectedFeedbackSensor() below.
         * Use Percent Output Mode (Holding A and using Left Joystick) to confirm talon
         * is driving
         * forward (Green LED on Talon/Victor) when the position sensor is moving in the
         * positive
         * direction. If this is not the case, flip the boolean input in
         * setSensorPhase().
         * 
         * Controls:
         * Button 1: When pressed, start and run Position Closed Loop on Talon/Victor
         * Button 2: When held, start and run Percent Output
         * Left Joytick Y-Axis:
         * + Position Closed Loop: Servo Talon forward and reverse [-10, 10] rotations
         * + Percent Ouput: Throttle Talon forward and reverse
         * 
         * Gains for Position Closed Loop may need to be adjusted in Constants.java
         * 
         * 
         */

        /* Factory Default all hardware to prevent unexpected behaviour */
        climbMotor.configFactoryDefault();

        /* Config the sensor used for Primary PID and sensor direction */
        climbMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                Constants.kClimbPIDLoopIdx,
                Constants.kClimbTimeoutMs);

        /* Ensure sensor is positive when output is positive */
        climbMotor.setSensorPhase(Constants.kClimbSensorPhase);

        /**
         * Set based on what direction you want forward/positive to be.
         * This does not affect sensor phase.
         */
        climbMotor.setInverted(Constants.kClimbMotorInvert);

        /* Config the peak and nominal outputs, 12V means full */
        climbMotor.configNominalOutputForward(0, Constants.kClimbTimeoutMs);
        climbMotor.configNominalOutputReverse(0, Constants.kClimbTimeoutMs);
        climbMotor.configPeakOutputForward(1, Constants.kClimbTimeoutMs);
        climbMotor.configPeakOutputReverse(-1, Constants.kClimbTimeoutMs);

        /**
         * Config the allowable closed-loop error, Closed-Loop output will be
         * neutral within this range. See Table in Section 17.2.1 for native
         * units per rotation.
         */
        climbMotor.configAllowableClosedloopError(0, Constants.kClimbPIDLoopIdx, Constants.kClimbTimeoutMs);

        /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
        climbMotor.config_kF(Constants.kClimbPIDLoopIdx, Constants.kClimbF, Constants.kClimbTimeoutMs);
        climbMotor.config_kP(Constants.kClimbPIDLoopIdx, Constants.kClimbP, Constants.kClimbTimeoutMs);
        climbMotor.config_kI(Constants.kClimbPIDLoopIdx, Constants.kClimbI, Constants.kClimbTimeoutMs);
        climbMotor.config_kD(Constants.kClimbPIDLoopIdx, Constants.kClimbD, Constants.kClimbTimeoutMs);
       

        /**
         * Grab the 360 degree position of the MagEncoder's absolute
         * position, and intitally set the relative sensor to match.
         */
        // double absolutePosition =
        // climbMotor.getSensorCollection().getIntegratedSensorAbsolutePosition();

        /* Mask out overflows, keep bottom 12 bits */
        // absolutePosition &= 0xFFF;
        // if (Constants.kClimbSensorPhase) {
        // absolutePosition *= -1;
        // }
        // if (Constants.kClimbMotorInvert) {
        // absolutePosition *= -1;
        // }

        /* Set the quadrature (relative) sensor to match absolute */
        // climbMotor.setSelectedSensorPosition(absolutePosition,
        // Constants.kClimbPIDLoopIdx, Constants.kClimbTimeoutMs);
    }

    public void straightClimb(double output) {
        climbMotor.set(ControlMode.PercentOutput, output);
    }

    public void pidClimb(double targetPositionRotations) {
        climbMotor.set(ControlMode.Position, targetPositionRotations);

        /* If Talon is in position closed-loop, print some more info */
        if (climbMotor.getControlMode() == ControlMode.Position) {
            /* ppend more signals to print when in speed mode. */
            _sb.append("\terr:");
            _sb.append(climbMotor.getClosedLoopError(0));
            _sb.append("u"); // Native Units

            _sb.append("\ttrg:");
            _sb.append(targetPositionRotations);
            _sb.append("u"); /// Native Units
        }

        /**
         * Print every ten loops, printing too much too fast is generally bad
         * for performance.
         */
        if (++_loops >= 10) {
            _loops = 0;
            System.out.println(_sb.toString());
        }

        /* Reset built string for next loop */
        _sb.setLength(0);
    }

    public void climbMotorOutput() {

        /* Get Talon/Victor's current output percentage */
        double motorOutput = climbMotor.getMotorOutputPercent();

        /* Prepare line to print */
        _sb.append("\tout:");
        /* Cast to int to remove decimal places */
        _sb.append((int) (motorOutput * 100));
        _sb.append("%"); // Percent

        _sb.append("\tpos:");
        _sb.append(climbMotor.getSelectedSensorPosition(0));
        _sb.append("u"); // Native units

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
