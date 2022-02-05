/*----------------------------------------------------------------------------*/
/* Copyright (c) 2022 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;

/**
 *
 */
public class Chassis extends PIDSubsystem {

    // private WPI_TalonFX right1; // right 1
    // private WPI_TalonFX right2; // right 2
    // private WPI_TalonFX left1; // left 1
    // private WPI_TalonFX left2; // left 2

    private WPI_TalonSRX right1; // right 1
    private WPI_TalonSRX right2; // right 2
    private WPI_TalonSRX left1; // left 1
    private WPI_TalonSRX left2; // left 2

    private ADXRS450_Gyro gyro;
    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private boolean turning = true;
    Timer setPointTimer = new Timer();

    // Initialize your subsystem here
    public Chassis() {
        super(new PIDController(Constants.kp, Constants.ki, Constants.kd));
        getController().setTolerance(Constants.kgyroPIDErrorTolerance);
        getController().enableContinuousInput(-180, 180);
        getController().setIntegratorRange(-1.0, 1.0);

        // Create wheels
        right1 = new WPI_TalonSRX(1);
        right2 = new WPI_TalonSRX(2);
        left1 = new WPI_TalonSRX(3);
        left2 = new WPI_TalonSRX(4);

        // left2 = new WPI_TalonFX(4);

        // create encoders

        rightEncoder = new Encoder(new DigitalInput(0), new DigitalInput(1), false,
                Encoder.EncodingType.k4X);
        leftEncoder = new Encoder(new DigitalInput(2), new DigitalInput(3), true,
                Encoder.EncodingType.k4X);

        // Setup Gyro
        gyro = new ADXRS450_Gyro();
        addChild("Gyro", gyro);
        gyro.calibrate();

        // Initialize encoders
        resetEncoders();

    }

    public void disablePID() {
        if (getController().isContinuousInputEnabled()) {
            getController().disableContinuousInput();
            getController().reset(); // disables and resets integral
        }
    }

    public void enablePID() {
        if (!getController().isContinuousInputEnabled()) {
            gyro.reset();
            getController().enableContinuousInput(-180, 180);
            getController().setTolerance(1.0);
            setSetpoint(0.0);
        }
    }

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public double getGyroAngle() {
        return gyro.getAngle();
    }

    public void resetgyro() {
        gyro.reset();
    }

    // setup
    public void setupDrive() {
        left1.set(0);
        left2.follow(left1);
        right1.set(0);
        right2.follow(right1);
        gyro.reset();
        resetEncoders();
        brakemode(false);
    }

    public void brakemode(boolean brakemode) {
        if (brakemode) {
            left1.setNeutralMode(NeutralMode.Brake);
            left2.setNeutralMode(NeutralMode.Brake);
            right1.setNeutralMode(NeutralMode.Brake);
            right2.setNeutralMode(NeutralMode.Brake);
        } else {
            left1.setNeutralMode(NeutralMode.Coast);
            left2.setNeutralMode(NeutralMode.Coast);
            right1.setNeutralMode(NeutralMode.Coast);
            right2.setNeutralMode(NeutralMode.Coast);

        }
    }

    public double deadZone(double input) {
        double d = Math.abs(input);
        if (d < Constants.kdeadzone) {
            return 0.0;
        }
        return input;
    }

    public void arcadeDrive(double forward, double turn) {

        if (Constants.kenablePID == true) { // use PID process
            // **********************************************************
            // * PID Proccessing
            // *
            // **********************************************************
            //
            if (turn == 0.0) { // *** Not turning ****
                if (turning == true) { // Code is called first time, when we stopped turning
                    setPointTimer.start();
                    Constants.gyroPIDOutput = 0.0; // Reset PIDOutput to zero
                    turning = false; // Set turning to false, because we are not
                                     // turning any more
                    // SmartDashboard.putBoolean("turnin",turning );
                } else if (setPointTimer.get() != 0) {// If this isn't the first time
                    if (setPointTimer.get() >= 1.0) { // Robot is moving straight
                                                      // wait for timer before turning on PID
                        enablePID();
                        gyro.reset();
                        getController().setSetpoint(0);
                        setPointTimer.stop();
                        setPointTimer.reset();
                        // SmartDashboard.putNumber("setpoittimer", setPointTimer.get() );
                    }
                } else { // after initializing ** Driving straight using PID
                    turn = Constants.gyroPIDOutput;
                }
                // ELSE the user is still commanding
                // User is commanding a turn
            } else if (turn != 0.0) {
                // SmartDashboard.putNumber("turn in elseif",turn );
                disablePID();
                setPointTimer.stop();
                setPointTimer.reset();
                turning = true;
                // Reset angle
            }
            displayChasisData();
            arcade(forward, turn); // PID controlled Drive
        } // End of BasicDrive PID Control
          // ELSE PID is Off
        else { // use standard arcadeDrive
               // * PID off mode *****
            displayChasisData();
            arcade(forward, turn);
        }
    } // End of PID enable loop\

    public void autoshift() {
        // auto down shift only
        // if ((Constants.shiftOpenState = true) &&
        // (Math.abs(leftEncoder.getRate()) < Constants.kshiftRateDown)) {
        // Robot.shifter.closeShifter();
        // }
    }

    public int getLeftEncoderPosition() {
        int intValue = 0;
        intValue = (int) leftEncoder.get();
        return intValue;

    }

    public int getRightEncoderPosition() {
        int intValue = 0;
        intValue = (int) rightEncoder.get();
        return intValue;

    }

    public void displayChasisData() {
        // These are the new encoders
        SmartDashboard.putNumber("LEFT ENCODER", leftEncoder.get());
        SmartDashboard.putNumber("RIGTH ENCODER", rightEncoder.get());
        SmartDashboard.putNumber("Chassis angle", gyro.getAngle());
        SmartDashboard.putNumber("Chassis gyro setpoint", getSetpoint());
        SmartDashboard.putNumber("Chassis turn", Constants.gyroPIDOutput);
        SmartDashboard.putNumber("Drive Rate", leftEncoder.getRate());

    }

    public void setLeftRightMotorOutputs(double leftOutput, double rightOutput) {
        left1.set(limit(-leftOutput));
        right1.set(limit(rightOutput));

    }

    /*********************************************************************************
     * Method to check if input is in range
     * 
     */
    protected static double limit(double number) {
        if (number >= 1.0) {
            return 1.0;
        } else if (number <= -1.0) {
            return -1.0;
        }
        return number;
    }

    /**
     * More advanced method to control the robot with just one joystick
     */

    public void arcade(double moveValue, double rotateValue) {

        boolean squaredInputs = false;
        moveValue = limit(moveValue);
        rotateValue = limit(rotateValue);
        if (squaredInputs) {
            if (moveValue >= 0.0D) {
                moveValue *= moveValue;
            } else {
                moveValue = -(moveValue * moveValue);
            }

            if (rotateValue >= 0.0D) {
                rotateValue *= rotateValue;
            } else {
                rotateValue = -(rotateValue * rotateValue);
            }
        }

        double rightMotorSpeed;
        double leftMotorSpeed;

        if (moveValue > 0.0D) {
            if (rotateValue > 0.0D) {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = Math.max(moveValue, rotateValue);
            } else {
                leftMotorSpeed = Math.max(moveValue, -rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            }
        } else {

            if (rotateValue > 0.0D) {
                leftMotorSpeed = -Math.max(-moveValue, rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            } else {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
            }
        }

        setLeftRightMotorOutputs(leftMotorSpeed, rightMotorSpeed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        super.periodic();

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    @Override
    public double getMeasurement() {
        return gyro.getAngle();
    }

    // @Override
    public void useOutput(double output, double setpoint) {
        Constants.gyroPIDOutput = output;
    }

/**
      * Simple method to drive the robot like a tank
      * 
      */
      public void tankDrive(double leftValue, double rightValue) {

        /**
         * boolean squaredInputs = false; leftValue = limit(leftValue); rightValue =
         * limit(rightValue); if (squaredInputs) { leftValue *= leftValue; } else {
         * leftValue = -(leftValue * leftValue); } if (rightValue >= 0.0D) { rightValue
         * *= rightValue; } else { rightValue = -(rightValue * rightValue); }
         */
        setLeftRightMotorOutputs(leftValue, rightValue);
   }




}
