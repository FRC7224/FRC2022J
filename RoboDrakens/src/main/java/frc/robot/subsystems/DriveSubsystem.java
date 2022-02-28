/*----------------------------------------------------------------------------*/
/* Copyright (c) 2022 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.SPI;

/**
 *
 */
public class DriveSubsystem extends SubsystemBase {

  // private WPI_TalonFX right1; // right 1
  // private WPI_TalonFX right2; // right 2
  // private WPI_TalonFX left1; // left 1
 //  private WPI_TalonFX temp; // left 2
  

// WPI_TalonFX left1 = new WPI_TalonFX(Constants.kLeftMotor1Port);
// WPI_TalonFX left2 = new WPI_TalonFX(Constants.kLeftMotor2Port);
// WPI_TalonFX right1 = new WPI_TalonFX(Constants.kRightMotor1Port);
// WPI_TalonFX right2 = new WPI_TalonFX(Constants.kRightMotor2Port);

 WPI_TalonSRX left1 = new WPI_TalonSRX(Constants.kLeftMotor1Port);
 WPI_TalonSRX left2 = new WPI_TalonSRX(Constants.kLeftMotor2Port);
 WPI_TalonSRX right1 = new WPI_TalonSRX(Constants.kRightMotor1Port);
 WPI_TalonSRX right2 = new WPI_TalonSRX(Constants.kRightMotor2Port);
 

  // The motors on the left side of the drive.
 MotorControllerGroup m_leftMotors = new MotorControllerGroup( left1, left2);


  // The motors on the right side of the drive.
  MotorControllerGroup m_rightMotors = new MotorControllerGroup(right1,right2);

     

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  
  // The left-side drive encoder
  private final Encoder m_leftEncoder = new Encoder(
      Constants.kLeftEncoderPorts[0],
      Constants.kLeftEncoderPorts[1],
      Constants.kLeftEncoderReversed);

  // The right-side drive encoder
  private final Encoder m_rightEncoder = new Encoder(
      Constants.kRightEncoderPorts[0],
      Constants.kRightEncoderPorts[1],
      Constants.kRightEncoderReversed);

  // The gyro sensor
 // private final Gyro m_gyro = new ADXRS450_Gyro();
  public AHRS  m_gyro = new AHRS(SPI.Port.kMXP);
 


  // Initialize your subsystem here
  public DriveSubsystem() {

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.setInverted(true);


    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);

    // Setup live windoww items
    addChild("Left Encoder", m_leftEncoder);
    addChild("Right Encoder", m_rightEncoder);
    addChild("Drive Subsytem", m_drive);
    addChild("Left Motor", m_leftMotors);
    addChild("Right Motor", m_rightMotors);

  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Drives the robot using tank controls.
   *
   * @param left  movement
   * @param right movement
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the raw left drive encoder.
   *
   * @return the raw left drive encoder
   */
  public int getLeftEncoderPosition() {
    return m_leftEncoder.getRaw();
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Gets the raw right drive encoder.
   *
   * @return the right drive encoder
   */
  public int getRightEncoderPosition() {
    return m_rightEncoder.getRaw();
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (Constants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    SmartDashboard.putNumber("PID drive turnrate", m_gyro.getRate() * (Constants.kGyroReversed ? -1.0 : 1.0));
    return m_gyro.getRate() * (Constants.kGyroReversed ? -1.0 : 1.0);
  }

  

  // setup
  public void setupDrive() {
    m_leftMotors.set(0);
    m_rightMotors.set(0);
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

  public void autoshift() {
    // auto down shift only
    // if ((Constants.shiftOpenState = true) &&
    // (Math.abs(leftEncoder.getRate()) < Constants.kshiftRateDown)) {
    // Robot.shifter.closeShifter();
    // }
  }

  public void displayDriveData() {
    // These are the new encoders
    SmartDashboard.putNumber("Left Encoder Scaled", getLeftEncoder().getDistance());
    SmartDashboard.putNumber("Right Encoder Sclaed",getRightEncoder().getDistance());
    SmartDashboard.putNumber("Left Encoder Raw",getLeftEncoderPosition());
    SmartDashboard.putNumber("Right Encoder Raw",getRightEncoderPosition());;
    SmartDashboard.putNumber("Chassis angle", getHeading());
  }

}
