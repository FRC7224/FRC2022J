// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be
 * declared globally (i.e. public static). Do not put anything functional in
 * this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {

   /**
    * 
    CAN ID constants
    */
    // Drive
    public static int kRightMotor1Port = 1;
    public static int kRightMotor2Port = 2;
    public static int kLeftMotor1Port = 3;
    public static int kLeftMotor2Port = 4;
    // Shooter
    public static int kShootMotorTopPort = 5;
    public static int kShootMotorBottomPort = 6;
    public static int kShootMotorElevatorPort = 7;
    // intake
    public static int kIntakeMotorPort = 8;
    public static int kConveyorMotorPort = 9;
    // Light
    public static int kLightPort = 10;
    // Climb
    public static int kClimblockPort = 11;
    public static int kClimbMotorPort = 12;


 




    // Pneumatics

    public static int kPneumaticsShootGate = 2;
    public static int kPneumaticsShootPush = 3;
    public static int kPneumaticsIntake = 1;
    public static int kPneumaticsShift =  0;




    // Joystick 1 constants
    public static double kdeadzone = 0.1; // Deadzone

    public static int kshiftbutton = 1;
    public static int kintakeupbutton = 2;
    public static int kdrvautobutton = 3;
    public static int kclimboverridebutton = 4;

    public static int kintakebutton = 5;
    public static int kconveyerbutton = 6;
    public static int kshortshootbutton = 7;
    public static int kinitShooter = 8;
    public static int kautobutton = 9;
    public static int kclimbButton = 10;
    public static int kclimbrelease = 11;
    public static int kclimbLock = 12;
  

/***
 * 
 * Global state varibles
 */
public static Boolean LAUNCHED = false;
public static Boolean LAUNCHREADY = false;




    // Drive Subsystem


    public static boolean kenablePID = false;

 //   public static final int[] kLeftEncoderPorts = new int[] { 2, 3 };
 //   public static final int[] kRightEncoderPorts = new int[] { 0, 1 };
 //   public static final boolean kLeftEncoderReversed = false;
 //   public static final boolean kRightEncoderReversed = true;

   // public static final int kEncoderCPR = 348; // 12.41:1 Gear box, 1:3 encoder ratio, encoder 360 CPR
                                               // Ratio A 12.41 Ratio B 5.45
                                               // 360/12.41 * 3 * 4 =
 //   public static final double kWheelDiameterInches = 4;
 //   public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
  //          (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;

    public static final boolean kGyroReversed = false;

    public static final double kTurnP = .05;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;

    public static final double kMaxTurnRateDegPerS = 100;
    public static final double kMaxTurnAccelerationDegPerSSquared = 300;

    public static final double kTurnToleranceDeg = 2;
    public static final double kTurnRateToleranceDegPerS = 10; // degrees per second

    // Shifter
    public static double kshiftRateUp = 1200;
    public static double kshiftRateDown = 800;
    public static boolean shiftOpenState = false;

    // Autonomous
    public static int TrajectorySegments;
    public static boolean isTrajectory = false;
    public static int kencodermode = 0;

    // Climb

    public static double kMaxClimbHeight = 151000 ;
    public static double kMinClimbHeight = 0;
    public static int kClimbTimeoutMs = 30;
    public static double kClimbP = 0.15;
    public static double kClimbI = 0;
    public static double kClimbD = 1.0;
    public static double kClimbF = 0.0;
    public static double kIzone = 0.0;
    public static double  kPeakOutput = 0.5;
    public static boolean kClimbSensorPhase  = true ;
    public static boolean kClimbMotorInvert = true; 
    public static int  kClimbSlotIdx = 0;
    public static int  kClimbPIDLoopIdx = 0;
    public static double kclimbreleaseP = 0.5;


    // Intake
    public static double kIntakeSpeed = -0.5;
    public static double kConveyorSpeed = -0.4;
    public static int kballsensorchannel = 4;

    // Move Ball to shooter

    public static int kballIntakeTimer_timer = 10;

    // Shooter / Elevator Constants
    public static double kB0 = 2000;
    public static double kB1 = 7000;
    public static double kB2 = 7000;
    public static double kB3 = 7000;
    public static double kB4 = 7000;
    public static double kB5 = 7000;
    public static double kB6 = 7000;
    public static double kB7 = 7000;
    public static double kB8 = 9500;
    public static double kB9 = 10000;
    public static double kB10 = 10500;
    public static double kB11 = 11000;
    public static double kB12 = 11500;
    public static double kB13 = 12000;
    public static double kB14 = 12500;
    public static double kB15 = 13000;
    public static double kB16 = 13500;
    public static double kB17 = 14000;
    public static double kB18 = 15000;
    public static double kB19 = 16000;
    public static double kB20 = 17000;
    public static double kB21 = 18000;
    public static double kB22 = 19000;
    public static double kB23 = 20000;
    public static double kB24 = 21000;
    public static double kB25 = 22000;
    public static double kT0 = 1000;
    public static double kT1 = 3000;
    public static double kT2 = 3000;
    public static double kT3 = 3000;
    public static double kT4 = 3000;
    public static double kT5 = 3000;
    public static double kT6 = 3000;
    public static double kT7 = 3000;;
    public static double kT8 = 3450;
    public static double kT9 = 3600;
    public static double kT10 = 3750;
    public static double kT11 = 3900;
    public static double kT12 = 4050;
    public static double kT13 = 4200;
    public static double kT14 = 4350;
    public static double kT15 = 4500;
    public static double kT16 = 4650;
    public static double kT17 = 4800;
    public static double kT18 = 5200;
    public static double kT19 = 5600;
    public static double kT20 = 6000;
    public static double kT21 = 6400;
    public static double kT22 = 6800;
    public static double kT23 = 7200;
    public static double kT24 = 7600;
    public static double kT25 = 8000;
    //
    //
    public static double kB26 = 500;
    public static double kT26 = 1000;

    public static int kshortshootzone = 26;
    public static double kelvspeed = -0.7;

    public static double shooterTolerance = 300.0;
    public static double kshooterTimer_spin = 1.2;  /// was 1.2
    public static double kshooterTimer_timer = 2.2;  /// was 1.2
    public static boolean shooterMode = false;
    public static int kPIDLoopIdx = 0;
    public static int kTimeoutMs = 30;
    public static double kshootTopP = 0.25;
    public static double kshootTopI = 0.001;
    public static double kshootTopD = 20;
    public static double kshootTopF = 1023.0 / 7200.0;
    //
    public static double kshootBottomP = 0.25;
    public static double kshootBottomI = 0.001;
    public static double kshootBottomD = 20;
    public static double kshootBottomF = 1023.0 / 7200.0;
    public static int rightencoder;
    public static int leftencoder;



   //  New drive to point
   // These are example values only - NEED to UPDATE
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kp = 1;

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;   

}
