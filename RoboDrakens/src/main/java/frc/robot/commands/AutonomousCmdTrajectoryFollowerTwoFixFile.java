/*----------------------------------------------------------------------------*/
/* Copyright (c) 2022 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


import frc.robot.subsystems.DriveSubsystem;
import java.io.File;
import java.io.IOException;
import java.util.Timer;
import java.util.TimerTask;

import frc.robot.Constants;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;


/**
 *
 */
public class AutonomousCmdTrajectoryFollowerTwoFixFile extends CommandBase {
 
  private final DriveSubsystem m_drivesubsystem;
 
  public AutonomousCmdTrajectoryFollowerTwoFixFile( DriveSubsystem subsystem, String fileName) {
    m_drivesubsystem = subsystem;
    addRequirements(m_drivesubsystem);      

    FileName = fileName;
  }

  edu.wpi.first.wpilibj.Timer timeout;

  Timer t;
  int gyrowait;
  double lasttime;
  double timelapse;
  public double maxtimeout = 8;
  EncoderFollower left;
  EncoderFollower right;
  public String FileName = "";
  // This has a max size of three
  Waypoint[] waypoints = new Waypoint[2];

 
  

  @Override
public void initialize() {

    timeout = new edu.wpi.first.wpilibj.Timer();
    timeout.start();
    Constants.isTrajectory = true;
    m_drivesubsystem.setupDrive();
    Constants.TrajectorySegments = 0;
    // ***********************************************
    // FRC 2020 First Power up game decision
    // Determine Switch, Scale or far Switch
    // ***********************************************
    SmartDashboard.putString("debug output", "selecting file");
    // switch (Filenum) {
    // case 0: // Drive straight
    // fileString = "/home/lvuser/mytrafile.csv";
    // maxtimeout = 10;
    // break;

    // default: // Default drive straight
    // fileString = "/home/lvuser/driveStraight.traj";
    // maxtimeout = 10;
    // break;
    // } // end of switch

    File myFile = new File(FileName);

    // Waypoint[] points = waypoints;
    // Trajectory.Config config = new
    // Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
    // Trajectory.Config.SAMPLES_LOW,
    // 0.05, 1.2, .5, .4);
    // Trajectory trajectory = Pathfinder.generate(points, config);
    try {
      SmartDashboard.putString("debug output", "opening file " + myFile.getAbsolutePath());
      // Trajectory trajectory = Pathfinder.readFromFile(myFile);
      // try {
      // trajectory = Pathfinder.readFromFile(myFile);
      // SmartDashboard.putString("debug output", "read from file");
      // } catch (IOException e) {
      // SmartDashboard.putString("debug output", "file read crashed");
      // }

      Trajectory trajectory = Pathfinder.readFromCSV(myFile);
      // Waypoint[] points = waypoints;
      // Trajectory.Config config = new
      // Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
      // Trajectory.Config.SAMPLES_LOW,0.05,1.2, .5, .4);
      // Trajectory trajectory = Pathfinder.generate(points, config);

      double wheelbase_width = .66; // MG updated

      // Create the Modifier Object
      TankModifier modifier = new TankModifier(trajectory).modify(wheelbase_width);

      left = new EncoderFollower(modifier.getLeftTrajectory());
      right = new EncoderFollower(modifier.getRightTrajectory());

      // Encoder Position is the current, cumulative position of your encoder.
      // If you're using an SRX, this will be the
      // 'getEncPosition' function.
      // 1440 is the amount of encoder ticks per full revolution
      // 1440 ticks per rev * 3:1 gear ratio *4x = 1920
      // Wheel Diameter is the diameter of your wheels (or pulley for a track system)
      // in meters
      // 6" * .0254 = .1524

      left.configureEncoder(m_drivesubsystem.getLeftEncoderPosition(), 365, 0.1524);
      right.configureEncoder(m_drivesubsystem.getRightEncoderPosition(), 365, 0.1525);

      // The first argument is the proportional gain. Usually this will be quite high
      // The second argument is the integral gain. This is unused for motion profiling
      // The third argument is the derivative gain. Tweak this if you are unhappy with
      // the tracking of the trajectory
      // The fourth argument is the velocity ratio. This is 1 over the maximum
      // velocity you provided in the
      // trajectory configuration (it translates m/s to a -1 to 1 scale that your
      // motors can read)
      // The fifth argument is your acceleration gain. Tweak this if you want to get
      // to a higher or lower speed quicker
      left.configurePIDVA(0.7, 0.0, 0.00, 0.6, 0);
      right.configurePIDVA(0.7, 0.0, 0.00, 0.6, 0);

      t = new Timer();
      m_drivesubsystem.resetEncoders();
      m_drivesubsystem.resetEncoders();
      m_drivesubsystem.brakemode(true);
      t.schedule(new TimerTask() {
        // Sample setup
        // double l = left.calculate(encoder_position_left);
        // double r = right.calculate(encoder_position_right);
        //
        // double gyro_heading = ... your gyro code here ... // Assuming the gyro is
        // giving a value in degrees
        // double desired_heading = Pathfinder.r2d(left.getHeading()); // Should also be
        // in degrees
        //
        // double angleDifference = Pathfinder.boundHalfDegrees(desired_heading -
        // gyro_heading);
        // double turn = 0.8 * (-1.0/80.0) * angleDifference;
        //
        // setLeftMotors(l + turn);
        // setRightMotors(r - turn);
        ////// End sample setup

        double l = 0;
        double r = 0;
        double gyro_heading = 0;

        public void run() {
          if (!left.isFinished() || !right.isFinished()) {
            l = left.calculate(m_drivesubsystem.getLeftEncoderPosition());
            r = right.calculate(m_drivesubsystem.getRightEncoderPosition());
            double desired_heading = Pathfinder.r2d(left.getHeading()); // Should also be in degrees
            double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
            double turn = 0.8 * (-1.0 / 80.0) * angleDifference;
            m_drivesubsystem.tankDrive((l + turn), (r - turn));
            SmartDashboard.putNumber("tra r ", r);
            SmartDashboard.putNumber("tra l ", l);
            SmartDashboard.putNumber("tra head", desired_heading);
            SmartDashboard.putNumber("tra angle Difference", angleDifference);
            SmartDashboard.putNumber("tra gyro ", m_drivesubsystem.getHeading());
            SmartDashboard.putNumber("tra encoder right", m_drivesubsystem.getRightEncoderPosition());
            SmartDashboard.putNumber("tra encodeer left", m_drivesubsystem.getLeftEncoderPosition());

          } else {
            m_drivesubsystem.tankDrive(0, 0);
            left.reset();
            right.reset();
            t.cancel();
          }
        }

      }, 0, 50); // end timed task 0 delay, execute every 50 mSec
      left.reset();
      right.reset();
      m_drivesubsystem.resetEncoders();
      m_drivesubsystem.displayDriveData();
      // Constants.kMaxSpeed_a = Constants.kNormalArm_a; // Normal arm
    } catch (IOException exc) {
      SmartDashboard.putString("debug output", "File reading error: " + exc.getMessage());
    }
  }

  @Override
  public void execute() {
  }

  @Override
  public boolean isFinished() {
    if (left.isFinished() || right.isFinished() || (timeout.get() > maxtimeout)) {
      m_drivesubsystem.resetEncoders();
      t.cancel();
      return true;
    }
    return false;
  }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {   
    timeout.stop();
    timeout.reset();
    t.cancel();
    Constants.isTrajectory = false;
    m_drivesubsystem.resetEncoders();
    m_drivesubsystem.brakemode(false);
  }
 

}
