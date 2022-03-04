
package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class AutonomousGrpLeft extends SequentialCommandGroup {

     public AutonomousGrpLeft(DriveSubsystem subsystem) {
          addCommands(
        //  addParallel(new AutonomousCmdRunIntake());
        new AutonomousCmdTrajectoryFollowerTwoFixFile(subsystem, "/home/lvuser/Pathleft_1.csv"));
     }

}


/** A complex auto command that drives forward, releases a hatch, and then drives backward. 
public class ComplexAutoCommand extends SequentialCommandGroup {
     /**
      * Creates a new ComplexAutoCommand.
      *

      */

      /** 
     public ComplexAutoCommand(DriveSubsystem driveSubsystem, HatchSubsystem hatchSubsystem) {
       addCommands(
           // Drive forward up to the front of the cargo ship
           new FunctionalCommand(
               // Reset encoders on command start
               driveSubsystem::resetEncoders,
               // Drive forward while the command is executing
               () -> driveSubsystem.arcadeDrive(AutoConstants.kAutoDriveSpeed, 0),
               // Stop driving at the end of the command
               interrupt -> driveSubsystem.arcadeDrive(0, 0),
               // End the command when the robot's driven distance exceeds the desired value
               () ->
                   driveSubsystem.getAverageEncoderDistance()
                       >= AutoConstants.kAutoDriveDistanceInches,
               // Require the drive subsystem
               driveSubsystem),
   
           // Release the hatch
           new InstantCommand(hatchSubsystem::releaseHatch, hatchSubsystem),
   
           // Drive backward the specified distance
           new FunctionalCommand(
               // Reset encoders on command start
               driveSubsystem::resetEncoders,
               // Drive backward while the command is executing
               () -> driveSubsystem.arcadeDrive(-AutoConstants.kAutoDriveSpeed, 0),
               // Stop driving at the end of the command
               interrupt -> driveSubsystem.arcadeDrive(0, 0),
               // End the command when the robot's driven distance exceeds the desired value
               () ->
                   driveSubsystem.getAverageEncoderDistance()
                       <= AutoConstants.kAutoBackupDistanceInches,
               // Require the drive subsystem
               driveSubsystem));
     }
   }

   */



