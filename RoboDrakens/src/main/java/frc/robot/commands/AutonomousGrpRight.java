
package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class AutonomousGrpRight extends SequentialCommandGroup {

     public AutonomousGrpRight(DriveSubsystem subsystem) {
          addCommands(
        //  addParallel(new AutonomousCmdRunIntake());
        new AutonomousCmdTrajectoryFollowerTwoFixFile(subsystem, "/home/lvuser/Pathleft_1.csv"));
     }

}



