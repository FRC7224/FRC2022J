
package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class AutonomousGrpCenter extends SequentialCommandGroup {

     public AutonomousGrpCenter(DriveSubsystem subsystem) {
          addCommands(
        //  addParallel(new AutonomousCmdRunIntake());
        new AutonomousCmdTrajectoryFollowerTwoFixFile(subsystem, "/home/lvuser/Pathcenter_1.csv"));
     }

}




