
package frc.robot.commands;

import frc.robot.Point;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousGrpFileGenerator extends  SequentialCommandGroup {

     public AutonomousGrpFileGenerator(DriveSubsystem subsystem) {
          addCommands(
     new AutonomousCmdTrajectoryFollowerFileGenerator(subsystem, new Point(0, 0, 0), new Point(45, 0, 0), "Pathleft_1"));
  //  new AutonomousCmdTrajectoryFollowerFileGenerator(subsystem, new Point(0, 0, 0), new Point(37.5,37.5, 45), "Pathcenter_1"));
    //  new AutonomousCmdTrajectoryFollowerFileGenerator(subsystem, new Point(0, 0, 0), new Point(41, 0, 0), "Pathright_1"));
     }

}
