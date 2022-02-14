
package frc.robot.commands;

import frc.robot.Point;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousGrpFileGenerator extends  SequentialCommandGroup {

     public AutonomousGrpFileGenerator(DriveSubsystem subsystem) {
          addCommands(
        new AutonomousCmdTrajectoryFollowerFileGenerator(subsystem, new Point(0, 0, 0), new Point(3, 3, 0), "Pathleft_1"),
        new AutonomousCmdTrajectoryFollowerFileGenerator(subsystem, new Point(0, 0, 0), new Point(6, 6, 0), "Pathcenter_1"),
        new AutonomousCmdTrajectoryFollowerFileGenerator(subsystem, new Point(0, 0, 0), new Point(9, 9, 0), "Pathright_1"));
     }

}
