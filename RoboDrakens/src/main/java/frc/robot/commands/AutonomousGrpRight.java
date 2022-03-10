
package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousGrpRight extends SequentialCommandGroup {

     public AutonomousGrpRight(DriveSubsystem drivesubsystem, ShootSubsystem shootsubsystem,
               IntakeSubsystem intakesubsystem) {
          addCommands(
                    new AutonomousShoot(shootsubsystem, 7),
                    new ParallelCommandGroup(
                         //     new AutonomousCmdTrajectoryFollowerTwoFixFile(drivesubsystem, "/home/lvuser/Pathright_1.csv"),
                              new AutonomousSimpleDrive(drivesubsystem),
                              new AutonomousIntakeBall(intakesubsystem)),
                    new AutonomousShoot(shootsubsystem, 10));

     }
}
