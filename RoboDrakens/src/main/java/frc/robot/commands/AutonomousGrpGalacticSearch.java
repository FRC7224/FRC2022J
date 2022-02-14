/*
package org.usfirst.frc.team7224.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team7224.robot.grip.GripPipeline;

public class AutonomousGrpGalacticSearch extends CommandGroup {

    public AutonomousGrpGalacticSearch() { 

        // vision logic determines with case we're in
        //GripPipeline pipeline = new GripPipeline();
        int PathType = 2;// pipeline.RunPipeline();
        SmartDashboard.putNumber("Path Chosen", PathType);
        switch (PathType) {
            case 0:
            // all files for path a1
            addSequential(new AutonomousCmdTrajectoryFollowerTwoFixFile("PathA1_1"));
            break;
            case 1:
            // all files for path a2
            addSequential(new AutonomousCmdTrajectoryFollowerTwoFixFile("PathA2_1"));
            break;
            case 2:
            // all files for path b1
            for (int i = 1; i < 5; i++){
                addSequential(new AutonomousCmdTrajectoryFollowerTwoFixFile(String.format("/home/lvuser/PathB1_%d.csv", i)));
            }
           
            break;
            case 3:
            // all files for path b2
            addSequential(new AutonomousCmdTrajectoryFollowerTwoFixFile("PathB2_1"));
            break;
        }
        
    }

}

*/