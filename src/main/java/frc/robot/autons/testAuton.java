package frc.robot.autons;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.driveCommands.cmdDriveStraight;
import frc.robot.commands.driveCommands.cmdStop;

public class testAuton extends SequentialCommandGroup {
    
    public testAuton() {
        addCommands(new cmdDriveStraight(24, 0.3, 0));
        addCommands(new cmdStop());
        //addCommands(new cmdDriveStraight(24, -0.3, 0));
        //addCommands(new cmdStop());
    }
}
