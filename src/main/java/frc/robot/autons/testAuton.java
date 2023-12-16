package frc.robot.autons;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.driveCommands.cmdDriveStraight;
import frc.robot.commands.driveCommands.cmdStop;
import frc.robot.commands.util.cmdDelay;

public class testAuton extends SequentialCommandGroup {
    
    public testAuton() {
        addCommands(new cmdDriveStraight(12, 0.15, 0));
        addCommands(new cmdStop());
        addCommands(new cmdDelay(1.0));
        addCommands(new cmdDriveStraight(12, -0.15, 0));
        addCommands(new cmdStop());
        addCommands(new cmdDelay(1.0));
        // once the command can drive forward 24 inches then return 
        // to the starting spot by driving backwards 24 inches
        
        //addCommands(new cmdDriveStraight(24, -0.3, 0));
        //addCommands(new cmdStop());
        //addCommands(new cmdDelay(1.0));
    }
}
