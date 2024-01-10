package frc.robot.commands.util;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotMath;

/**
 *
 */
public class cmdDelay extends Command {
    private boolean bDone = false;
    private double seconds2Delay = 0 ;
    private double endTime = 0;

    public cmdDelay(double seconds) {
        bDone = false;
        seconds2Delay = Math.abs(seconds);
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        bDone = false;
        endTime = RobotMath.getTime() + seconds2Delay;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        if (endTime <= RobotMath.getTime()) {
            bDone = true;
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        bDone = true;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return bDone;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
