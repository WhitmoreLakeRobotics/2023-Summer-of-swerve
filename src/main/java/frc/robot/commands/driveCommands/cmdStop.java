package frc.robot.commands.driveCommands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/**
 *
 */
public class cmdStop extends CommandBase {
    private boolean bDone = false;

    public cmdStop() {

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        bDone = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        RobotContainer.getInstance().m_robotDrive.stopDrive();
        bDone = true;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.getInstance().m_robotDrive.stopDrive();
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
