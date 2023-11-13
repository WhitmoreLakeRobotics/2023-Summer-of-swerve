package frc.robot.commands.driveCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMath;

/**
 *
 */
public class cmdDriveStraight extends CommandBase {
    private double targetPosition = 0; // inches
    private double power_X = 0.0;
    private double power_Y = 0.0;
    private double targetHeading = 0;
    private boolean bDone = false;
    // private double overshootValue = 0;
    // private WL_Spark.IdleMode idleMode = WL_Spark.IdleMode.kBrake;
    private Pose2d startingPose2d_meters = null;


    public cmdDriveStraight(double targetDistance, double speed_X, double heading) {

        targetPosition = targetDistance;
        power_X = speed_X;
        power_Y = 0.0;
        targetHeading = heading;

        // m_subsystem = subsystem;
        // addRequirements(m_subsystem);
    }

    public cmdDriveStraight(double targetDistance, double speed_X, double speed_Y, double heading) {

        targetPosition = targetDistance;
        power_X = speed_X;
        power_Y = speed_Y;
        targetHeading = heading;

        // m_subsystem = subsystem;
        // addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        bDone = false;
        // since we are doing a delta pose to tell if we are done there should be no need to 
        // reset encoders.
        // RobotContainer.getInstance().m_robotDrive.resetEncoders();

        // Starting Pose is in meters
        startingPose2d_meters = RobotContainer.getInstance().m_robotDrive.getPose();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double headingDelta = RobotMath.calcTurnRate(RobotContainer.getInstance().m_robotDrive.m_gyro.getAngle(),
                targetHeading, RobotContainer.getInstance().m_robotDrive.kp_driveStraightGyro);

        // double headingDelta = 0;

        RobotContainer.getInstance().m_robotDrive.drive(power_X,power_Y, headingDelta, false, true);
        if (Math.abs(RobotContainer.getInstance().m_robotDrive.getDistanceTraveledInches(startingPose2d_meters)) >= Math
                .abs(targetPosition)) {
            bDone = true;
            RobotContainer.getInstance().m_robotDrive.stopDrive();
        }

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
