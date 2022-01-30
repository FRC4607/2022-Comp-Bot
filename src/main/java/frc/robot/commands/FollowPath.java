package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FollowPathConstants;

public class FollowPath extends CommandBase {

    private DrivetrainSubsystem m_drivetrain;

    private Trajectory m_trajectory;
    RamseteCommand m_ramseteCommand;

    public FollowPath(DrivetrainSubsystem drivertrain, Trajectory trajectory) {
        m_drivetrain = drivertrain;
        addRequirements(m_drivetrain);

        m_trajectory = trajectory;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Reset odometry to the starting pose of the trajectory.
        m_drivetrain.resetOdometry(m_trajectory.getInitialPose());

        m_ramseteCommand = new RamseteCommand(
                m_trajectory,
                m_drivetrain::getPose,
                new RamseteController(),
                new SimpleMotorFeedforward(DriveConstants.ks_Volts, DriveConstants.kv_VoltSecondsPerMeters, DriveConstants.ka_VoltSecondsSquaredPerMeters),
                DriveConstants.kDriveKinematics,
                m_drivetrain::getWheelSpeeds,
                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_drivetrain::tankDriveVolts,
                m_drivetrain);
        CommandScheduler.getInstance().schedule(m_ramseteCommand);

        
    }

    @Override
    public boolean isFinished() {
        // Returns true when the command should end.
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        m_ramseteCommand.cancel();
    }
}
