package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FollowPathConstants;

public class FollowPath extends CommandBase {

    private DrivetrainSubsystem m_drivetrain;

    private DifferentialDriveVoltageConstraint m_voltageConstraint;
    private TrajectoryConfig m_trajectoryConfig;
    private Trajectory m_exampleTrajectory;
    RamseteCommand m_ramseteCommand;

    public FollowPath(DrivetrainSubsystem drivertrain) {
        m_drivetrain = drivertrain;
        addRequirements(m_drivetrain);

        m_voltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka),
                DriveConstants.kDriveKinematics, DriveConstants.maxVoltage);

        m_trajectoryConfig = new TrajectoryConfig(
                FollowPathConstants.kMaxSpeed_MetersPerSecond,
                FollowPathConstants.kMaxAcceleration_MetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(DriveConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(m_voltageConstraint);

        m_exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(
                        new Translation2d(1, 1),
                        new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                m_trajectoryConfig);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Reset odometry to the starting pose of the trajectory.
        m_drivetrain.resetOdometry(m_exampleTrajectory.getInitialPose());

        m_ramseteCommand = new RamseteCommand(
                m_exampleTrajectory,
                m_drivetrain::getPose,
                new RamseteController(FollowPathConstants.kRamseteB, FollowPathConstants.kRamseteZeta),
                new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka),
                DriveConstants.kDriveKinematics,
                m_drivetrain::getWheelSpeeds,
                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_drivetrain::tankDriveVolts,
                m_drivetrain);
        CommandScheduler.getInstance().schedule(m_ramseteCommand);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_ramseteCommand.isFinished();
    }
}
