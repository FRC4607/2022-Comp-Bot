package frc.robot.commands.Auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Paths;
import frc.robot.Constants.FollowPathConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TestPath extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    public TestPath(DrivetrainSubsystem drivetrainSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        Trajectory SCurve = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                List.of(
                        new Translation2d(Units.feetToMeters(3), Units.feetToMeters(3)),
                        new Translation2d(Units.feetToMeters(6), Units.feetToMeters(6))),
                new Pose2d(Units.feetToMeters(9), Units.feetToMeters(9), Rotation2d.fromDegrees(0)),
                FollowPathConstants.trajectoryConfig.setReversed(false));

        Trajectory Strate = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)), List.of(),
                new Pose2d(Units.feetToMeters(9), Units.feetToMeters(0), Rotation2d.fromDegrees(0)),
                FollowPathConstants.trajectoryConfig.setReversed(false));

        
        m_drivetrainSubsystem.setBrakeMode(false);
        CommandScheduler.getInstance().schedule(new FollowPath(m_drivetrainSubsystem, Paths.Hub_Ball3_Ball4));

    }

    @Override
    public void execute() {
        
        // m_drivetrainSubsystem.tankDriveVolts(2, 2);

    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.setBrakeMode(false);
        m_drivetrainSubsystem.tankDriveVolts(0, 0);
    }
}
