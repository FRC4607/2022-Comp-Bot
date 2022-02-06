package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FollowPathConstants;

public class Paths {
    public static Trajectory twoBall0 = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Units.inchesToMeters(-42), Units.inchesToMeters(0), Rotation2d.fromDegrees(180)),
        // Pass config
        FollowPathConstants.trajectoryConfig);
    public static Trajectory twoBall1 = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(Units.inchesToMeters(-42), Units.inchesToMeters(0), Rotation2d.fromDegrees(180)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(43.797), Units.inchesToMeters(-5.683), Rotation2d.fromDegrees(157.5)),
            // Pass config
            FollowPathConstants.trajectoryConfig.setReversed(true));
}
