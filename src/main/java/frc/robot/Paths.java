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
                new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
                List.of(),
                new Pose2d(Units.inchesToMeters(-42), Units.inchesToMeters(0), Rotation2d.fromDegrees(180)),
                FollowPathConstants.trajectoryConfig.setReversed(false));

        public static Trajectory twoBall1_A = TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(-42), Units.inchesToMeters(0), Rotation2d.fromDegrees(180)),
                List.of(),
                new Pose2d(Units.inchesToMeters(46.479), Units.inchesToMeters(-12.173),Rotation2d.fromDegrees(157.5)),
                FollowPathConstants.trajectoryConfig.setReversed(true));

        public static Trajectory twoBall1_B = TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(-42), Units.inchesToMeters(0), Rotation2d.fromDegrees(180)),
                List.of(),
                new Pose2d(Units.inchesToMeters(46.498), Units.inchesToMeters(12.180),Rotation2d.fromDegrees(202.5)),
                FollowPathConstants.trajectoryConfig.setReversed(true));

        public static Trajectory threeBall2 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(46.479), Units.inchesToMeters(-12.173), Rotation2d.fromDegrees(157.5)),
                List.of(),
                new Pose2d(Units.inchesToMeters(9.883), Units.inchesToMeters(88.708), Rotation2d.fromDegrees(98.5)),
                FollowPathConstants.trajectoryConfig.setReversed(false));

        public static Trajectory threeBall3 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(9.883), Units.inchesToMeters(88.708), Rotation2d.fromDegrees(98.5)),
                List.of(),
                //new Pose2d(Units.inchesToMeters(46.479), Units.inchesToMeters(-12.173),Rotation2d.fromDegrees(157.5)),
                new Pose2d(Units.inchesToMeters(40.479), Units.inchesToMeters(-24.173),Rotation2d.fromDegrees(157.5)),
                FollowPathConstants.trajectoryConfig.setReversed(true));
}
