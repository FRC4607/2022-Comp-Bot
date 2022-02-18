package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FollowPathConstants;

public class Paths {
        
        private static final Pose2d hub_alt = new Pose2d(Units.inchesToMeters(40.479), Units.inchesToMeters(-24.173), Rotation2d.fromDegrees(157.5));
        // new Pose2d(Units.inchesToMeters(46.479), Units.inchesToMeters(-12.173),Rotation2d.fromDegrees(157.5)),
        private static final Pose2d start   = new Pose2d(0, 0, Rotation2d.fromDegrees(180));
        private static final Pose2d hub     = new Pose2d(Units.inchesToMeters(44.707695), Units.inchesToMeters(-10.024630),Rotation2d.fromDegrees(157.5));
        private static final Pose2d hubB    = new Pose2d(Units.inchesToMeters(40.707695), Units.inchesToMeters(8.024630),Rotation2d.fromDegrees(202.5));
        
        private static final Pose2d ball2   = new Pose2d(Units.inchesToMeters(-42), Units.inchesToMeters(0),Rotation2d.fromDegrees(180));
        private static final Pose2d ball3   = new Pose2d(Units.inchesToMeters(-10.883155), Units.inchesToMeters(93.708250),Rotation2d.fromDegrees(98.5));
        private static final Pose2d ball4   = new Pose2d(Units.inchesToMeters(-30.863335), Units.inchesToMeters(242.360533), Rotation2d.fromDegrees(132.251156));

        public static Trajectory twoBall0;
        public static Trajectory twoBall1_A;
        public static Trajectory twoBall1_B;
        public static Trajectory threeBall2;
        public static Trajectory threeBall3;
        public static Trajectory fourBall2;
        public static Trajectory fourBall3;

        public static void generateTrajectories() {
                twoBall0   = TrajectoryGenerator.generateTrajectory(List.of(start, ball2), FollowPathConstants.trajectoryConfig.setReversed(false));
                twoBall1_A = TrajectoryGenerator.generateTrajectory(List.of(ball2, hub), FollowPathConstants.trajectoryConfig.setReversed(true));
                twoBall1_B = TrajectoryGenerator.generateTrajectory(List.of(ball2, hubB), FollowPathConstants.trajectoryConfig.setReversed(true));
                threeBall2 = TrajectoryGenerator.generateTrajectory(List.of(hub, ball3), FollowPathConstants.trajectoryConfig.setReversed(false));
                threeBall3 = TrajectoryGenerator.generateTrajectory(List.of(ball3, hub_alt), FollowPathConstants.trajectoryConfig.setReversed(true));
                fourBall2 =  TrajectoryGenerator.generateTrajectory(hub, List.of(new Translation2d(Units.inchesToMeters(-10.883155), Units.inchesToMeters(93.708250))), ball4, FollowPathConstants.trajectoryConfig.setReversed(false));
                fourBall3 =  TrajectoryGenerator.generateTrajectory(List.of( ball4, hub ), FollowPathConstants.trajectoryConfig.setReversed(true));
        }
}
