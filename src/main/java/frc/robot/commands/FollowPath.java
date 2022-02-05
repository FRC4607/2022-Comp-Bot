package frc.robot.commands;

import java.nio.file.Path;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FollowPathConstants;

public class FollowPath {


    public static RamseteCommand generateRamseteCommand(DrivetrainSubsystem drivetrain, Trajectory trajectory) {

        // Reset odometry to the starting pose of the trajectory.
        drivetrain.resetOdometry(trajectory.getInitialPose());
        
        RamseteCommand ramseteCommand = new RamseteCommand(
                trajectory,
                drivetrain::getPose,
                new RamseteController(FollowPathConstants.kRamseteB_radSquaredPerMetersSquared, FollowPathConstants.kRamseteZeta_PerRad),
                new SimpleMotorFeedforward(DriveConstants.ks_Volts, DriveConstants.kv_VoltSecondsPerMeters, DriveConstants.ka_VoltSecondsSquaredPerMeters),
                DriveConstants.kDriveKinematics,
                drivetrain::getWheelSpeeds,
                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                drivetrain::tankDriveVolts,
                drivetrain);
        return ramseteCommand;
    }

    public static RamseteCommand generateRamseteCommand(DrivetrainSubsystem drivetrain, Path pathweaverJSON) {
        Trajectory trajectory;
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(pathweaverJSON);
            return generateRamseteCommand(drivetrain, trajectory);
        } catch (Exception e) {
            System.err.println(e.getMessage());
            return null;
        }
    }
}
