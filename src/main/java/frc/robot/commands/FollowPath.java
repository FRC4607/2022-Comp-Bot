package frc.robot.commands;

import java.nio.file.Path;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FollowPath extends CommandBase {

    private final Timer m_timer = new Timer();
    private final Trajectory m_trajectory;
    private DifferentialDriveWheelSpeeds m_prevSpeeds;
    private double m_prevTime;

    private DrivetrainSubsystem m_drivetrainSubsystem;
    // Reset odometry to the starting pose of the trajectory.

    public FollowPath(DrivetrainSubsystem drivetrainSubsystem, Trajectory trajectory) {
        m_trajectory = trajectory;
        m_drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(m_drivetrainSubsystem);
    }

    public FollowPath(DrivetrainSubsystem drivetrain, Path pathweaverJSON) {
        m_drivetrainSubsystem = drivetrain;
        Trajectory trajectory;
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(pathweaverJSON);
        } catch (Exception e) {
            System.err.println(e.getMessage());
            trajectory = null;
        }
        m_trajectory = trajectory;
    }

    @Override
    public void initialize() {
        m_drivetrainSubsystem.resetOdometry(m_trajectory.getInitialPose());
        m_prevTime = -1;
        var initialState = m_trajectory.sample(0);
        m_prevSpeeds = m_drivetrainSubsystem.getKinematics().toWheelSpeeds(
                new ChassisSpeeds(
                        initialState.velocityMetersPerSecond,
                        0,
                        initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
        m_timer.reset();
        m_timer.start();
        m_drivetrainSubsystem.resetPID();
    }

    @Override
    public void execute() {
        double curTime = m_timer.get();
        double dt = curTime - m_prevTime;

        if (m_prevTime < 0) {
            m_drivetrainSubsystem.tankDriveVolts(0.0, 0.0);
            m_prevTime = curTime;
            return;
        }

        // Ramset Controler
        DifferentialDriveWheelSpeeds targetWheelSpeeds = m_drivetrainSubsystem.getRamsetTargetWheelSpeeds(m_trajectory.sample(curTime));

        double leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        double rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

        SmartDashboard.putNumber("Left Ramsette Setpoint", leftSpeedSetpoint);
        SmartDashboard.putNumber("Right Ramsette Setpoint", rightSpeedSetpoint);

        // Feed Forward
        double leftFeedforward = m_drivetrainSubsystem.Feedforward().calculate(
                leftSpeedSetpoint, (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);

        double rightFeedforward = m_drivetrainSubsystem.Feedforward().calculate(
                rightSpeedSetpoint, (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);

        SmartDashboard.putNumber("Left FF Setpoint", leftFeedforward);
        SmartDashboard.putNumber("Right FF Setpoint", rightFeedforward);

        // PID
        double leftPID = m_drivetrainSubsystem.getLeftPID(leftSpeedSetpoint);
        double rightPID = m_drivetrainSubsystem.getRightPID(rightSpeedSetpoint);

        SmartDashboard.putNumber("Left PID", leftPID);
        SmartDashboard.putNumber("Right PID", rightPID);

        double leftOutput = leftFeedforward + leftPID;
        double rightOutput = rightFeedforward + rightPID;

        SmartDashboard.putNumber("Left Output", leftOutput);
        SmartDashboard.putNumber("Right Output", rightOutput);

        m_drivetrainSubsystem.tankDriveVolts(leftOutput, rightOutput);
        m_prevSpeeds = targetWheelSpeeds;
        m_prevTime = curTime;
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();

        if (interrupted) {
            m_drivetrainSubsystem.tankDriveVolts(0.0, 0.0);
        }
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }
}
