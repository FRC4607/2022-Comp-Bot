package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class CalibateTrackwidth extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private double m_startingRotation;
    private double m_endingRoatition;
    private boolean m_clockwise;

    public CalibateTrackwidth(DrivetrainSubsystem drivetrainSubsystem, boolean clockwise) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(m_drivetrainSubsystem);

        m_clockwise = clockwise;
    }

    @Override
    public void initialize() {
        m_drivetrainSubsystem.resetEncoders();
        m_startingRotation = m_drivetrainSubsystem.getGyroscopeReadingContinuous();
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.tankDriveVolts(m_clockwise ? 2 : -2, m_clockwise ? -2 : 2);
    }

    @Override
    public void end(boolean interrupted) {
        m_endingRoatition = m_drivetrainSubsystem.getGyroscopeReadingContinuous();

        double AverageDistanceTravled = (m_drivetrainSubsystem.getLeftEncoderPosition() - m_drivetrainSubsystem.getRightEncoderPosition()) / 2;
        double rotations = (m_endingRoatition - m_startingRotation)/360;
        double trackWidth = AverageDistanceTravled / (rotations * Math.PI);

        SmartDashboard.putNumber("Track Width", trackWidth);
        // SmartDashboard.putNumber("Number of Rotations", rotations);
        // SmartDashboard.putNumber("Average Distave Travled", AverageDistanceTravled);
    }

}
