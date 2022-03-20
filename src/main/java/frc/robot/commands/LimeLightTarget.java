package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import oi.limelightvision.limelight.frc.LimeLight;

public class LimeLightTarget extends CommandBase {
    private LimeLight m_limeLight;
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final XboxController m_driver;

    public LimeLightTarget(LimeLight limeLight, DrivetrainSubsystem drivetrainSubsystem, XboxController driver) {
        m_limeLight = limeLight;
        m_drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(m_drivetrainSubsystem);

        m_driver = driver;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_limeLight.setPipeline(1);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double yAxis = m_driver.getLeftY();
        double xAxis;
        if (m_limeLight.getIsTargetFound()) {
            double angle = m_limeLight.getdegRotationToTarget();
            xAxis = (angle * 0.07) + Math.copySign(.25, angle);
        } else {
            xAxis = m_driver.getLeftX();
        }
        m_drivetrainSubsystem.setArcadeDrive(xAxis, yAxis);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_limeLight.setPipeline(0);
        m_drivetrainSubsystem.setArcadeDrive(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;

    }
}
