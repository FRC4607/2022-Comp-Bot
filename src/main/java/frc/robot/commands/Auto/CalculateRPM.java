package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import oi.limelightvision.limelight.frc.LimeLight;

public class CalculateRPM extends CommandBase {

    private LimeLight m_limeLight;
    private ShooterSubsystem m_shooterSubsystem;

    public CalculateRPM(LimeLight limeLight, ShooterSubsystem shooterSubsystem) {
        m_limeLight = limeLight;
        m_shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        m_limeLight.setPipeline(1);
    }

    @Override
    public void execute() {
        if (m_limeLight.getIsTargetFound()) {
            double dV_in = 104 - 25; // Meshered
            double angle_deg = 31.5 + m_limeLight.getdegVerticalToTarget(); // Meshered
            double dD_in = dV_in / Math.tan(Math.toRadians(angle_deg)) - 4;
    
            SmartDashboard.putNumber("Limelight Distance", dD_in);
    
            double RPM = -0.052043 * Math.pow(dD_in, 2) + 34.3271 * dD_in + 217.264;
            m_shooterSubsystem.setLimeLightRPM(RPM);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_limeLight.setPipeline(0);
    }

    @Override
    public boolean isFinished() {
        return m_limeLight.getIsTargetFound();
    }
}