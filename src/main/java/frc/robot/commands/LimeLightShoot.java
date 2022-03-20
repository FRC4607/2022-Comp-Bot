package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShootingMode;
import oi.limelightvision.limelight.frc.LimeLight;

public class LimeLightShoot extends CommandBase {
    private LimeLight m_limeLight;
    private final TowerSubsystem m_towerSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;

    public LimeLightShoot(LimeLight limeLight, TowerSubsystem towerSubsystem, ShooterSubsystem shooterSubsystem) {
        m_limeLight = limeLight;
        m_towerSubsystem = towerSubsystem;
        m_shooterSubsystem = shooterSubsystem;

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        double dV_in = 104 - 24; // Not Meshered
        double angle_deg = 35 + m_limeLight.getdegVerticalToTarget(); // Not Meshered
        double dD_in = dV_in / Math.tan(Units.degreesToRadians(angle_deg));
        
        m_shooterSubsystem.setLimeLightRPM(1000);
        m_shooterSubsystem.setShootingMode(ShootingMode.limeLight);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_limeLight.setPipeline(0);
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
