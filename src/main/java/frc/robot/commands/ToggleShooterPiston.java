package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ToggleShooterPiston extends CommandBase {
    private final ShooterSubsystem m_shooterSubsystem;

    public ToggleShooterPiston(ShooterSubsystem shooterSubsystem) {
        m_shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        m_shooterSubsystem.togglePsiton();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
