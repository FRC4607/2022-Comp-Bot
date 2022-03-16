package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;

public class ToggleFlyweelPiston extends CommandBase {
    private final FlywheelSubsystem m_flywheelSubsystem;

    public ToggleFlyweelPiston(FlywheelSubsystem flywheelSubsystem) {
        m_flywheelSubsystem = flywheelSubsystem;
    }

    @Override
    public void initialize() {
        m_flywheelSubsystem.togglePsiton();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
