package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;


public class ToggleClimberPiston extends CommandBase {
    private final ClimberSubsystem m_climberSubsystem;

    public ToggleClimberPiston(ClimberSubsystem climberSubsystem) {
        m_climberSubsystem = climberSubsystem;
    }

    @Override
    public void initialize() {
        m_climberSubsystem.togglePiston();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
