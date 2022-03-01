package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ToggelClutch extends CommandBase {
    private final ClimberSubsystem m_climberSubsystem;

    public ToggelClutch(ClimberSubsystem climberSubsystem) {
        m_climberSubsystem = climberSubsystem;
    }
    @Override
    public void initialize() {
        m_climberSubsystem.toggleClutch();
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
