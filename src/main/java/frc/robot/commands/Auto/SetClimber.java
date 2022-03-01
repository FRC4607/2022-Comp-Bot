package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimberState;

public class SetClimber extends CommandBase {
    private final ClimberSubsystem m_climberSubsystem;
    private final boolean extended;

    public SetClimber(ClimberSubsystem climberSubsystem, boolean extended) {
        m_climberSubsystem = climberSubsystem;
        this.extended = extended;
    }

    @Override
    public void initialize() {
        m_climberSubsystem.extendClimber(extended);
    }

    @Override
    public boolean isFinished() {
        return (m_climberSubsystem.climberState == ClimberState.Extended && extended) || (m_climberSubsystem.climberState == ClimberState.Retracted && !extended);
    }
}