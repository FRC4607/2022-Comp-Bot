package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimberState;

public class ZeroClimber extends CommandBase {
    private final ClimberSubsystem m_climberSubsystem;
    
    public ZeroClimber(ClimberSubsystem climberSubsystem) {
        m_climberSubsystem = climberSubsystem;
    }

    @Override
    public void initialize() {
        m_climberSubsystem.extendClimber(false);
    }

    @Override
    public boolean isFinished() {
        return m_climberSubsystem.climberState == ClimberState.Retracted;
    }
}
