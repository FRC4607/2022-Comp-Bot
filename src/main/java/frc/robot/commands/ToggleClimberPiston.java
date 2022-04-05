package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class ToggleClimberPiston extends CommandBase {
    private final ClimberSubsystem m_climberSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;

    public ToggleClimberPiston(ClimberSubsystem climberSubsystem, IntakeSubsystem intakeSubsystem) {
        m_climberSubsystem = climberSubsystem;
        m_intakeSubsystem = intakeSubsystem;
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
