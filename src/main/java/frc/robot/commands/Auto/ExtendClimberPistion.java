package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ExtendClimberPistion extends CommandBase {
    private ClimberSubsystem m_climberSubsystem;
    private boolean m_extended;

    public ExtendClimberPistion(ClimberSubsystem climberSubsystem, boolean extended) {
        m_climberSubsystem = climberSubsystem;
        m_extended = extended;
    }

    @Override
    public void initialize() {
        m_climberSubsystem.setPistion(m_extended);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
