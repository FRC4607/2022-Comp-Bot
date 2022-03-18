package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class SetClimberPiston extends CommandBase {
    private final ClimberSubsystem m_climberSubsystem;
    private boolean m_extended;
    private Timer m_timer;
    
    public SetClimberPiston(ClimberSubsystem climberSubsystem, boolean extended) {
        m_climberSubsystem = climberSubsystem;

        m_extended = extended;

        m_timer = new Timer();
    }

    @Override
    public void initialize() {
        m_climberSubsystem.setPistion(m_extended);
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(ClimberConstants.pistionEstenchonTime);
    }
}
