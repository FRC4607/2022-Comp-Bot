package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ResetClimber extends CommandBase {
    
    ClimberSubsystem m_climberSubsystem;

    public ResetClimber(ClimberSubsystem climberSubsystem) {
        m_climberSubsystem = climberSubsystem;
    }

    @Override
    public void initialize() {
        m_climberSubsystem.resetEncoder();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
