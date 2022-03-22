package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimberPosition;

public class RetractClimber extends CommandBase {
    private final ClimberSubsystem m_climberSubsystem;

    public RetractClimber(ClimberSubsystem climberSubsystem) {
        m_climberSubsystem = climberSubsystem;
        
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        m_climberSubsystem.setPosition(ClimberPosition.retracted);
    }
    
    @Override
    public void end(boolean interrupted) {
        m_climberSubsystem.DisablePositionContorl();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
