package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ExtendClimber extends CommandBase {
    ClimberSubsystem m_climberSubsystem;

    public ExtendClimber(ClimberSubsystem climberSubsystem) {
        m_climberSubsystem = climberSubsystem;
    }

    @Override
    public void initialize() {
        m_climberSubsystem.setClutch(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
