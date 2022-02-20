package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.PistonState;

public class ToggleClimberPiston extends CommandBase {
    ClimberSubsystem m_climberSubsystem;

    public ToggleClimberPiston(ClimberSubsystem climberSubsystem) {
        m_climberSubsystem = climberSubsystem;
    }

    @Override
    public void initialize() {
        m_climberSubsystem.setPistion(m_climberSubsystem.pistonState == PistonState.Extended);
    }

}
