package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberTrigers extends CommandBase {
    private ClimberSubsystem m_climberSubsystem;
    private XboxController m_operator;

    public ClimberTrigers(ClimberSubsystem climberSubsystem, XboxController operator) {
        m_climberSubsystem = climberSubsystem;
        m_operator = operator;

        addRequirements(m_climberSubsystem);
    }

    @Override
    public void execute() {
        double speed = m_operator.getLeftTriggerAxis() - m_operator.getRightTriggerAxis();
        m_climberSubsystem.setClimber(speed * 0.5);
    }
}
