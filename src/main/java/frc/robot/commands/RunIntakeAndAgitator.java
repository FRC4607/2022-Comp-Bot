package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeAndAgitator extends CommandBase {
    private final boolean m_reverse;
    private final IntakeSubsystem m_intakeSubsystem;

    public RunIntakeAndAgitator(IntakeSubsystem intakeSubsystem, boolean reverse) {
        m_intakeSubsystem = intakeSubsystem;
        m_reverse = reverse;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (!m_reverse) {
            m_intakeSubsystem.setSpeed(IntakeConstants.intakeSpeed);
            m_intakeSubsystem.setAgitator(IntakeConstants.agitatorSpeed);
        }
        else {
            m_intakeSubsystem.setSpeed(-IntakeConstants.intakeSpeed);
            m_intakeSubsystem.setAgitator(-IntakeConstants.agitatorSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.setSpeed(0);
        m_intakeSubsystem.setAgitator(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

