package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AgitatorConstants;
import frc.robot.subsystems.AgitatorSubsystem;

public class RunAgitator extends CommandBase {
    private final boolean m_reverse;
    private final AgitatorSubsystem m_agitatorSubsystem;

    public RunAgitator(AgitatorSubsystem agitatorSubsystem, boolean reverse) {
        m_agitatorSubsystem = agitatorSubsystem;
        m_reverse = reverse;
        addRequirements(m_agitatorSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (!m_reverse) {
            m_agitatorSubsystem.setSpeed(AgitatorConstants.agitatorSpeed);
        }
        else {
            m_agitatorSubsystem.setSpeed(-AgitatorConstants.agitatorSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_agitatorSubsystem.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

