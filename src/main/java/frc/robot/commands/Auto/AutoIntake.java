package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class AutoIntake extends CommandBase {
    private final IntakeSubsystem m_intakeSubsystem;
    private final TowerSubsystem m_towerSubsystem;

    public AutoIntake(IntakeSubsystem intakeSubsystem, TowerSubsystem towerSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
        m_towerSubsystem = towerSubsystem;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void execute() {
        if (m_towerSubsystem.getMidBrakeBeam() && m_towerSubsystem.getHighBrakeBeam()) {
            m_intakeSubsystem.setSpeed(0);
        } else {
            m_intakeSubsystem.setSpeed(IntakeConstants.intakeSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.setSpeed(0);
    }
}
