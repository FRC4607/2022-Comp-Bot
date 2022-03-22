package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.Constants.TowerConstants;

public class OutakeBalls extends CommandBase {
    private final IntakeSubsystem m_intakeSubsystem;
    private final TowerSubsystem m_towerSubsystem;

    public OutakeBalls(IntakeSubsystem intakeSubsystem, TowerSubsystem towerSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
        m_towerSubsystem = towerSubsystem;

        addRequirements(m_intakeSubsystem, m_towerSubsystem);
    }

    @Override
    public void initialize() {
        m_intakeSubsystem.setState(IntakeState.Releasing);
        m_towerSubsystem.setSpeed(-TowerConstants.agitatiorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.setState(IntakeState.Idle);
        m_towerSubsystem.setSpeed(0);
    }
}
