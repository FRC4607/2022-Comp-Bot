package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class Auton_TwoBall extends CommandBase {
    private static CommandScheduler m_commandScheduler;

    private static FlywheelSubsystem m_flywheelSubsystem;
    private static TowerSubsystem m_towerSubsystem;
    private static IntakeSubsystem m_intakeSubsystem;

    public Auton_TwoBall(FlywheelSubsystem flywheelSubsystem, TowerSubsystem towerSubsystem, IntakeSubsystem intakeSubsystem) {
        m_commandScheduler = CommandScheduler.getInstance();

        m_flywheelSubsystem = flywheelSubsystem;
        m_towerSubsystem = towerSubsystem;
        m_intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        m_commandScheduler.schedule(new SequentialCommandGroup(
            new SetIntake(m_intakeSubsystem, true),
            new SpinFlywheel(m_flywheelSubsystem),
            new RunTransferWheel(true, m_towerSubsystem).withTimeout(0.5)
        ));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
