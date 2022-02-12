package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Paths;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferWheelSubsystem;

public class Auton_TwoBall_B extends CommandBase {
    private static CommandScheduler m_commandScheduler;

    private static DrivetrainSubsystem m_drivetrainSubsystem;
    private static IntakeSubsystem m_intakeSubsystem;
    private static TowerSubsystem m_towerSubsystem;
    private static TransferWheelSubsystem m_transferWheelSubsystem;
    private static FlywheelSubsystem m_flywheelSubsystem;

    public Auton_TwoBall_B(DrivetrainSubsystem drivetrainSubsystem, IntakeSubsystem intakeSubsystem, TowerSubsystem towerSubsystem, TransferWheelSubsystem transferWheelSubsystem, FlywheelSubsystem flywheelSubsystem) {
        m_commandScheduler = CommandScheduler.getInstance();

        m_drivetrainSubsystem = drivetrainSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        m_towerSubsystem = towerSubsystem;
        m_transferWheelSubsystem = transferWheelSubsystem;
        m_flywheelSubsystem = flywheelSubsystem;
    }

    @Override
    public void initialize() {
        /*m_commandScheduler.schedule(new SequentialCommandGroup(
            new SetIntake(m_intakeSubsystem, true),
            new SpinFlywheel(m_flywheelSubsystem),
            new RunTransferWheel(true, m_towerSubsystem).withTimeout(0.5),
        ));*/
        m_commandScheduler.schedule(new SequentialCommandGroup(
            new SetIntake(m_intakeSubsystem, false),
            new ParallelDeadlineGroup(
                new FollowPath(m_drivetrainSubsystem, Paths.twoBall0),
                new RunIntake(m_intakeSubsystem, false)
            ),
            new ParallelCommandGroup(
                new RunIntake(m_intakeSubsystem, false).withTimeout(0.1)
            ),
            new ParallelCommandGroup(
                new FollowPath(m_drivetrainSubsystem, Paths.twoBall1_B),
                new SpinFlywheel(m_flywheelSubsystem)
            ),
            new RunTransferWheel(m_transferWheelSubsystem, false).withTimeout(0.2),
            new ParallelCommandGroup(
                new SpinFlywheel(m_flywheelSubsystem),
                new RunIntake(m_intakeSubsystem, false).withTimeout(1)
            ),
            new RunTransferWheel(m_transferWheelSubsystem, false).withTimeout(0.2),
            new RunFlywheel(m_flywheelSubsystem).withTimeout(0.1)
        ).withTimeout(15), new AutoTower(m_towerSubsystem).withTimeout(15));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
