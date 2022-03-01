package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Paths;
import frc.robot.commands.RunTransferWheel;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.TransferWheelSubsystem;

public class Auton_FourBall extends CommandBase {
    private static CommandScheduler m_commandScheduler;

    private static FlywheelSubsystem m_flywheelSubsystem;
    private static TransferWheelSubsystem m_transferWheelSubsystem;
    private static IntakeSubsystem m_intakeSubsystem;
    private static DrivetrainSubsystem m_drivetrainSubsystem;
    private static TowerSubsystem m_towerSubsystem;

    public Auton_FourBall(FlywheelSubsystem flywheelSubsystem, TransferWheelSubsystem transferWheelSubsystem,
            IntakeSubsystem intakeSubsystem, DrivetrainSubsystem drivetrainSubsystem, TowerSubsystem towerSubsystem) {
        m_commandScheduler = CommandScheduler.getInstance();

        m_flywheelSubsystem = flywheelSubsystem;
        m_transferWheelSubsystem = transferWheelSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_towerSubsystem = towerSubsystem;
    }

    @Override
    public void initialize() {
        /*
         * m_commandScheduler.schedule(new SequentialCommandGroup(
         * new SetIntake(m_intakeSubsystem, true),
         * new SpinFlywheel(m_flywheelSubsystem),
         * new RunTransferWheel(true, m_towerSubsystem).withTimeout(0.5),
         * ));
         */
        m_commandScheduler.schedule(new SequentialCommandGroup(
                // Extend the intake
                new SetIntake(m_intakeSubsystem, true),
                new InstantCommand(() -> {
                    m_drivetrainSubsystem.setBrakeMode(true);
                }),
                // Go to ball 2 and intake
                new ParallelDeadlineGroup(
                        new FollowPath(m_drivetrainSubsystem, Paths.Start_Ball2),
                        new RunIntake(m_intakeSubsystem, false)),
                // Go to hub and spin flywheel to speed
                new ParallelCommandGroup(
                        new FollowPath(m_drivetrainSubsystem, Paths.Ball2_Hub),
                        new SpinFlywheel(m_flywheelSubsystem)),
                // Shoot ball 1
                new RunTransferWheel(m_transferWheelSubsystem, m_flywheelSubsystem, false).withTimeout(0.2),
                // Spin flywheel up to speed
                new SpinFlywheel(m_flywheelSubsystem),
                // Shoot ball 2
                new RunTransferWheel(m_transferWheelSubsystem, m_flywheelSubsystem, false).withTimeout(0.2),
                // Go to ball 3, and ball 4, stop the flywheel, Intake ball 3, and move ball 3
                // into position
                new ParallelDeadlineGroup(
                        new FollowPath(m_drivetrainSubsystem, Paths.Hub_Ball3_Ball4),
                        new InstantCommand(() -> {
                            m_flywheelSubsystem.setSpeed(0);
                        }, m_flywheelSubsystem),
                        new RunIntake(m_intakeSubsystem, false)),
                // Return to hub, Get flywheel up to speed, Run intake and aditator to get ball
                // 4 into position
                new ParallelCommandGroup(
                        new FollowPath(m_drivetrainSubsystem, Paths.Ball4_Hub),
                        new SpinFlywheel(m_flywheelSubsystem).beforeStarting(new WaitCommand(2)),
                        new RunIntake(m_intakeSubsystem, false).withTimeout(1)),
                // Shoot ball 3
                new RunTransferWheel(m_transferWheelSubsystem, m_flywheelSubsystem, false).withTimeout(0.2),
                new SpinFlywheel(m_flywheelSubsystem),
                // Shoot ball 4
                new RunTransferWheel(m_transferWheelSubsystem, m_flywheelSubsystem, false).withTimeout(0.2),
                new InstantCommand(() -> {
                    m_flywheelSubsystem.setSpeed(0);
                }, m_flywheelSubsystem),
                new InstantCommand(() -> {
                    m_drivetrainSubsystem.setBrakeMode(false);
                })), new RunAutoTower(m_towerSubsystem));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
