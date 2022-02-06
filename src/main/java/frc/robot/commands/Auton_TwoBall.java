package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Paths;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class Auton_TwoBall extends CommandBase {
    private static CommandScheduler m_commandScheduler;

    private static FlywheelSubsystem m_flywheelSubsystem;
    private static TowerSubsystem m_towerSubsystem;
    private static IntakeSubsystem m_intakeSubsystem;
    private static DrivetrainSubsystem m_drivetrainSubsystem;

    public Auton_TwoBall(FlywheelSubsystem flywheelSubsystem, TowerSubsystem towerSubsystem, IntakeSubsystem intakeSubsystem, DrivetrainSubsystem drivetrainSubsystem) {
        m_commandScheduler = CommandScheduler.getInstance();

        m_flywheelSubsystem = flywheelSubsystem;
        m_towerSubsystem = towerSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        m_drivetrainSubsystem = drivetrainSubsystem;
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
            new RunIntake(m_intakeSubsystem, false).withTimeout(0.3),
            new RunIntakeAndAgitator(m_intakeSubsystem, false).withTimeout(0.2),
            new ParallelCommandGroup(
                new FollowPath(m_drivetrainSubsystem, Paths.twoBall1),
                new SpinFlywheel(m_flywheelSubsystem)
            ),
            new ParallelCommandGroup(
                new RunTransferWheel(true, m_towerSubsystem).withTimeout(2),
                new RunAgitator(m_intakeSubsystem, false).withTimeout(2)    
            ),
            new RunFlywheel(m_flywheelSubsystem).withTimeout(0.1)
        ));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
