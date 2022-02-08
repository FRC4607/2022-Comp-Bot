package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Paths;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferWheelSubsystem;

public class Auton_FourBall extends CommandBase {
    private static CommandScheduler m_commandScheduler;

    private static FlywheelSubsystem m_flywheelSubsystem;
    private static TransferWheelSubsystem m_towerSubsystem;
    private static IntakeSubsystem m_intakeSubsystem;
    private static DrivetrainSubsystem m_drivetrainSubsystem;
    private static AgitatorSubsystem m_agitatorSubsystem;

    public Auton_FourBall(FlywheelSubsystem flywheelSubsystem, TransferWheelSubsystem towerSubsystem, IntakeSubsystem intakeSubsystem, DrivetrainSubsystem drivetrainSubsystem, AgitatorSubsystem agitatorSubsystem) {
        m_commandScheduler = CommandScheduler.getInstance();

        m_flywheelSubsystem = flywheelSubsystem;
        m_towerSubsystem = towerSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_agitatorSubsystem = agitatorSubsystem;
    }

    @Override
    public void initialize() {
        /*m_commandScheduler.schedule(new SequentialCommandGroup(
            new SetIntake(m_intakeSubsystem, true),
            new SpinFlywheel(m_flywheelSubsystem),
            new RunTransferWheel(true, m_towerSubsystem).withTimeout(0.5),
        ));*/
        m_commandScheduler.schedule(new SequentialCommandGroup(
            // Extend the intake
            new SetIntake(m_intakeSubsystem, false),
            // Go to ball 2 and intake
            new ParallelDeadlineGroup(
                new FollowPath(m_drivetrainSubsystem, Paths.twoBall0),
                new RunIntake(m_intakeSubsystem, false)
            ),
            // Move ball 2 further in
            new ParallelCommandGroup(
                new RunIntake(m_intakeSubsystem, false).withTimeout(0.1),
                new RunAgitator(m_agitatorSubsystem, false).withTimeout(0.1)
            ),
            // Go to hub and spin flywheel to speed
            new ParallelCommandGroup(
                new FollowPath(m_drivetrainSubsystem, Paths.twoBall1_A),
                new SpinFlywheel(m_flywheelSubsystem)
            ),
            // Shoot ball 1
            new RunTransferWheel(true, m_towerSubsystem).withTimeout(0.2),
            // Spin flywheel and move ball 2 into positon
            new ParallelCommandGroup(
                new SpinFlywheel(m_flywheelSubsystem),
                new RunAgitator(m_agitatorSubsystem, false).withTimeout(1)    
            ),
            // Shoot ball 2
            new RunTransferWheel(true, m_towerSubsystem).withTimeout(0.2),
            // Go to ball 3, and ball 4, stop the flywheel, Intake ball 3, and move ball 3 into position
            new ParallelDeadlineGroup(
                new FollowPath(m_drivetrainSubsystem, Paths.fourBall2),
                new RunFlywheel(m_flywheelSubsystem).withTimeout(0),
                new RunIntake(m_intakeSubsystem, false),
                new RunAgitator(m_agitatorSubsystem, false)
            ),
            // Return to hub, Get flywheel up to speed, Run intake and aditator to get ball 4 into position
            new ParallelCommandGroup(
                new FollowPath(m_drivetrainSubsystem, Paths.fourBall3),
                new SpinFlywheel(m_flywheelSubsystem),
                new RunIntake(m_intakeSubsystem, false).withTimeout(0.1),
                new RunAgitator(m_agitatorSubsystem, false).withTimeout(0.1)
            ),
            // Shoot ball 3
            new RunTransferWheel(true, m_towerSubsystem).withTimeout(0.2),
            new ParallelCommandGroup(
                new SpinFlywheel(m_flywheelSubsystem),
                new RunAgitator(m_agitatorSubsystem, false).withTimeout(1)    
            ),
            // Shoot ball 4
            new RunTransferWheel(true, m_towerSubsystem).withTimeout(0.2),
            new RunFlywheel(m_flywheelSubsystem).withTimeout(0),
            new FollowPath(m_drivetrainSubsystem, Paths.exit)
            ).withTimeout(15));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
