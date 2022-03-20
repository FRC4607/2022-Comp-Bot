package frc.robot.commands.Auto;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Paths;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.ShootBalls;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TowerSubsystem;

public class Auton_FourBall extends CommandBase {
    private static CommandScheduler m_commandScheduler;

    private static ShooterSubsystem m_shooterSubsystem;
    private static IntakeSubsystem m_intakeSubsystem;
    private static DrivetrainSubsystem m_drivetrainSubsystem;
    private static TowerSubsystem m_towerSubsystem;

    public Auton_FourBall(ShooterSubsystem shooterSubsystem,
            IntakeSubsystem intakeSubsystem, DrivetrainSubsystem drivetrainSubsystem, TowerSubsystem towerSubsystem) {
        m_commandScheduler = CommandScheduler.getInstance();

        m_shooterSubsystem = shooterSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_towerSubsystem = towerSubsystem;
    }

    @Override
    public void initialize() {
        
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable FMSInfo = inst.getTable("FMSInfo");
        NetworkTableEntry alienceColor = FMSInfo.getEntry("IsRedAlliance");
        boolean m_isRed = alienceColor.getBoolean(true);
        SmartDashboard.putBoolean("Is Red Allience", m_isRed);
        
        m_commandScheduler.schedule(new SequentialCommandGroup(
                new InstantCommand(() -> {
                    m_drivetrainSubsystem.setBrakeMode(true);
                }),
                new ParallelDeadlineGroup(
                        new FollowPath(m_drivetrainSubsystem, m_isRed ? Paths.redPaths.Start_Ball2 : Paths.bluePaths.Start_Ball2),
                        new IntakeBalls(m_intakeSubsystem, m_towerSubsystem)),
                new ParallelDeadlineGroup(
                        new FollowPath(m_drivetrainSubsystem, m_isRed ? Paths.redPaths.Ball2_Hub : Paths.bluePaths.Ball2_Hub),
                        new IntakeBalls(m_intakeSubsystem, m_towerSubsystem).withTimeout(1).andThen(new RunAutoTower(m_towerSubsystem))),
                new ShootBalls(m_towerSubsystem, m_shooterSubsystem, m_intakeSubsystem, 2),
                new ParallelDeadlineGroup(
                    new FollowPath(m_drivetrainSubsystem, m_isRed ? Paths.redPaths.Hub_Ball3_Ball4 : Paths.bluePaths.Hub_Ball3_Ball4),
                    new IntakeBalls(m_intakeSubsystem, m_towerSubsystem)),
                new ParallelDeadlineGroup(
                        new FollowPath(m_drivetrainSubsystem, m_isRed ? Paths.redPaths.Ball4_Hub : Paths.bluePaths.Ball4_Hub),
                        new IntakeBalls(m_intakeSubsystem, m_towerSubsystem).withTimeout(1).andThen(new RunAutoTower(m_towerSubsystem))),
                new ShootBalls(m_towerSubsystem, m_shooterSubsystem, m_intakeSubsystem, 2),
                new InstantCommand(() -> {
                    m_shooterSubsystem.setSpeed(0);
                }, m_shooterSubsystem),
                new InstantCommand(() -> {
                    m_drivetrainSubsystem.setBrakeMode(false);
                })));
            
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}


