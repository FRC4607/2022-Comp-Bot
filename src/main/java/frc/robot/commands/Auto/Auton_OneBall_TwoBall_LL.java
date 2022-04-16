package frc.robot.commands.Auto;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Paths;
import frc.robot.commands.LimeLightTarget;
import frc.robot.commands.ShootBalls;
import frc.robot.commands.ToggleShooterPiston;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ShooterSubsystem.ShootingMode;
import oi.limelightvision.limelight.frc.LimeLight;

public class Auton_OneBall_TwoBall_LL extends CommandBase {

    private static DrivetrainSubsystem m_drivetrainSubsystem;
    private static IntakeSubsystem m_intakeSubsystem;
    private static TowerSubsystem m_towerSubsystem;
    private static ShooterSubsystem m_shooterSubsystem;
    private static LimeLight m_limeLight;

    private final int m_numberOfBalls;

    private Command m_sequence;
    private Timer m_timer;

    public Auton_OneBall_TwoBall_LL(DrivetrainSubsystem drivetrainSubsystem, IntakeSubsystem intakeSubsystem,
            TowerSubsystem towerSubsystem, ShooterSubsystem shooterSubsystem, LimeLight limeLight, int numberOfBalls) {

        m_drivetrainSubsystem = drivetrainSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        m_towerSubsystem = towerSubsystem;
        m_shooterSubsystem = shooterSubsystem;
        m_limeLight = limeLight;
        m_numberOfBalls = numberOfBalls;

        m_timer = new Timer();
    }

    @Override
    public void initialize() {

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable FMSInfo = inst.getTable("FMSInfo");
        NetworkTableEntry alienceColor = FMSInfo.getEntry("IsRedAlliance");
        boolean m_isRed = alienceColor.getBoolean(true);

        m_drivetrainSubsystem.setBrakeMode(true);
        m_intakeSubsystem.setIntake(true);
        m_shooterSubsystem.setShootingMode(ShootingMode.limeLight);
        m_timer.reset();
        m_timer.start();

        m_sequence = new SequentialCommandGroup(
                new ToggleShooterPiston(m_shooterSubsystem),
                new ParallelDeadlineGroup(
                        new FollowPath(m_drivetrainSubsystem,
                                m_isRed ? Paths.redPaths.Start_Ball2 : Paths.bluePaths.Start_Ball2),
                        new AutoIntake(m_intakeSubsystem, m_towerSubsystem), new RunAutoTower(m_towerSubsystem)),
                new LimeLightTarget(m_limeLight, m_drivetrainSubsystem, m_shooterSubsystem),
                new ShootBalls(m_towerSubsystem, m_shooterSubsystem, m_intakeSubsystem, m_numberOfBalls),
                new InstantCommand(() -> {
                    m_drivetrainSubsystem.setBrakeMode(false);
                }));
        m_sequence.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        m_sequence.cancel();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(15);
        // return false;
    }
}
