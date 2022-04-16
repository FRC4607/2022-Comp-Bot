package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer.LimeLightTargetState;
import frc.robot.subsystems.*;
import oi.limelightvision.limelight.frc.LimeLight;

public class LimeLightTarget extends CommandBase {
    private LimeLight m_limeLight;
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    private final XboxController m_driver;
    private final XboxController m_operator;
    private ProfiledPIDController pidController;
    private boolean m_alingined;
    private boolean m_finish;

    public LimeLightTarget(LimeLight limeLight, DrivetrainSubsystem drivetrainSubsystem, ShooterSubsystem shooterSubsystem, XboxController driver, XboxController operator) {
        m_limeLight = limeLight;
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_drivetrainSubsystem);

        m_driver = driver;
        m_operator = operator;
        m_finish = false;
        pidController = new ProfiledPIDController(0.25, 0, 0, new TrapezoidProfile.Constraints(10, 10));
    }

    public LimeLightTarget(LimeLight limeLight, DrivetrainSubsystem drivetrainSubsystem, ShooterSubsystem shooterSubsystem) {
        m_limeLight = limeLight;
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_drivetrainSubsystem);

        m_driver = null;
        m_operator = null;
        m_finish = true;
        pidController = new ProfiledPIDController(0.25, 0, 0, new TrapezoidProfile.Constraints(10, 10));
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_limeLight.setPipeline(1);
        pidController.setGoal(0);
        m_alingined = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double yAxis;
        if (m_driver != null) {
            yAxis = m_driver.getLeftY();
        }
        else {
            yAxis = 0;
        }
        double PIDVolatge = 0.0;
        double driverInput = 0.0;
        if (m_limeLight.getIsTargetFound()) {
            double angle = m_limeLight.getdegRotationToTarget();
            double PID = pidController.calculate(angle);
            
            if (Math.abs(angle) > 0.1) {
                PIDVolatge = PID + Math.copySign(DriveConstants.ks_Volts, PID);
            }
            if (yAxis > 0.05 || yAxis < -0.05) { 
                driverInput = yAxis * -1.0;
            }
            m_drivetrainSubsystem.tankDriveVolts(-PIDVolatge + driverInput, PIDVolatge + driverInput);
            if (Math.abs(PID) < 0.4) {
                RobotContainer.getInstance().m_lightTargetState = LimeLightTargetState.Ready;
                setRumble(0.2);
            }
            else {
                RobotContainer.getInstance().m_lightTargetState = LimeLightTargetState.Targeting;
                setRumble(0);
            }
            if (Math.abs(PID) < 0.2) {
                double dV_in = 104 - 25; // Meshered
                double angle_deg = 31.5 + m_limeLight.getdegVerticalToTarget(); // Meshered
                double dD_in = dV_in / Math.tan(Math.toRadians(angle_deg));
                
                SmartDashboard.putNumber("Limelight Distance", dD_in);

                double RPM = -0.052043 * Math.pow(dD_in, 2) + 34.3271 * dD_in + 217.264;
                m_shooterSubsystem.setLimeLightRPM(RPM);
                m_alingined = true;
            } else {
                m_alingined = false;
            }
        } else {
            if (m_driver != null) {
                m_drivetrainSubsystem.setArcadeDrive(m_driver.getLeftX(), m_driver.getLeftY());
            }
            RobotContainer.getInstance().m_lightTargetState = LimeLightTargetState.NoTarget;
        }
    }

    private void setRumble(double value) {
        if (m_driver != null && m_operator != null) {
            m_driver.setRumble(RumbleType.kLeftRumble, value);
            m_driver.setRumble(RumbleType.kRightRumble, value);
            m_operator.setRumble(RumbleType.kLeftRumble, value);
            m_operator.setRumble(RumbleType.kRightRumble, value);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_limeLight.setPipeline(0);
        m_drivetrainSubsystem.setArcadeDrive(0, 0);
        RobotContainer.getInstance().m_lightTargetState = LimeLightTargetState.Idle;

        setRumble(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_alingined && m_finish;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
