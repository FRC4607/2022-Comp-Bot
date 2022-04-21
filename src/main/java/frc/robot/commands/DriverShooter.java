package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class DriverShooter extends CommandBase {

    private final ShooterSubsystem m_shooterSubsystem;
    private final XboxController m_driver;

    public DriverShooter(ShooterSubsystem shooterSubsystem, XboxController driver) {
        m_shooterSubsystem = shooterSubsystem;
        m_driver = driver;

        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void execute() {
        double speed = m_driver.getLeftTriggerAxis();
        if (speed > 0.1) {
            m_shooterSubsystem.setKickerWheel(-0.5 * speed);
        } else {
            m_shooterSubsystem.setKickerWheel(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.setKickerWheel(0);
    }
}
