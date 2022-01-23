package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
    
    private Intake m_intake;
    private XboxController m_driver;

    public RunIntake(Intake intake, XboxController driver) {
        m_intake = intake;
        m_driver = driver;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double speed = (m_driver.getRawAxis(3) - m_driver.getRawAxis(2)) * IntakeConstants.maxSpeed;
        m_intake.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.setSpeed(0);
    }
}
