package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeJoystick extends CommandBase {
    
    private final IntakeSubsystem m_intakeSubsystem;
    private final XboxController m_driver;

    public RunIntakeJoystick(IntakeSubsystem intakeSubsystem, XboxController driver) {
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_intakeSubsystem);
        
        m_driver = driver;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double speed = m_driver.getRightTriggerAxis() - m_driver.getLeftTriggerAxis() * IntakeConstants.reverseScalar;
        m_intakeSubsystem.setSpeed(speed * IntakeConstants.intakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.setSpeed(0);
    }
}
