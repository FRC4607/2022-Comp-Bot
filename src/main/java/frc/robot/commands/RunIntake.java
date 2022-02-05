package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntake extends CommandBase {
    
    private IntakeSubsystem m_intakeSubsystem;
    private XboxController m_driver;

    public RunIntake(IntakeSubsystem intakeSubsystem, XboxController driver) {
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_intakeSubsystem);
        m_driver = driver;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double speed = m_driver.getRawAxis(XboxController.Axis.kRightTrigger.value) - m_driver.getRawAxis(XboxController.Axis.kLeftTrigger.value) * IntakeConstants.reverseScalar;
        m_intakeSubsystem.setSpeed(speed * IntakeConstants.intakeSpeed);
        m_intakeSubsystem.setAgitator(speed * IntakeConstants.agitatorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.setSpeed(0);
        m_intakeSubsystem.setAgitator(0);
    }
}
