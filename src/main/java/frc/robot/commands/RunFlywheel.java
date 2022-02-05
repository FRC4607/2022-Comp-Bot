package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.subsystems.FlywheelSubsystem;

public class RunFlywheel extends CommandBase {

    private FlywheelSubsystem m_flywheelSubsystem;

    public RunFlywheel(FlywheelSubsystem flywheelSubsystem) {
        m_flywheelSubsystem = flywheelSubsystem;
        addRequirements(m_flywheelSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_flywheelSubsystem.setRPM(FlywheelConstants.flywheeelRPM);
    }

    @Override
    public void end(boolean interuptied) {
        m_flywheelSubsystem.setRPM(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}