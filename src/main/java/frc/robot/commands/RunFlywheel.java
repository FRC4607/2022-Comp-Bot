package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
        m_flywheelSubsystem.setSpeed(SmartDashboard.getNumber("Max flywheel speed", 0));
    }

    @Override
    public void end(boolean interuptied) {
        m_flywheelSubsystem.setSpeed(0);
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