package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.subsystems.FlywheelSubsystem;

public class SpinFlywheel extends CommandBase {
    private final FlywheelSubsystem m_flywheelSubsystem;
    private int m_counter = 0;

    public SpinFlywheel(FlywheelSubsystem flywheelSubsystem) {
        m_flywheelSubsystem = flywheelSubsystem;
        addRequirements(flywheelSubsystem );
    }

    @Override
    public void initialize() {
        m_flywheelSubsystem.setRPM(FlywheelConstants.flywheeelRPM);
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(m_flywheelSubsystem.getFlywheelError()) < FlywheelConstants.flywheelMaxError) {
            m_counter ++;
        } 
        else {
            m_counter = 0;
        }
        if (m_counter >= 10) {
            return true;
        }
        else {
            return false;
        }
    }  
}
