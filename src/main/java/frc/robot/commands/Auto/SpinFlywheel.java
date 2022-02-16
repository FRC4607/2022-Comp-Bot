package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.subsystems.FlywheelSubsystem;

public class SpinFlywheel extends CommandBase {
    private final FlywheelSubsystem m_flywheelSubsystem;

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
        double calculatedError = Math.abs(m_flywheelSubsystem.getFlywheelError() - 30);
        SmartDashboard.putNumber("Calculated Error", calculatedError);
        return m_flywheelSubsystem.constantSpeed();
    }  
}
