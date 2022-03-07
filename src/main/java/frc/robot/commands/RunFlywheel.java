package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.TransferWheelConstants;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.TransferWheelSubsystem;

public class RunFlywheel extends CommandBase {

    private final FlywheelSubsystem m_flywheelSubsystem;
    private final TransferWheelSubsystem m_transferWheelSubsystem;

    public RunFlywheel(FlywheelSubsystem flywheelSubsystem, TransferWheelSubsystem transferWheelSubsystem) {
        m_flywheelSubsystem = flywheelSubsystem;
        m_transferWheelSubsystem = transferWheelSubsystem;
        addRequirements(m_flywheelSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_flywheelSubsystem.setRPM(FlywheelConstants.flywheeelRPM);
        // m_flywheelSubsystem.setRPM(SmartDashboard.getNumber("Flywheel Speed RPM", FlywheelConstants.flywheeelRPM));
        // if (m_flywheelSubsystem.constantSpeed()) {
        //     m_transferWheelSubsystem.setTransferWheel(TransferWheelConstants.transferWheelSpeed);
        // } else {
        //     m_transferWheelSubsystem.setTransferWheel(0);
        // }
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