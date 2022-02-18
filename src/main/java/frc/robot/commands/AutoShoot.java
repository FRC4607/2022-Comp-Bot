package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.TransferWheelSubsystem;

public class AutoShoot extends CommandBase {
    private FlywheelSubsystem m_flywheelSubsystem;
    private TransferWheelSubsystem m_transferWheelSubsystem;

    public AutoShoot() {
        
    }
}
