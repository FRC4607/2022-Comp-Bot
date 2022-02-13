package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Not Functional
 */
public class AutoClimb extends CommandBase {
    private static CommandScheduler m_commandScheduler;

    private DrivetrainSubsystem m_drivetrainSubsystem;
    private ClimberSubsystem m_climberSubsystem;

    public AutoClimb(DrivetrainSubsystem drivetrainSubsystem, ClimberSubsystem climberSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_climberSubsystem = climberSubsystem;

        m_commandScheduler.schedule(new SequentialCommandGroup(
            new ExtendClimberPistion(m_climberSubsystem, true),
            new ExtendClimber(m_climberSubsystem).withInterrupt(() -> {
                    return (m_climberSubsystem.getEncoder() == ClimberConstants.maxHight)
                            && (m_drivetrainSubsystem.getYaw() > ClimberConstants.rotationToExtend);
                }))
        );

    }

}