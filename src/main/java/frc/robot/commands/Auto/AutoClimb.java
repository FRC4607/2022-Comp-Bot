package frc.robot.commands.Auto;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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

        BooleanSupplier ValidRotation = () -> {return (ClimberConstants.minRoatation <= /*m_drivetrainSubsystem.getYaw()*/ 0 && /*m_drivetrainSubsystem.getYaw()*/0 <= ClimberConstants.maxRoatation); };
        m_commandScheduler.schedule(new SequentialCommandGroup(
            new ExtendClimberPistion(m_climberSubsystem, true),
            new WaitUntilCommand(ValidRotation),
            new SetClimber(m_climberSubsystem, true),
            new ExtendClimberPistion(m_climberSubsystem, false),
            new SetClimber(m_climberSubsystem, false),
            new ExtendClimberPistion(m_climberSubsystem, true),
            new WaitUntilCommand(ValidRotation),
            new SetClimber(m_climberSubsystem, true),
            new ExtendClimberPistion(m_climberSubsystem, false),
            new SetClimber(m_climberSubsystem, false),
            new ExtendClimberPistion(m_climberSubsystem, true),
            new WaitUntilCommand(ValidRotation),
            new SetClimber(m_climberSubsystem, true),
            new ExtendClimberPistion(m_climberSubsystem, false),
            new SetClimber(m_climberSubsystem, false)
        ));

    }

}