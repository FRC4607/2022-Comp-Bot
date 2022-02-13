// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TowerConstants;
import frc.robot.subsystems.TransferWheelSubsystem;

/**
 *
 */
public class RunTransferWheel extends CommandBase {

    private final TransferWheelSubsystem m_towerSubsystem;
    private boolean m_reverse;

    public RunTransferWheel(TransferWheelSubsystem subsystem, boolean reverse) {
        m_towerSubsystem = subsystem;
        addRequirements(m_towerSubsystem);
        
        m_reverse = reverse;

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (!m_reverse) {
            m_towerSubsystem.setTransferWheel(TowerConstants.agitatiorSpeed);
        }
        else {
            m_towerSubsystem.setTransferWheel(-TowerConstants.agitatiorSpeed);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_towerSubsystem.setTransferWheel(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
