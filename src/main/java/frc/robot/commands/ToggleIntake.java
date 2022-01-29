package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ToggleIntake extends CommandBase {
    
    private IntakeSubsystem m_intakeSubsytem;

    public ToggleIntake(IntakeSubsystem intakeSubsystem) {
        m_intakeSubsytem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        m_intakeSubsytem.toggleIntake();
    }

     @Override
     public boolean isFinished() {
         return true;
     }
}
