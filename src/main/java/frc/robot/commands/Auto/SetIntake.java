package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntake extends CommandBase {
    
    private IntakeSubsystem m_intakeSubsytem;
    private boolean m_extended;

    public SetIntake(IntakeSubsystem intakeSubsystem, boolean extended) {
        m_intakeSubsytem = intakeSubsystem;
        m_extended = extended;
    }

    @Override
    public void initialize() {
        if (m_extended) {
            m_intakeSubsytem.extendIntake();
        } else {
            m_intakeSubsytem.retractIntake();
        }
    }

     @Override
     public boolean isFinished() {
         return true;
     }
}
