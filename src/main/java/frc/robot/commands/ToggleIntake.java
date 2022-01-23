package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class ToggleIntake extends CommandBase {
    
    private Intake m_intake;

    public ToggleIntake(Intake intake) {
        m_intake = intake;
    }

    @Override
    public void initialize() {
        m_intake.toggleIntake();
    }

     @Override
     public boolean isFinished() {
         return true;
     }
}
