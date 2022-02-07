package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AgitatorConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class AgitatorSubsystem extends SubsystemBase {

    private CANSparkMax m_agitator;

    public AgitatorSubsystem() {

        m_agitator = new CANSparkMax(AgitatorConstants.agitatorID, MotorType.kBrushless);
        m_agitator.restoreFactoryDefaults();
        m_agitator.setInverted(false);
        m_agitator.setIdleMode(IdleMode.kBrake);
    }
   

    public void setSpeed(double speed) {
        m_agitator.set(speed);
    }

    /**
     * Sets the voltage of the motor
     * @param voltage
     */
    public void setVoltage(double voltage) {
        m_agitator.setVoltage(voltage);
    }
}
