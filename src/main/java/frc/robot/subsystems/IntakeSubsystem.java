package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax m_intakeMotor;
    private Solenoid m_solenoid;

    public IntakeSubsystem() {
        m_solenoid = new Solenoid(Constants.pnumaticHub, PneumaticsModuleType.REVPH, IntakeConstants.solenoidChannel);
        m_solenoid.set(false);

        m_intakeMotor = new CANSparkMax(IntakeConstants.motorID, MotorType.kBrushless);
        m_intakeMotor.restoreFactoryDefaults();
        m_intakeMotor.setInverted(true);
        m_intakeMotor.setSmartCurrentLimit(20, 40);

    }

    /**
     * Sets the speed of the motor
     * 
     * @param speed the speed the motor is set to. [-1, 1]
     */
    public void setSpeed(double speed) {
        if (m_solenoid.get()) {
            m_intakeMotor.set(speed);
        } else {
            m_intakeMotor.set(0);
        }
    }

    /**
     * Sets the voltage of the motor
     * 
     * @param voltage
     */
    public void setVoltage(double voltage) {
        if (m_solenoid.get()) {
            m_intakeMotor.setVoltage(voltage);
        } else {
            m_intakeMotor.setVoltage(0);
        }
    }

    /**
     * Extends the solenoid
     */
    public void extendIntake() {
        m_solenoid.set(true);
    }

    /**
     * Retracts the solenoid
     */
    public void retractIntake() {
        m_solenoid.set(false);
    }

    /**
     * Toggles the solenoid
     */
    public void toggleIntake() {
        m_solenoid.toggle();
    }
}
