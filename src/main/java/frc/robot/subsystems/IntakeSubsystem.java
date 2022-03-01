package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax m_intakeMotor;
    private DoubleSolenoid m_solenoid;

    public IntakeSubsystem() {
        m_solenoid = new DoubleSolenoid(IntakeConstants.solenoidModule, IntakeConstants.SolenoidType,
                IntakeConstants.solenoidForwardChannel, IntakeConstants.solenoidReverseChannel);
        m_solenoid.set(Value.kForward);

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
        if (m_solenoid.get() == Value.kReverse) {
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
        if (m_solenoid.get() == Value.kReverse) {
            m_intakeMotor.setVoltage(voltage);
        } else {
            m_intakeMotor.setVoltage(0);
        }
    }

    /**
     * Extends the solenoid
     */
    public void extendIntake() {
        m_solenoid.set(Value.kReverse);
    }

    /**
     * Retracts the solenoid
     */
    public void retractIntake() {
        m_solenoid.set(Value.kForward);
    }

    /**
     * Toggles the solenoid
     */
    public void toggleIntake() {
        m_solenoid.toggle();
    }
}
