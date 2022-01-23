package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase {

    private CANSparkMax m_motor;
    private DoubleSolenoid m_solenoid;

    public Intake() {
        m_solenoid = new DoubleSolenoid(IntakeConstants.solenoidModule, IntakeConstants.SolenoidType,
                IntakeConstants.solenoidForwardChannel, IntakeConstants.solenoidReverseChannel);
        m_motor = new CANSparkMax(IntakeConstants.motorID, MotorType.kBrushless);
        m_motor.setSmartCurrentLimit(20, 40);
        
    }

    /**
     * Sets the speed of the motor
     * 
     * @param speed the speed the motor is set to. [-1, 1]
     */
    public void setSpeed(double speed) {
        m_motor.set(speed);
    }

    /**
     * Sets the voltage of the motor
     * @param voltage
     */
    public void setVoltage(double voltage) {
        m_motor.setVoltage(voltage);
    }

    /**
     * Extends the solenoid
     */
    public void extendIntake() {
        m_solenoid.set(Value.kForward);
    }

    /**
     * Retracts the solenoid
     */
    public void retractIntake() {
        m_solenoid.set(Value.kReverse);
    }

    /**
     * Toggles the solenoid
     */
    public void toggleIntake() {
        m_solenoid.toggle();
    }
}
