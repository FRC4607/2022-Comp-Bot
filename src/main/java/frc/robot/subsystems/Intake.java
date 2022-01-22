package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

class Intake extends SubsystemBase {

    private WPI_TalonFX m_motor;
    private DoubleSolenoid m_leftSolenoid;
    private DoubleSolenoid m_rightSolenoid;

    public Intake() {
        m_leftSolenoid = new DoubleSolenoid(IntakeConstants.leftSolenoidModule, IntakeConstants.SolenoidType,
                IntakeConstants.leftSolenoidForwardChannel, IntakeConstants.leftSolenoidReverseChannel);
        m_rightSolenoid = new DoubleSolenoid(IntakeConstants.rightSolenoidModule, IntakeConstants.SolenoidType,
                IntakeConstants.rightSolenoidForwardChannel, IntakeConstants.rightSolenoidReverseChannel);

        m_motor = new WPI_TalonFX(IntakeConstants.motorID);
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
        m_leftSolenoid.set(Value.kForward);
        m_rightSolenoid.set(Value.kForward);
    }

    /**
     * Retracts the solenoid
     */
    public void retractIntake() {
        m_leftSolenoid.set(Value.kReverse);
        m_rightSolenoid.set(Value.kReverse);
    }

    /**
     * Toggles the solenoid
     */
    public void toggleIntake() {
        m_leftSolenoid.toggle();
        m_rightSolenoid.toggle();
    }
}
