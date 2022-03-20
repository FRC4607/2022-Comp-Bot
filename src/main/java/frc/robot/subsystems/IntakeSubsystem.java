package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax m_intakeMotor;
    private SparkMaxPIDController m_PIDController;
    private Solenoid m_solenoid;

    public enum IntakeState {
        Idle,
        Intaking,
        Releasing
    }

    private IntakeState m_state = IntakeState.Idle;

    public IntakeSubsystem() {
        m_solenoid = new Solenoid(Constants.pnumaticHub, PneumaticsModuleType.REVPH, IntakeConstants.solenoidChannel);
        m_solenoid.set(false);

        m_intakeMotor = new CANSparkMax(IntakeConstants.motorID, MotorType.kBrushless);
        m_intakeMotor.restoreFactoryDefaults();
        m_intakeMotor.setInverted(true);
        m_intakeMotor.setSmartCurrentLimit(40, 20);

        m_PIDController = m_intakeMotor.getPIDController();

        m_PIDController.setP(IntakeConstants.kP);
        m_PIDController.setI(IntakeConstants.kI);
        m_PIDController.setD(IntakeConstants.kD);

        m_PIDController.setOutputRange(-10, 10);
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
    public void setState(IntakeState state) {
        if (state != m_state) {
            switch (state) {
                case Idle:
                    m_intakeMotor.set(0);
                        m_solenoid.set(false);
                    break;
                case Intaking:
                    m_PIDController.setReference(IntakeConstants.intakeingRPM, ControlType.kVelocity, 0,
                            IntakeConstants.kS + IntakeConstants.kV * IntakeConstants.intakeingRPM / 60);
                    m_solenoid.set(true);
                    break;
                case Releasing:
                    m_PIDController.setReference(-IntakeConstants.releasingRPM, ControlType.kVelocity, 0,
                            -IntakeConstants.kS - IntakeConstants.kV * IntakeConstants.releasingRPM / 60);
                    m_solenoid.set(true);
                    break;
            }
            m_state = state;
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

    public IntakeState getState() {
        return m_state;
    }

    public void togglePiston() {
        m_solenoid.toggle();
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
