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
    private boolean forcedExtention;

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

    public void setState(IntakeState state) {
        if (state != m_state) {
            switch (state) {
                case Idle:
                    m_intakeMotor.set(0);
                    if (!forcedExtention)
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

    public IntakeState getState() {
        return m_state;
    }

    public void togglePiston() {
        m_solenoid.toggle();
    }

    public void setForcedExtention(boolean exetended) {
        forcedExtention = exetended;
        if (exetended) {
            m_solenoid.set(true);
        } else if (m_state == IntakeState.Idle) {
            m_solenoid.set(false);
        }
        SmartDashboard.putBoolean("Forced Extenchon", forcedExtention);
    }
}
