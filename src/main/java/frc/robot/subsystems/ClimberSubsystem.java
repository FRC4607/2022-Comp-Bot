package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private CANSparkMax m_motor1;
    private CANSparkMax m_motor2;
    private RelativeEncoder m_encoder;

    private SparkMaxPIDController m_PIDController;

    private DoubleSolenoid m_pistion;
    private DoubleSolenoid m_clutch;

    private DigitalInput m_limitSwitch;

    public ClimberState climberState;
    public ClutchState clutchState;
    public PistonState pistonState;

    public enum ClimberState {
        Retracted,
        Retracting,
        Extended,
        Extending
    }

    public enum ClutchState {
        Engaged,
        Disengaged
    }

    public enum PistonState {
        Extended,
        Retracted
    }

    public ClimberSubsystem() {
        m_motor1 = new CANSparkMax(ClimberConstants.motor1ID, MotorType.kBrushless);
        m_motor2 = new CANSparkMax(ClimberConstants.motor2ID, MotorType.kBrushless);

        m_motor2.follow(m_motor1);
        m_encoder = m_motor1.getEncoder();
        m_encoder.setPositionConversionFactor(ClimberConstants.conversenFactor_SensorUnitsPerInch);

        m_PIDController = m_motor1.getPIDController();
        m_PIDController.setFeedbackDevice(m_motor1.getEncoder());
        m_PIDController.setP(ClimberConstants.kP);
        m_PIDController.setI(ClimberConstants.kI);
        m_PIDController.setD(ClimberConstants.kD);
        m_PIDController.setFF(ClimberConstants.kFF);

        m_pistion = new DoubleSolenoid(ClimberConstants.pistionModule, ClimberConstants.pistionType,
                ClimberConstants.pistionForwardChannel, ClimberConstants.pistionReverseChannel);

        m_clutch = new DoubleSolenoid(ClimberConstants.clutchModule, ClimberConstants.clutchType,
                ClimberConstants.clutchForwardChannel, ClimberConstants.clutchReverseChannel);

        m_pistion.set(Value.kForward);
        m_clutch.set(Value.kReverse);

        pistonState = PistonState.Extended;
        clutchState = ClutchState.Engaged;

        m_limitSwitch = new DigitalInput(ClimberConstants.limitSwitchID);

    }

    @Override
    public void periodic() {
        switch (climberState) {
            case Retracting:
                m_PIDController.setReference(0, ControlType.kPosition);
                if (m_limitSwitch.get()) {
                    climberState = ClimberState.Retracted;
                    m_encoder.setPosition(0);
                }
                break;
            case Extending:
                m_PIDController.setReference(ClimberConstants.maxHight, ControlType.kPosition);
                if (m_encoder.getPosition() >= ClimberConstants.maxHight) {
                    climberState = ClimberState.Extended;
                }
                break;
            default:
                break;
        }
    }

    public void extendClimber(boolean extended) {
        climberState = extended ? ClimberState.Extending : ClimberState.Retracting;
    }

    public void setClutch(boolean engaged) {
        m_clutch.set(engaged ? Value.kReverse : Value.kForward);
        clutchState = engaged ? ClutchState.Engaged : ClutchState.Disengaged;
    }

    public void setPistion(boolean extended) {
        m_pistion.set(extended ? Value.kForward : Value.kReverse);
        pistonState = extended ? PistonState.Extended : PistonState.Retracted;
    }

    public boolean atPosition() {
        return climberState == ClimberState.Retracted || climberState == ClimberState.Extended;
    }

    public double getEncoder() {
        return m_encoder.getPosition();
    }
}