package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private CANSparkMax m_motor1;
    private CANSparkMax m_motor2;
    private Encoder m_encoder;
    private DutyCycleEncoder m_absolutEncoder;

    private SparkMaxPIDController m_PIDController;

    private DoubleSolenoid m_pistion;
    private DoubleSolenoid m_clutch;

    private DigitalInput m_limitSwitch;

    public ClimberState climberState = ClimberState.Retracted;
    public ClutchState clutchState = ClutchState.Engaged;
    public PistonState pistonState = PistonState.Extended;

    public LimitSwitchState limitSwitchState;

    private final SlewRateLimiter m_limiter;

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

    public enum LimitSwitchState {
        min,
        max,
        changing
    }

    public ClimberSubsystem() {
        m_motor1 = new CANSparkMax(ClimberConstants.motor1ID, MotorType.kBrushless);
        m_motor2 = new CANSparkMax(ClimberConstants.motor2ID, MotorType.kBrushless);

        m_motor2.follow(m_motor1);

        m_motor1.setIdleMode(IdleMode.kBrake);
        m_motor2.setIdleMode(IdleMode.kBrake);

        // m_encoder = new Encoder(3, 4, 5);
        m_absolutEncoder = new DutyCycleEncoder(2);

        // m_encoder = m_motor1.getEncoder();
        // m_encoder.setPositionConversionFactor(ClimberConstants.conversenFactor_SensorUnitsPerInch);

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

        m_limiter = new SlewRateLimiter(2);

        // m_limitSwitch = new DigitalInput(ClimberConstants.limitSwitchID);
        // limitSwitchState = LimitSwitchState.changing;
    }

    @Override
    public void periodic() {
        // switch (climberState) {
        // case Retracting:
        // m_PIDController.setReference(0, ControlType.kPosition);
        // if (m_limitSwitch.get()) {
        // climberState = ClimberState.Retracted;
        // m_encoder.reset();
        // }
        // break;
        // case Extending:
        // m_PIDController.setReference(ClimberConstants.maxHight_Rotations,
        // ControlType.kPosition);
        // if (m_encoder.getDistance() >= ClimberConstants.maxHight_Rotations) {
        // climberState = ClimberState.Extended;
        // }
        // break;
        // default:
        // break;
        // }
        // SmartDashboard.putNumber("Climber Curent", m_motor1.getOutputCurrent());

        // SmartDashboard.putNumber("Encoder Distance", m_encoder.getDistance());
        // SmartDashboard.putNumber("Absolut Encoder", m_absolutEncoder.get());
        SmartDashboard.putNumber("Absolut Encoder Distance", m_absolutEncoder.getDistance());

        if (m_absolutEncoder.getDistance() < 0) {
            m_absolutEncoder.reset();
        }
    }

    public void extendClimber(boolean extended) {
        climberState = extended ? ClimberState.Extending : ClimberState.Retracting;
    }

    public void setClimber(double speed) {
        if (speed > 0) {
            if (m_absolutEncoder.getDistance() >= ClimberConstants.maxHight_Rotations) {
                m_motor1.set(0);
                m_motor2.set(0);
            } else {
                m_motor1.set(m_limiter.calculate(speed));
                m_motor2.set(m_limiter.calculate(speed));
            }
        } else {
            m_motor1.set(m_limiter.calculate(speed));
            m_motor2.set(m_limiter.calculate(speed));
        }

    }

    public void setClutch(boolean engaged) {
        m_clutch.set(engaged ? Value.kReverse : Value.kForward);
        clutchState = engaged ? ClutchState.Engaged : ClutchState.Disengaged;
    }

    public void setPistion(boolean extended) {
        m_pistion.set(extended ? Value.kForward : Value.kReverse);
        pistonState = extended ? PistonState.Extended : PistonState.Retracted;
    }

    public void togglePiston() {
        m_pistion.toggle();
    }

    public void toggleClutch() {
        m_clutch.toggle();
    }

    public boolean atPosition() {
        return climberState == ClimberState.Retracted || climberState == ClimberState.Extended;
    }

    public double getEncoder() {
        return m_absolutEncoder.getDistance();
    }

    public void resetEncoder() {
        m_absolutEncoder.reset();
    }
}