package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private CANSparkMax m_motor1;
    private CANSparkMax m_motor2;
    private DutyCycleEncoder m_absolutEncoder;
    private ProfiledPIDController m_PIDController;

    private Solenoid m_pistion;

    public ClimberState climberState = ClimberState.Retracted;
    public ClutchState clutchState = ClutchState.Engaged;
    public PistonState pistonState = PistonState.Extended;

    public LimitSwitchState limitSwitchState;

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

    public enum ClimberPosition {
        retracted,
        relesed,
        extended
    }

    private boolean PIDEnabled;

    private boolean ExtendedClimber;

    private Timer m_pistonExtentionTimer;

    public ClimberSubsystem() {
        m_motor1 = new CANSparkMax(ClimberConstants.motor1ID, MotorType.kBrushless);
        m_motor2 = new CANSparkMax(ClimberConstants.motor2ID, MotorType.kBrushless);

        m_motor1.restoreFactoryDefaults();
        m_motor2.restoreFactoryDefaults();

        m_motor2.follow(m_motor1);

        m_motor1.setIdleMode(IdleMode.kBrake);
        m_motor2.setIdleMode(IdleMode.kBrake);

        m_motor1.setSmartCurrentLimit(60, 40);
        m_motor2.setSmartCurrentLimit(60, 40);

        // m_encoder = new Encoder(3, 4, 5);
        m_absolutEncoder = new DutyCycleEncoder(2);
        m_absolutEncoder.reset();

        // m_encoder = m_motor1.getEncoder();
        // m_encoder.setPositionConversionFactor(ClimberConstants.conversenFactor_SensorUnitsPerInch);

        m_pistion = new Solenoid(Constants.pnumaticHub, PneumaticsModuleType.REVPH,
                ClimberConstants.pistionChannel);

        m_pistion.set(false);

        pistonState = PistonState.Extended;
        clutchState = ClutchState.Engaged;

        // m_limitSwitch = new DigitalInput(ClimberConstants.limitSwitchID);
        // limitSwitchState = LimitSwitchState.changing;

        m_PIDController = new ProfiledPIDController(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD,
                new TrapezoidProfile.Constraints(ClimberConstants.maxVelocity, ClimberConstants.maxAcceleration));

        m_PIDController.setTolerance(ClimberConstants.PositonTolerace);

        ExtendedClimber = true;

        m_pistonExtentionTimer = new Timer();
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Absolut Encoder Distance", m_absolutEncoder.get());

        if (m_absolutEncoder.getDistance() < 0) {
            m_absolutEncoder.reset();
        }

        if (PIDEnabled) {
            double dutyCycle = m_PIDController.calculate(m_absolutEncoder.get());
            m_motor1.set(dutyCycle);
            m_motor2.set(dutyCycle);
        }

        // SmartDashboard.putBoolean("PID Controled", PIDEnabled);

    }

    public void setClimber(double speed) {
        if (!PIDEnabled) {
            if (speed > 0 && m_absolutEncoder.getDistance() >= ClimberConstants.maxHight_Rotations) {
                m_motor1.set(0);
                m_motor2.set(0);
            } else {
                m_motor1.set(speed);
                m_motor2.set(speed);
            }
        }
    }

    public void setPistion(boolean extended) {
        m_pistion.set(!extended);
        ExtendedClimber = extended;

        pistonState = extended ? PistonState.Extended : PistonState.Retracted;
    }

    public void togglePiston() {
        m_pistion.toggle();
        ExtendedClimber = !ExtendedClimber;

        pistonState = m_pistion.get() ? PistonState.Retracted : PistonState.Extended;
    }

    public void setPosition(ClimberPosition position) {
        PIDEnabled = true;
        m_PIDController.reset(m_absolutEncoder.get());
        switch (position) {
            case retracted:
                m_PIDController.setGoal(0);
                break;
            case relesed:
                m_PIDController.setGoal(1);
                break;
            case extended:
                m_PIDController.setGoal(ClimberConstants.maxHight_Rotations);
                break;
        }
    }

    public void DisablePositionContorl() {
        PIDEnabled = false;
    }

    public double getEncoder() {
        return m_absolutEncoder.getDistance();
    }

    public void resetEncoder() {
        m_absolutEncoder.reset();
    }

    public boolean atPosition() {
        if (m_PIDController.getPositionError() < ClimberConstants.PositonTolerace) {
            return true;
        } else {
            return false;
        }
    }
}