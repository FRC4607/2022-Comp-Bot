package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;;

public class ClimberSubsystem extends SubsystemBase {
    private CANSparkMax m_motor1;
    private CANSparkMax m_motor2;

    private SparkMaxPIDController m_PIDController;

    private DoubleSolenoid m_pistion;
    private DoubleSolenoid m_clutch;

    public ClimberSubsystem() {
        m_motor1 = new CANSparkMax(ClimberConstants.motor1ID, MotorType.kBrushless);
        m_motor2 = new CANSparkMax(ClimberConstants.motor2ID, MotorType.kBrushless);

        m_motor2.follow(m_motor1);

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

    }

    public void setPosition(double postion) {
        m_PIDController.setReference(postion, ControlType.kPosition);
    }
    
    public void setClutch(boolean engaged) {
        if (engaged) {
            m_clutch.set(Value.kReverse);
        } else {
            m_clutch.set(Value.kForward);
        }
    }

    public void setPistion(boolean extended) {
        if (extended) {
            m_pistion.set(Value.kForward);
        } else {
            m_pistion.set(Value.kReverse);
        }
    }

    public double getEncoder() {
        return m_motor1.getEncoder().getPosition();
    }
}