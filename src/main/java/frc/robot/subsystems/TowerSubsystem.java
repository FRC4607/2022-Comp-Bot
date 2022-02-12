package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TowerConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class TowerSubsystem extends SubsystemBase {

    private CANSparkMax m_agitatior;

    private DigitalInput m_lowBrakeBeam;
    private DigitalInput m_midBrakeBeam;
    private DigitalInput m_highBrakeBeam;

    private boolean m_autoTower = false;

    public TowerSubsystem() {

        m_agitatior = new CANSparkMax(TowerConstants.agitatiorID, MotorType.kBrushless);
        m_agitatior.restoreFactoryDefaults();
        m_agitatior.setInverted(false);
        m_agitatior.setIdleMode(IdleMode.kBrake);

        m_lowBrakeBeam = new DigitalInput(TowerConstants.lowBrakeBeamID);
        m_midBrakeBeam = new DigitalInput(TowerConstants.midBrakeBeamID);
        m_highBrakeBeam = new DigitalInput(TowerConstants.highBrakeBeamID);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Low Brake Beam", m_lowBrakeBeam.get());
        SmartDashboard.putBoolean("Mid Brake Beam", m_midBrakeBeam.get());
        SmartDashboard.putBoolean("High Brake Beam", m_highBrakeBeam.get());

        if (m_autoTower) {
            String Color = "none";
            String CS = "none";
            if ( !(m_midBrakeBeam.get() && m_highBrakeBeam.get()) && !( m_lowBrakeBeam.get() && !(Color == CS || Color == "none") ) ) {
                setSpeed(TowerConstants.agitatiorSpeed);
            } else {
                setSpeed(0);
            }
        }
    }

    public void setSpeed(double speed) {
        m_agitatior.set(speed);
    }

    /**
     * Sets the voltage of the motor
     * @param voltage
     */
    public void setVoltage(double voltage) {
        m_agitatior.setVoltage(voltage);
    }

    public void AutoTower(boolean enabled) {
        m_autoTower = enabled;
    }
}
