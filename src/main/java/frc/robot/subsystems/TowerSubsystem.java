package frc.robot.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TowerConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class TowerSubsystem extends SubsystemBase {

    private CANSparkMax m_agitatior;

    private DigitalInput m_midBrakeBeam;
    private DigitalInput m_highBrakeBeam;

    private boolean m_autoTower = false;
    private Color m_allianceColor = Color.None;
    private Color m_ColorSensor;

    public static enum Color {
        None(0), Red(1), Blue(2);

        private int value;

        Color(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    public TowerSubsystem() {

        m_agitatior = new CANSparkMax(TowerConstants.agitatiorID, MotorType.kBrushless);
        m_agitatior.restoreFactoryDefaults();
        m_agitatior.setInverted(false);
        m_agitatior.setIdleMode(IdleMode.kBrake);

        m_midBrakeBeam = new DigitalInput(TowerConstants.midBrakeBeamID);
        m_highBrakeBeam = new DigitalInput(TowerConstants.highBrakeBeamID);
    }
    public TowerSubsystem(Color teamColor) {

        m_agitatior = new CANSparkMax(TowerConstants.agitatiorID, MotorType.kBrushless);
        m_agitatior.restoreFactoryDefaults();
        m_agitatior.setInverted(false);
        m_agitatior.setIdleMode(IdleMode.kBrake);

        m_midBrakeBeam = new DigitalInput(TowerConstants.midBrakeBeamID);
        m_highBrakeBeam = new DigitalInput(TowerConstants.highBrakeBeamID);

        m_allianceColor = teamColor;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
		NetworkTable FMSInfo = inst.getTable("FMSInfo");
		NetworkTableEntry alienceColor = FMSInfo.getEntry("IsRedAllience");

		alienceColor.addListener((event) -> {
			m_allianceColor = Color.values()[(int)event.getEntry().getNumber(0)];
		}, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

		NetworkTable PIDatabase = inst.getTable("PiTable");
		NetworkTableEntry ColorSensor = PIDatabase.getEntry("");
		ColorSensor.addListener((event) -> {
			m_ColorSensor = Color.values()[(int)event.getEntry().getNumber(0)];
		}, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Mid Brake Beam", m_midBrakeBeam.get());
        SmartDashboard.putBoolean("High Brake Beam", m_highBrakeBeam.get());
        
        if ( m_autoTower && 
            (m_midBrakeBeam.get() || m_highBrakeBeam.get()) && 
            (getColorSensor() == m_allianceColor || getColorSensor() == Color.None || m_allianceColor == Color.None) 
        ) {
            setSpeed(TowerConstants.agitatiorSpeed);
        } else {
            setSpeed(0);
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

    public Color getColorSensor() {
        return m_ColorSensor;
    }

    public void setAllianceColor(Color allianceColor) {
        m_allianceColor = allianceColor;
    }
}
