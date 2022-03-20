package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
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
    
    private AddressableLED m_LEDs;
    private AddressableLEDBuffer m_LEDBuffer;
    // 27, 26
    
    private NetworkTableEntry alienceColorEntry;

    public TowerSubsystem() {

        m_agitatior = new CANSparkMax(TowerConstants.agitatiorID, MotorType.kBrushless);
        m_agitatior.restoreFactoryDefaults();
        m_agitatior.setInverted(false);
        m_agitatior.setIdleMode(IdleMode.kBrake);
        
        m_agitatior.setSmartCurrentLimit(40, 20);
        
        m_midBrakeBeam = new DigitalInput(TowerConstants.midBrakeBeamID);
        m_highBrakeBeam = new DigitalInput(TowerConstants.highBrakeBeamID);
        
        m_LEDs = new AddressableLED(9);
        m_LEDs.setLength(27);
        m_LEDBuffer = new AddressableLEDBuffer(27);
        m_LEDs.setData(m_LEDBuffer);
        m_LEDs.start();

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable FMSInfo = inst.getTable("FMSInfo");
        alienceColorEntry = FMSInfo.getEntry("IsRedAlliance");
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Mid Brake Beam", m_midBrakeBeam.get());
        SmartDashboard.putBoolean("High Brake Beam", m_highBrakeBeam.get());

        m_LEDs.setData(m_LEDBuffer);

        boolean isRed = alienceColorEntry.getBoolean(true);
        boolean[] state = { !m_midBrakeBeam.get(), !m_highBrakeBeam.get() };

        for (int i = 0; i < 13; i++) {
            if (state[0]) {
                m_LEDBuffer.setHSV(i, isRed ? 0 : 100, 255, 255);
            } else {
                m_LEDBuffer.setHSV(i, 0, 0, 0);
            }
        }

        for (int i = 13; i < 27; i++) {
            if (state[1]) {
                m_LEDBuffer.setHSV(i, isRed ? 0 : 100, 255, 255);
            } else {
                m_LEDBuffer.setHSV(i, 0, 0, 0);
            }
        }

        m_LEDs.setData(m_LEDBuffer);

        // i++;
        // i %= 27;
        // m_LEDBuffer.setHSV((i+1)%27, 90, 255, 255);
        // m_LEDBuffer.setHSV(i, 0, 0, 0);

    }

    public void setSpeed(double speed) {
        m_agitatior.set(speed);
    }

    /**
     * Sets the voltage of the motor
     * 
     * @param voltage
     */
    public void setVoltage(double voltage) {
        m_agitatior.setVoltage(voltage);
    }

    public Color getColorSensor() {
        return m_ColorSensor;
    }

    public void setAllianceColor(Color allianceColor) {
        m_allianceColor = allianceColor;
    }

    public Color getAllianceColor() {
        return m_allianceColor;
    }

    public boolean getMidBrakeBeam() {
        return m_midBrakeBeam.get();
    }

    public boolean getHighBrakeBeam() {
        return m_highBrakeBeam.get();
    }
}
