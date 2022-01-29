// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

/**
 *
 */
public class FlywheelSubsystem extends SubsystemBase {

    private WPI_TalonFX m_flywheelMotor1;
    private WPI_TalonFX m_flywheelMotor2;
    private final TalonFXConfiguration motorConfig;
    /**
    *
    */
    public FlywheelSubsystem() {
        motorConfig = new TalonFXConfiguration();
        motorConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 35, 40, 0.2);
        motorConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        motorConfig.nominalOutputForward = 0;
        motorConfig.nominalOutputReverse = 0;
        motorConfig.peakOutputForward = 1;
        motorConfig.peakOutputReverse = -1;
        motorConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

        m_flywheelMotor1 = new WPI_TalonFX(Constants.FlywheelConstants.flywheelMotor1ID);
        m_flywheelMotor2 = new WPI_TalonFX(Constants.FlywheelConstants.flywheelMotor2ID);

        /* Factory default hardware to prevent unexpected behavior */
        m_flywheelMotor1.configFactoryDefault();
        m_flywheelMotor2.configFactoryDefault();

        m_flywheelMotor1.configAllSettings(motorConfig);
        m_flywheelMotor2.configAllSettings(motorConfig);
        
        /* Invert Motor? and set Break Mode */
        m_flywheelMotor1.setInverted(true);
        m_flywheelMotor1.setNeutralMode(NeutralMode.Coast);
        m_flywheelMotor2.setNeutralMode(NeutralMode.Coast);
        m_flywheelMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        m_flywheelMotor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        m_flywheelMotor1.configSelectedFeedbackCoefficient(1 / 2048);
        m_flywheelMotor2.configSelectedFeedbackCoefficient(1 / 2048);

        m_flywheelMotor2.follow(m_flywheelMotor1);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Flywheel RPM", m_flywheelMotor1.getSelectedSensorVelocity() / 2048 * 600);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void setSpeed(double speed) {
        m_flywheelMotor1.set(speed);
    }
}
