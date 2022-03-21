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
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 *
 */
public class ShooterSubsystem extends SubsystemBase {

    private WPI_TalonFX m_flywheelMotor1;
    private WPI_TalonFX m_flywheelMotor2;
    private final TalonFXConfiguration motorConfig;

    private ArrayList<Double> speeds = new ArrayList<>(Arrays.asList(0.0, 0.0, 0.0, 0.0, 0.0)); // 5 items
    private CANSparkMax m_kickerWheel;
    private Solenoid m_piston;

    boolean m_lowGoal;

    private ShootingMode m_mode;

    public enum ShootingMode {
        lowGoal,
        highGoal,
        limeLight
    }

    double m_LimeLightRPM;

    /**
    *
    */
    public ShooterSubsystem() {
        motorConfig = new TalonFXConfiguration();
        motorConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 35, 40, 0.2);
        motorConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        motorConfig.nominalOutputForward = 0;
        motorConfig.nominalOutputReverse = 0;
        motorConfig.peakOutputForward = 1;
        motorConfig.peakOutputReverse = -1;
        motorConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

        m_flywheelMotor1 = new WPI_TalonFX(Constants.ShooterConstants.flywheelMotor1ID);
        m_flywheelMotor2 = new WPI_TalonFX(Constants.ShooterConstants.flywheelMotor2ID);

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

        // Set up PID
        TalonFXConfiguration pidConfig = new TalonFXConfiguration();
        pidConfig.remoteFilter0.remoteSensorDeviceID = m_flywheelMotor2.getDeviceID();
        pidConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor;

        setRobotDistanceConfigs(TalonFXInvertType.Clockwise, pidConfig);

        pidConfig.slot0.kF = ShooterConstants.flywheelF;
        // pidConfig.slot0.kF = 0.05;
        pidConfig.slot0.kP = ShooterConstants.flywheelP;
        // pidConfig.slot0.kP = 0;
        // pidConfig.slot0.kI = 0;
        pidConfig.slot0.kD = ShooterConstants.flywheelD;

        m_flywheelMotor1.configAllSettings(pidConfig);

        m_flywheelMotor2.follow(m_flywheelMotor1);

        m_kickerWheel = new CANSparkMax(ShooterConstants.kickerWheelID, MotorType.kBrushless);

        m_kickerWheel.restoreFactoryDefaults();
        m_kickerWheel.setInverted(false);
        m_kickerWheel.setIdleMode(IdleMode.kBrake);

        m_kickerWheel.setSmartCurrentLimit(40, 20);

        m_piston = new Solenoid(Constants.pnumaticHub, PneumaticsModuleType.REVPH, ShooterConstants.pistionChannel);
        m_piston.set(false);

        SmartDashboard.putNumber("Low goal RPM", ShooterConstants.lowGoalRPM);
        SmartDashboard.putNumber("High goal RPM", ShooterConstants.highGoalRPM);
        SmartDashboard.putNumber("Lime Light RPM", 2825);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Flywheel RPM", m_flywheelMotor1.getSelectedSensorVelocity() / 2048 * 600);
        // SmartDashboard.putNumber("Flywheel Error",
        // m_flywheelMotor1.getClosedLoopError());

        speeds.remove(0);
        speeds.add(m_flywheelMotor1.getSelectedSensorVelocity());

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

    public void setRPM(double rpm) {
        double target = rpm * 2048 / 600; // Taken from CTRE's code on how to convert to native units.
        m_flywheelMotor1.set(ControlMode.Velocity, target);
        // , DemandType.ArbitraryFeedForward,
        //         (ShooterConstants.flywheelKs + rpm * ShooterConstants.flywheelKv / 60)
        //                 / m_flywheelMotor1.getBusVoltage());
    }

    public void spinupFlywheel() {
        switch (m_mode) {
            case lowGoal:
                setRPM(SmartDashboard.getNumber("Low goal RPM", ShooterConstants.lowGoalRPM));
                m_piston.set(true);
                break;
            case highGoal:
                setRPM(SmartDashboard.getNumber("High goal RPM", ShooterConstants.highGoalRPM));
                m_piston.set(false);
                break;
            case limeLight:
                setRPM(SmartDashboard.getNumber("Lime Light RPM", ShooterConstants.highGoalRPM));
                m_piston.set(true);
                break;
        }
    }

    public double getFlywheelError() {
        return m_flywheelMotor1.getClosedLoopError();
    }

    public boolean constantSpeed() {
        return (Collections.max(speeds) - Collections.min(speeds)) < ShooterConstants.flywheelMaxError;
    }

    // This is taken from CTRE's sample code at
    // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20Talon%20FX%20(Falcon%20500)/MotionMagic_ArbFeedForward/src/main/java/frc/robot/Robot.java.

    /**
     * Determines if SensorSum or SensorDiff should be used
     * for combining left/right sensors into Robot Distance.
     * 
     * Assumes Aux Position is set as Remote Sensor 0.
     * 
     * configAllSettings must still be called on the master config
     * after this function modifies the config values.
     * 
     * @param masterInvertType Invert of the Master Talon
     * @param masterConfig     Configuration object to fill
     */
    void setRobotDistanceConfigs(TalonFXInvertType masterInvertType, TalonFXConfiguration masterConfig) {
        /**
         * Determine if we need a Sum or Difference.
         * 
         * The auxiliary Talon FX will always be positive
         * in the forward direction because it's a selected sensor
         * over the CAN bus.
         * 
         * The master's native integrated sensor may not always be positive when forward
         * because
         * sensor phase is only applied to *Selected Sensors*, not native
         * sensor sources. And we need the native to be combined with the
         * aux (other side's) distance into a single robot distance.
         */

        /*
         * THIS FUNCTION should not need to be modified.
         * This setup will work regardless of whether the master
         * is on the Right or Left side since it only deals with
         * distance magnitude.
         */

        /* Check if we're inverted */
        if (masterInvertType == TalonFXInvertType.Clockwise) {
            /*
             * If master is inverted, that means the integrated sensor
             * will be negative in the forward direction.
             * If master is inverted, the final sum/diff result will also be inverted.
             * This is how Talon FX corrects the sensor phase when inverting
             * the motor direction. This inversion applies to the *Selected Sensor*,
             * not the native value.
             * Will a sensor sum or difference give us a positive total magnitude?
             * Remember the Master is one side of your drivetrain distance and
             * Auxiliary is the other side's distance.
             * Phase | Term 0 | Term 1 | Result
             * Sum: -((-)Master + (+)Aux )| NOT OK, will cancel each other out
             * Diff: -((-)Master - (+)Aux )| OK - This is what we want, magnitude will be
             * correct and positive.
             * Diff: -((+)Aux - (-)Master)| NOT OK, magnitude will be correct but negative
             */

            masterConfig.diff0Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); // Local Integrated
                                                                                                // Sensor
            masterConfig.diff1Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice(); // Aux Selected Sensor
            masterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice(); // Diff0
                                                                                                                        // -
                                                                                                                        // Diff1
        } else {
            /* Master is not inverted, both sides are positive so we can sum them. */
            masterConfig.sum0Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice(); // Aux Selected Sensor
            masterConfig.sum1Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); // Local IntegratedSensor
            masterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorSum.toFeedbackDevice(); // Sum0
                                                                                                                 // +
                                                                                                                 // Sum1
        }

        /*
         * Since the Distance is the sum of the two sides, divide by 2 so the total
         * isn't double
         * the real-world value
         */
        masterConfig.primaryPID.selectedFeedbackCoefficient = 0.5;
    }

    public void setKickerWheel(double speed) {
        if (speed > 0.01) {
            System.out.println("Runing Kicker");
        }
        else {
            System.out.println("Kicker Commanded to 0");
        }
        m_kickerWheel.set(speed);
    }

    public void setPiston(boolean extended) {
        m_piston.set(extended);
    }

    public void togglePsiton() {
        m_piston.toggle();
    }

    public void setShootingMode(ShootingMode mode) {
        m_mode = mode;
    }

    public void setLimeLightRPM(double RPM) {
        m_LimeLightRPM = RPM;
    }
}
