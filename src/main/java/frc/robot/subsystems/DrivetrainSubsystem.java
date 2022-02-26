package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 *
 */
public class DrivetrainSubsystem extends SubsystemBase {
    private CANCoder m_leftEncoder;
    private CANCoder m_rightEncoder;
    private WPI_TalonFX m_leftMotor1;
    private WPI_TalonFX m_leftMotor2;
    private WPI_TalonFX m_rightMotor1;
    private WPI_TalonFX m_rightMotor2;
    private MotorControllerGroup m_leftDrive;
    private MotorControllerGroup m_rightDrive;
    private DifferentialDrive m_drivetrain;
    private final CANCoderConfiguration m_encoderConfig;
    private final TalonFXConfiguration m_motorConfig;
    // private Pigeon2 m_pigeon2;
    private PigeonIMU m_pigeonIMU;
    private final SlewRateLimiter m_rateLimiter;

    private final DifferentialDriveOdometry m_odometry;

    public final Field2d m_field = new Field2d();

    // Motion Planing
    public final RamseteController m_ramseteController = new RamseteController(
            DriveConstants.kRamseteB_radSquaredPerMetersSquared, DriveConstants.kRamseteZeta_PerRad);
    public final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(DriveConstants.ks_Volts,
            DriveConstants.kv_VoltSecondsPerMeters, DriveConstants.ka_VoltSecondsSquaredPerMeters);

    public final DifferentialDriveKinematics m_kinematics = DriveConstants.kDriveKinematics;
    public final PIDController m_leftController = new PIDController(DriveConstants.kPDriveVel, 0, 0);
    public final PIDController m_rightController = new PIDController(DriveConstants.kPDriveVel, 0, 0);

    /**
    *
    */
    public DrivetrainSubsystem() {
        m_encoderConfig = new CANCoderConfiguration();
        m_encoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        m_encoderConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        m_encoderConfig.sensorCoefficient = DriveConstants.sensorCoefficient;
        m_encoderConfig.unitString = "meters";

        m_leftEncoder = new CANCoder(DriveConstants.leftEncoderID);
        m_leftEncoder.configFactoryDefault();
        m_leftEncoder.configAllSettings(m_encoderConfig);
        m_leftEncoder.configSensorDirection(true);

        m_rightEncoder = new CANCoder(DriveConstants.rightEncoderID);
        m_rightEncoder.configFactoryDefault();
        m_rightEncoder.configAllSettings(m_encoderConfig);

        // Motor config
        m_motorConfig = new TalonFXConfiguration();
        m_motorConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 35, 40, 0.2);
        // m_motorConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 20,
        // 40, 0.1);
        m_motorConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        m_motorConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        m_motorConfig.remoteFilter0.remoteSensorDeviceID = m_leftEncoder.getDeviceID();
        m_motorConfig.remoteFilter1.remoteSensorSource = RemoteSensorSource.CANCoder;
        m_motorConfig.remoteFilter1.remoteSensorDeviceID = m_rightEncoder.getDeviceID();

        m_leftMotor1 = new WPI_TalonFX(DriveConstants.leftMotor1ID);
        m_leftMotor1.configFactoryDefault();
        m_leftMotor1.configAllSettings(m_motorConfig);
        //m_leftMotor1.setNeutralMode(NeutralMode.Brake);
        m_leftMotor2 = new WPI_TalonFX(DriveConstants.leftMotor2ID);
        m_leftMotor2.configFactoryDefault();
        m_leftMotor2.configAllSettings(m_motorConfig);
        //m_leftMotor2.setNeutralMode(NeutralMode.Brake);

        m_leftDrive = new MotorControllerGroup(m_leftMotor1, m_leftMotor2);
        addChild("LeftDrive", m_leftDrive);

        // Right motors
        m_rightMotor1 = new WPI_TalonFX(DriveConstants.rightMotor1ID);
        m_rightMotor1.configFactoryDefault();
        m_rightMotor1.configAllSettings(m_motorConfig);
        m_rightMotor1.setInverted(true);
        //m_rightMotor1.setNeutralMode(NeutralMode.Brake);
        m_rightMotor2 = new WPI_TalonFX(DriveConstants.rightMotor2ID);
        m_rightMotor2.configFactoryDefault();
        m_rightMotor2.configAllSettings(m_motorConfig);
        m_rightMotor2.setInverted(true);
        //m_rightMotor2.follow(m_rightMotor1);
        //m_rightMotor2.setNeutralMode(NeutralMode.Brake);

        m_rightDrive = new MotorControllerGroup(m_rightMotor1, m_rightMotor2);
        addChild("RightDrive", m_rightDrive);

        m_drivetrain = new DifferentialDrive(m_leftDrive, m_rightDrive);
        // drivetrain = new DifferentialDrive(leftMotor2, rightMotor2);
        addChild("Drivetrain", m_drivetrain);

        m_drivetrain.setSafetyEnabled(true);
        m_drivetrain.setExpiration(0.1);
        m_drivetrain.setMaxOutput(1.0);

        // Set up encoder for left side
        // Pidion IMU
        // m_pigeon2 = new Pigeon2(DriveConstants.pigeonID);
        // m_pigeon2.configFactoryDefault();
        // m_pigeon2.configMountPose(-90, -90, 0);
        // m_pigeon2.setYaw(0);

        m_pigeonIMU = new PigeonIMU(DriveConstants.pigeonID);
        m_pigeonIMU.setFusedHeading(0);

        m_odometry = new DifferentialDriveOdometry(
                new Rotation2d(getGyroscopeReadingRad()));

        m_rateLimiter = new SlewRateLimiter(DriveConstants.maxUnitsPerSecond);

        SmartDashboard.putData("Field", m_field);

        // SmartDashboard.putNumber("Max drive speed", DriveConstants.maxSpeed);
        // SmartDashboard.putNumber("Max drive turning", DriveConstants.maxTurning);
    }

    @Override
    public void periodic() {
        m_odometry.update(getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition());
        m_field.setRobotPose(m_odometry.getPoseMeters());

        // SmartDashboard.putNumber("Front Left Voltage",
        // m_leftMotor1.getMotorOutputVoltage());
        // SmartDashboard.putNumber("Back Left Voltage",
        // m_leftMotor2.getMotorOutputVoltage());
        // SmartDashboard.putNumber("Front Right Voltage",
        // m_rightMotor1.getMotorOutputVoltage());
        // SmartDashboard.putNumber("Back Right Voltage",
        // m_rightMotor2.getMotorOutputVoltage());
        SmartDashboard.putNumber("Left Encoder", getLeftEncoderPosition());
        SmartDashboard.putNumber("Right Encoder", getRightEncoderPosition());

        SmartDashboard.putNumber("Gyroscrope", getGyroscopeReadingContinuous());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    // Motors

    /**
     * Sets the voltage of the left side motors. Needs to be called continuously.
     * Will be overwritten by calls to setArcadeDrive!
     * 
     * @param voltage The voltage to set the left side of the drivtrain to.
     */
    public void setLeftVoltage(double voltage) {
        m_leftDrive.setVoltage(voltage);
    }

    /**
     * Sets the voltage of the right side motors. Needs to be called continuously.
     * Will be overwritten by calls to setArcadeDrive!
     * 
     * @param voltage The voltage to set the right side of the drivtrain to.
     */
    public void setRightVoltage(double voltage) {
        m_rightDrive.setVoltage(voltage);
    }

    /**
     * Sets the voltage of both the left and right side motors. Needs to be called
     * continuously. Will be overwritten by calls to setArcadeDrive!
     * 
     * @param voltage The voltage to set both sides of the drivetrain to.
     */
    public void setVoltage(double voltage) {
        m_leftDrive.setVoltage(voltage);
        m_rightDrive.setVoltage(voltage);
        m_leftMotor1.setVoltage(voltage);
    }

    /**
     * Allows the joystick to be used in a way people understand. Arcade style.
     * 
     * @param x The x-axis of the controller.
     * @param y The y-axis of the controller.
     */
    public void setArcadeDrive(double x, double y) {
        double maxSpeed = SmartDashboard.getNumber("Max speed", DriveConstants.maxSpeed);
        double maxTurning = SmartDashboard.getNumber("Max turning", DriveConstants.maxTurning);

        // The y is inverted so forward will be positive and backward will be negative.
        m_drivetrain.arcadeDrive(m_rateLimiter.calculate(-y * maxSpeed), x * maxTurning);

        // YAAY my first code!!! I did something useful! -Helen
    }

    /**
     * Sets the Voltages of the left and right side independently
     * 
     * @param leftVolts  Left side Voltage
     * @param rightVolts Right side Voltage
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftVolts = Math.max(Math.min(leftVolts, 12), -12);
        rightVolts = Math.max(Math.min(rightVolts, 12), -12);
        SmartDashboard.putNumber("Left Volts", leftVolts);
        SmartDashboard.putNumber("Right Volts", rightVolts);
        m_leftDrive.setVoltage(leftVolts);
        m_rightDrive.setVoltage(rightVolts);
        m_drivetrain.feed();
    }

    // Encoders

    /**
     * Returns the left encoder position.
     * 
     * @return The position of the right side of the drivetrain in meters.
     */
    public double getLeftEncoderPosition() {
        return m_leftEncoder.getPosition();
        // return (m_leftMotor1.getSelectedSensorPosition() + m_leftMotor2.getSelectedSensorPosition()) * 0.5 * DriveConstants.MeterPerFalconCount;
    }

    /**
     * Returns the right encoder position.
     * 
     * @return The position of the right side of the drivetrain in meters.
     */
    public double getRightEncoderPosition() {
        return m_rightEncoder.getPosition();
        // return (m_rightMotor1.getSelectedSensorPosition() + m_rightMotor2.getSelectedSensorPosition()) * 0.5 * DriveConstants.MeterPerFalconCount;
    }

    /**
     * Sets the position of the left encoder.
     * 
     * @param position The position of the left side of the drivetrain in meters.
     */
    public void setLeftEncoderPostion(double position) {
        m_leftEncoder.setPosition(position);
        // m_leftMotor1.setSelectedSensorPosition(position / DriveConstants.MeterPerFalconCount);
        // m_leftMotor2.setSelectedSensorPosition(position / DriveConstants.MeterPerFalconCount);
    }

    /**
     * Sets the position of the right encoder.
     * 
     * @param position The position of the right side of the drivetrain in meters.
     */
    public void setRightEncoderPostion(double position) {
        m_rightEncoder.setPosition(position);
        // m_rightMotor1.setSelectedSensorPosition(position / DriveConstants.MeterPerFalconCount);
        // m_rightMotor2.setSelectedSensorPosition(position / DriveConstants.MeterPerFalconCount);
    }

    /**
     * Sets the position of both the left and right side encoders at the same time.
     * 
     * @param position The position of the two sides of the drivetrain in meters.
     */
    public void setEncoderPostion(double position) {
        m_leftEncoder.setPosition(position);
        m_rightEncoder.setPosition(position);
        // setLeftEncoderPostion(position);
        // setRightEncoderPostion(position);
    }

    /**
     * Sets the encoders to 0
     */
    public void resetEncoders() {
        setEncoderPostion(0);
    }

    /**
     * Return the Velocity of the left Encoder
     */
    public double getLeftVelocity() {
        return m_leftEncoder.getVelocity();
        // return (m_leftMotor1.getSelectedSensorVelocity() + m_leftMotor2.getSelectedSensorVelocity()) * 5
        //         * DriveConstants.MeterPerFalconCount;
    }

    /**
     * Return the Velocity of the right Encoder
     */
    public double getRightVelocity() {
        return m_rightEncoder.getVelocity();
        // return (m_rightMotor1.getSelectedSensorVelocity() + m_rightMotor2.getSelectedSensorVelocity()) * 5
        //         * DriveConstants.MeterPerFalconCount;
    }

    // Gyroscope

    /*
     * Quick note about Rotation2d from WPILib:
     * - It's based on radians, so all angles are internally stored as a x and y
     * coordinate on the unit circle, or cos and sin components respectively.
     * - Therefore, Rotation2d is CCW positive, just like the radian system.
     * - This also means you must be really careful to not pass in any degree
     * measures without converting them first.
     * - Rotation2d goes from -inf to inf, so you can use it continuously.
     */

    /**
     * Returns the robot's heading, ranging from -inf to inf, with counterclockwise
     * being positive.
     * 
     * @return The rotation of the robot in degrees, with counterclockwise rotations
     *         increasing this value.
     */
    public double getGyroscopeReadingContinuous() {
        return getGyroscopeReadingContinuous(true);
    }

    /**
     * Returns the robot's heading, ranging from -inf to inf.
     * 
     * @param ccwPositive Whether the reading should be returned treating
     *                    counterclockwise as positive or negative, with true
     *                    meaning ccw is positive.
     * @return The rotation of the robot in degrees.
     */
    public double getGyroscopeReadingContinuous(boolean ccwPositive) {
        // if (ccwPositive) {
        //     return m_pigeon2.getYaw();
        // } else {
        //     return -m_pigeon2.getYaw();
        // }
        if (ccwPositive) {
            return m_pigeonIMU.getFusedHeading();
        } else {
            return -m_pigeonIMU.getFusedHeading();
        }
    }


    // /**
    //  * Returns the robot's heading, with a range of [-180, 180], with
    //  * counterclockwise being positive.
    //  * 
    //  * @return The rotation of the robot in degrees, with counterclockwise rotations
    //  *         increasing this value.
    //  */
    // public double getGyroscopeReading() {
    //     return getGyroscopeReading(true);
    // }

    // /**
    //  * Returns the robot's heading, with a range of [-180, 180].
    //  * 
    //  * @param ccwPositive Whether the reading should be returned treating
    //  *                    counterclockwise as positive or negative, with true
    //  *                    meaning ccw is positive.
    //  * @return The rotation of the robot in degrees.
    //  */
    // public double getGyroscopeReading(boolean ccwPositive) {
    //     double continuousHeading = getGyroscopeReadingContinuous(ccwPositive);

    //     /**
    //      * Corrects the values to the range of [-180, 180]
    //      * 
    //      * First the value is caped to [-360, 360],
    //      * than if the value is outside of [-180, 180]
    //      * 360 is added or subtracted as nesasary to get within the desiered range
    //      */
    //     double correctedValue = continuousHeading % 360;

    //     if (correctedValue > 180) {
    //         return correctedValue - 360;
    //     } else if (correctedValue < -180) {
    //         return correctedValue + 360;
    //     } else {
    //         return correctedValue;
    //     }
    // }

    /**
     * returns the rotation of the gyrosope in Radians
     */
    public double getGyroscopeReadingRad() {
        return Units.degreesToRadians(getGyroscopeReadingContinuous());
    }

    /**
     * returns the rotation of the gyroscop in a Rotation2d
     * 
     * @return Rotation2d
     */
    public Rotation2d getRotation2d() {
        return new Rotation2d(getGyroscopeReadingRad());
    }

    /**
     * Sets the yaw to 0 deg.
     */
    public void zeroHeading() {
        // m_pigeon2.setYaw(0);
        m_pigeonIMU.setYaw(0);
    }

    // Odometry

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, getRotation2d());
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
    }

    // PID, FF, Ramset

    public DifferentialDriveWheelSpeeds getRamsetTargetWheelSpeeds(State tragectorySample) {
        return m_kinematics.toWheelSpeeds(m_ramseteController.calculate(getPose(), tragectorySample));

    }

    public SimpleMotorFeedforward Feedforward() {
        return m_feedforward;
    }

    public double getLeftPID(double leftSpeedSetpoint) {
        return m_leftController.calculate(getWheelSpeeds().leftMetersPerSecond,
                leftSpeedSetpoint);
    }

    public double getRightPID(double rightSpeedSetpoint) {
        return m_rightController.calculate(getWheelSpeeds().rightMetersPerSecond,
                rightSpeedSetpoint);
    }

    public void resetPID() {
        m_leftController.reset();
        m_rightController.reset();
    }

    public void setBrakeMode(boolean brakeMode) {
        m_rightMotor1.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
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
    // void setRobotDistanceConfigs(TalonFXInvertType masterInvertType, TalonFXConfiguration masterConfig) {
    //     /**
    //      * Determine if we need a Sum or Difference.
    //      * 
    //      * The auxiliary Talon FX will always be positive
    //      * in the forward direction because it's a selected sensor
    //      * over the CAN bus.
    //      * 
    //      * The master's native integrated sensor may not always be positive when forward
    //      * because
    //      * sensor phase is only applied to *Selected Sensors*, not native
    //      * sensor sources. And we need the native to be combined with the
    //      * aux (other side's) distance into a single robot distance.
    //      */

    //     /*
    //      * THIS FUNCTION should not need to be modified.
    //      * This setup will work regardless of whether the master
    //      * is on the Right or Left side since it only deals with
    //      * distance magnitude.
    //      */

    //     /* Check if we're inverted */
    //     if (masterInvertType == TalonFXInvertType.Clockwise) {
    //         /*
    //          * If master is inverted, that means the integrated sensor
    //          * will be negative in the forward direction.
    //          * If master is inverted, the final sum/diff result will also be inverted.
    //          * This is how Talon FX corrects the sensor phase when inverting
    //          * the motor direction. This inversion applies to the *Selected Sensor*,
    //          * not the native value.
    //          * Will a sensor sum or difference give us a positive total magnitude?
    //          * Remember the Master is one side of your drivetrain distance and
    //          * Auxiliary is the other side's distance.
    //          * Phase | Term 0 | Term 1 | Result
    //          * Sum: -((-)Master + (+)Aux )| NOT OK, will cancel each other out
    //          * Diff: -((-)Master - (+)Aux )| OK - This is what we want, magnitude will be
    //          * correct and positive.
    //          * Diff: -((+)Aux - (-)Master)| NOT OK, magnitude will be correct but negative
    //          */

    //         masterConfig.diff0Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); // Local Integrated
    //                                                                                             // Sensor
    //         masterConfig.diff1Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice(); // Aux Selected Sensor
    //         masterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice(); // Diff0
    //                                                                                                                     // -
    //                                                                                                                     // Diff1
    //     } else {
    //         /* Master is not inverted, both sides are positive so we can sum them. */
    //         masterConfig.sum0Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice(); // Aux Selected Sensor
    //         masterConfig.sum1Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); // Local IntegratedSensor
    //         masterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorSum.toFeedbackDevice(); // Sum0
    //                                                                                                              // +
    //                                                                                                              // Sum1
    //     }

    //     /*
    //      * Since the Distance is the sum of the two sides, divide by 2 so the total
    //      * isn't double
    //      * the real-world value
    //      */
    //     masterConfig.primaryPID.selectedFeedbackCoefficient = 0.5;
    // }
}
