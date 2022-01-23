package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.PigeonIMU.GeneralStatus;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
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
    private PigeonIMU m_pigeonIMU;
    private boolean m_pigeonFailure = false;

    private final DifferentialDriveOdometry m_odometry;

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

        m_motorConfig = new TalonFXConfiguration();
        m_motorConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 35, 40, 0.2);
        m_motorConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        m_motorConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        m_motorConfig.remoteFilter0.remoteSensorDeviceID = m_leftEncoder.getDeviceID();
        m_motorConfig.remoteFilter1.remoteSensorSource = RemoteSensorSource.CANCoder;
        m_motorConfig.remoteFilter1.remoteSensorDeviceID = m_rightEncoder.getDeviceID();

        m_leftMotor1 = new WPI_TalonFX(DriveConstants.leftMotor1ID);
        m_leftMotor1.configFactoryDefault();
        m_leftMotor1.configAllSettings(m_motorConfig);
        m_leftMotor2 = new WPI_TalonFX(DriveConstants.leftMotor2ID);
        m_leftMotor2.configFactoryDefault();
        m_leftMotor2.configAllSettings(m_motorConfig);

        m_leftDrive = new MotorControllerGroup(m_leftMotor1, m_leftMotor2);
        addChild("LeftDrive", m_leftDrive);

        m_rightMotor1 = new WPI_TalonFX(DriveConstants.rightMotor1ID);
        m_rightMotor1.configFactoryDefault();
        m_rightMotor1.configAllSettings(m_motorConfig);
        m_rightMotor1.setInverted(true);
        m_rightMotor2 = new WPI_TalonFX(DriveConstants.rightMotor2ID);
        m_rightMotor2.configFactoryDefault();
        m_rightMotor2.configAllSettings(m_motorConfig);
        m_rightMotor2.setInverted(true);

        m_rightDrive = new MotorControllerGroup(m_rightMotor1, m_rightMotor2);
        addChild("RightDrive", m_rightDrive);

        m_drivetrain = new DifferentialDrive(m_leftDrive, m_rightDrive);
        // drivetrain = new DifferentialDrive(leftMotor2, rightMotor2);
        addChild("Drivetrain", m_drivetrain);

        m_drivetrain.setSafetyEnabled(true);
        m_drivetrain.setExpiration(0.1);
        m_drivetrain.setMaxOutput(1.0);

        // ADISIMU = new ADIS16470_IMU();
        m_pigeonIMU = new PigeonIMU(DriveConstants.pigeonID);

        m_pigeonIMU.setFusedHeading(0);

        m_odometry = new DifferentialDriveOdometry(
                new Rotation2d(getGyroscopeReadingRad()));
    }

    @Override
    public void periodic() {
        m_odometry.update(getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
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
    }

    /**
     * Allows the joystick to be used in a way people understand. Arcade style.
     * 
     * @param x The x-axis of the controller.
     * @param y The y-axis of the controller.
     */
    public void setArcadeDrive(double x, double y) {
        // The y is inverted so forward will be positive and backward will be negative.
        m_drivetrain.arcadeDrive(-y * DriveConstants.maxSpeed, x * DriveConstants.maxTurning);

        // YAAY my first code!!! I did something useful! -Helen
    }

    /**
     * Sets the Voltages of the left and right side independently
     * 
     * @param leftVolts  Left side Voltage
     * @param rightVolts Right side Voltage
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_leftDrive.setVoltage(leftVolts);
        m_rightDrive.setVoltage(rightVolts);
        m_drivetrain.feed();
    }

    // Encoders

    /**
     * Returns the left encoder absolute position.
     * 
     * @return The absolute position of the left encoder ranging from [0, 0.47877)
     *         in meters.
     */
    public double getLeftEncoderAbsolutePosition() {
        return m_leftEncoder.getAbsolutePosition();
    }

    /**
     * Returns the right encoder absolute position.
     * 
     * @return The absolute position of the right encoder ranging from [0, Math.PI
     * Units.inchesToMeters(6)) in meters.
     */
    public double getRightEncoderAbsolutePosition() {
        return m_rightEncoder.getAbsolutePosition();
    }

    /**
     * Returns the left encoder position.
     * 
     * @return The position of the right side of the drivetrain in meters.
     */
    public double getLeftEncoderPosition() {
        return m_leftEncoder.getPosition();
    }

    /**
     * Returns the right encoder position.
     * 
     * @return The position of the right side of the drivetrain in meters.
     */
    public double getRightEncoderPosition() {
        return m_rightEncoder.getPosition();
    }

    /**
     * Sets the position of the left encoder.
     * 
     * @param position The position of the left side of the drivetrain in meters.
     */
    public void setLeftEncoderPostion(double position) {
        m_leftEncoder.setPosition(position);
    }

    /**
     * Sets the position of the right encoder.
     * 
     * @param position The position of the right side of the drivetrain in meters.
     */
    public void setRightEncoderPostion(double position) {
        m_rightEncoder.setPosition(position);
    }

    /**
     * Sets the position of both the left and right side encoders at the same time.
     * 
     * @param position The position of the two sides of the drivetrain in meters.
     */
    public void setEncoderPostion(double position) {
        m_leftEncoder.setPosition(position);
        m_rightEncoder.setPosition(position);
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
    }

    /**
     * Return the Velocity of the right Encoder
     */
    public double getRightVelocity() {
        return m_rightEncoder.getVelocity();
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
     * Check to see if our pigeon has disconnected. Useful for switching to a backup
     * IMU if present.
     * 
     * @return True if the pigeon has failed, false otherwise.
     */
    private boolean checkForPigeonFailure() {
        GeneralStatus status = new GeneralStatus();
        m_pigeonIMU.getGeneralStatus(status);
        if (status.state == PigeonState.NoComm || status.state == PigeonState.Unknown) {
            m_pigeonFailure = true;
            return true;
        } else {
            return false;
        }
    }

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
        if (!m_pigeonFailure && !checkForPigeonFailure()) {
            if (ccwPositive) {
                return m_pigeonIMU.getFusedHeading();
            } else {
                return -m_pigeonIMU.getFusedHeading();
            }
        } else {
            // Can't use these due to Analog Devices gyro bug.
            /*
             * double uncorrectedHeading = ADISIMU.getAngle();
             * if (ccwPositive) {
             * return uncorrectedHeading;
             * } else {
             * return -uncorrectedHeading;
             * }
             */
            return 0;
        }
    }

    /**
     * Returns the robot's heading, with a range of [-180, 180], with
     * counterclockwise being positive.
     * 
     * @return The rotation of the robot in degrees, with counterclockwise rotations
     *         increasing this value.
     */
    public double getGyroscopeReading() {
        return getGyroscopeReading(true);
    }

    /**
     * Returns the robot's heading, with a range of [-180, 180].
     * 
     * @param ccwPositive Whether the reading should be returned treating
     *                    counterclockwise as positive or negative, with true
     *                    meaning ccw is positive.
     * @return The rotation of the robot in degrees.
     */
    public double getGyroscopeReading(boolean ccwPositive) {
        double continuousHeading = getGyroscopeReadingContinuous(ccwPositive);

        /**
         * Corrects the values to the range of [-180, 180]
         * 
         * First the value is caped to [-360, 360],
         * than if the value is outside of [-180, 180]
         * 360 is added or subtracted as nesasary to get within the desiered range
         */
        double correctedValue = continuousHeading % 360;

        if (correctedValue > 180) {
            return correctedValue - 360;
        } else if (correctedValue < -180) {
            return correctedValue + 360;
        } else {
            return correctedValue;
        }
    }

    /**
     * returns the rotation of the gyrosope in Radians
     */
    public double getGyroscopeReadingRad() {
        return Units.degreesToRadians(getGyroscopeReading());
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
        return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
    }
}
