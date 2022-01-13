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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.FilterConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class DrivetrainSubsystem extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // _BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private CANCoder leftEncoder;
    private CANCoder rightEncoder;
    private WPI_TalonFX leftMotor1;
    private WPI_TalonFX leftMotor2;
    private MotorControllerGroup leftDrive;
    private WPI_TalonFX rightMotor1;
    private WPI_TalonFX rightMotor2;
    private MotorControllerGroup rightDrive;
    private DifferentialDrive drivetrain;
    private ADIS16470_IMU gyro;
    private final CANCoderConfiguration encoderConfig;
    private final TalonFXConfiguration motorConfig;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    /**
    *
    */
    public DrivetrainSubsystem() {
        encoderConfig = new CANCoderConfiguration();
        encoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        encoderConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

        // _BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        leftEncoder = new CANCoder(2);
        leftEncoder.configFactoryDefault();
        leftEncoder.configAllSettings(encoderConfig);

        rightEncoder = new CANCoder(5);
        rightEncoder.configFactoryDefault();
        rightEncoder.configAllSettings(encoderConfig);

        motorConfig = new TalonFXConfiguration();
        motorConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 35, 40, 0.2);
        motorConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        motorConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        motorConfig.remoteFilter0.remoteSensorDeviceID = leftEncoder.getDeviceID();
        motorConfig.remoteFilter1.remoteSensorSource = RemoteSensorSource.CANCoder;
        motorConfig.remoteFilter1.remoteSensorDeviceID = rightEncoder.getDeviceID();

        leftMotor1 = new WPI_TalonFX(0);
        leftMotor1.configFactoryDefault();
        leftMotor1.configAllSettings(motorConfig);
        leftMotor2 = new WPI_TalonFX(1);
        leftMotor2.configFactoryDefault();
        leftMotor2.configAllSettings(motorConfig);

        leftDrive = new MotorControllerGroup(leftMotor1, leftMotor2);
        addChild("LeftDrive", leftDrive);

        rightMotor1 = new WPI_TalonFX(3);
        rightMotor1.configFactoryDefault();
        rightMotor1.configAllSettings(motorConfig);
        rightMotor2 = new WPI_TalonFX(4);
        rightMotor2.configFactoryDefault();
        rightMotor2.configAllSettings(motorConfig);

        rightDrive = new MotorControllerGroup(rightMotor1, rightMotor2);
        addChild("RightDrive", rightDrive);

        drivetrain = new DifferentialDrive(leftDrive, rightDrive);
        addChild("Drivetrain", drivetrain);

        drivetrain.setSafetyEnabled(true);
        drivetrain.setExpiration(0.1);
        drivetrain.setMaxOutput(1.0);

        gyro = new ADIS16470_IMU();

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    /**
     * Sets the voltage of the left side motors
     */
    public void setLeftVoltage(double voltage) {
        leftDrive.setVoltage(voltage);
    }

    /**
     * Sets the voltage of the right side motors
     */
    public void setRightVoltage(double voltage) {
        rightDrive.setVoltage(voltage);
    }

    /**
     * Sets the voltage of both the left and right side motors
     */
    public void setVoltage(double voltage) {
        leftDrive.setVoltage(voltage);
        rightDrive.setVoltage(voltage);
    }

    /**
     * Returns the left encoder absolute position
     */
    public double getLeftEncoderAbsolutePosition() {
        return leftEncoder.getAbsolutePosition();
    }

    /**
     * Returns the left encoder absolute position
     */
    public double getRightEncoderAbsolutePosition() {
        return rightEncoder.getAbsolutePosition();
    }

    /**
     * Returns the left encoder position
     */
    public double getLeftEncoderPosition() {
        return leftEncoder.getPosition();
    }

    /**
     * Returns the right encoder position
     */
    public double getRightEncoderPosition() {
        return rightEncoder.getPosition();
    }

    /**
     * Sets the position of the left encoder
     */
    public void setLeftEncoderPostion(double position) {
        leftEncoder.setPosition(position);
    }

    /**
     * Sets the position of the right encoder
     */
    public void setRightEncoderPostion(double position) {
        rightEncoder.setPosition(position);
    }

    /**
     * Sets the position of both the left and right side encoders
     */
    public void setEncoderPostion(double position) {
        leftEncoder.setPosition(position);
        rightEncoder.setPosition(position);
    }

    /**
     * Allows the joystick to be used in a way people understand. Arcade style.
     * 
     * @param x The x-axis of the controller.
     * @param y The y-axis of the controller.
     */
    public void setArcadeDrive(double x, double y) {
        // The y is inverted so forward will be positive and backward will be negative.
        drivetrain.arcadeDrive(-y, x, true);

        // YAAY my first code!!! I did something useful! -Helen
    }

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
     * Returns the robot's heading, ranging from -inf to inf.
     * 
     * @param ccwPositive Whether the reading should be returned treating
     *                    counterclockwise as positive or negative, with true
     *                    meaning ccw is positive.
     * @return The rotation of the robot in degrees, with counterclockwise rotations
     *         increasing this value.
     */
    public double getGyroscopeReadingContinuous(boolean ccwPositive) {
        double uncorrectedHeading = gyro.getAngle();
        if (ccwPositive) {
            return uncorrectedHeading;
        } else {
            return -uncorrectedHeading;
        }
    }

    /**
     * Returns the robot's heading, with a range of [-180, 180].
     * 
     * @param ccwPositive Whether the reading should be returned treating
     *                    counterclockwise as positive or negative, with true
     *                    meaning ccw is positive.
     * @return The rotation of the robot in degrees, with counterclockwise rotations
     *         increasing this value.
     */
    public double getGyroscopeReading(boolean ccwPositive) {
        double continuousHeading = gyro.getAngle();

        if (!ccwPositive) {
            continuousHeading = -continuousHeading;
        }

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
     * Gets the x acceleration of the robot in m/s^2 (i think). You can check which
     * axis is which direction and sign by looking at the graphic on the roborio.
     * 
     * @return The acceleration on the x axis.
     */
    public double getXAccel() {
        return gyro.getAccelX();
    }

    /**
     * Gets the y acceleration of the robot in m/s^2 (i think). You can check which
     * axis is which direction and sign by looking at the graphic on the roborio.
     * 
     * @return The acceleration on the y axis.
     */
    public double getYAccel() {
        return gyro.getAccelY();
    }

    /**
     * Gets the z acceleration of the robot in m/s^2 (i think). You can check which
     * axis is which direction and sign by looking at the graphic on the roborio.
     * 
     * @return The acceleration on the z axis.
     */
    public double getZAccel() {
        return gyro.getAccelZ();
    }
}
