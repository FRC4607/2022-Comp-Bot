// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be
 * declared globally (i.e. public static). Do not put anything functional in
 * this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {
    /**
     * public static final class DriveConstants {
     * public static final int kLeftMotor1Port = 0;
     * public static final int kLeftMotor2Port = 1;
     * public static final int kRightMotor1Port = 2;
     * public static final int kRightMotor2Port = 3;
     * }
     */

    public static final class DriveConstants {
        public static final int leftMotor1ID = 0;
        public static final int leftMotor2ID = 1;
        public static final int rightMotor1ID = 3;
        public static final int rightMotor2ID = 4;
        public static final int leftEncoderID = 2;
        public static final int rightEncoderID = 5;
        public static final int pigeonID = 6;
        //public static final int countsPerRevolution = 2048;
        public static final double sensorCoefficient = (Math.PI * Units.inchesToMeters(6)) / 4096; // 0.478777
        public static final double ks_Volts = 0.65288; // [0.58644, 0.5908, 0.65852, 0.65288]
        public static final double kv_VoltSecondsPerMeters = 2.5016; // [2.5331, 2.5377, 2.4524, 2.5016]
        public static final double ka_VoltSecondsSquaredPerMeters = 0.38331; // [0.20049, 0.25145, 0.42348, 0.38331]
        public static final double trackWidth_Meters = 0.7036911491; // Theoretical
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            trackWidth_Meters);
            public static final double maxVoltage = 10;
            public static final double maxUnitsPerSecond = 2; // Slue Rate
            public static final double maxSpeed = 1;
            public static final double maxTurning = 0.8;
            
            // PID drive
            public static final double kPDriveVel = 2.9025 * 2; // 2.9445, 2.9025
            
            // Ramset
            public static final double kRamseteB_radSquaredPerMetersSquared = 2;
            public static final double kRamseteZeta_PerRad = 1;
            
            // Conversons
            public static final double gearing = 4.0 / 45.0;
            public static final double MeterPerFalconCount = gearing / 2048 * (Math.PI * Units.inchesToMeters(6));
            public static double maxTurnPerSecond = 2;
    }

    public static final class IntakeConstants {
        public static final int motorID = 7;
        
        public static final int solenoidChannel = 14;

        public static final double kP = 0.0;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kS = 0.24302;
        public static final double kV = 0.12861;

        public static final double intakeingRPM = 3500;
        public static final double releasingRPM = 1000;

        public static final double intakeSpeed = 0.80;

    }

    public static final class TowerConstants {
        public static final int agitatiorID = 9;
        public static final double agitatiorSpeed = 0.75;

        public static final int midBrakeBeamID = 0;
        public static final int highBrakeBeamID = 1;
    }

    public static final class ShooterConstants {
        public static final int flywheelMotor1ID = 11;
        public static final int flywheelMotor2ID = 12;

        // This is calculated from the characterization tool's CTRE preset.
        //public static final double flywheelP =  0.15;// 0.17121;
        public static final double flywheelF =  0.05;
        public static final double flywheelP =  0.08;
        public static final double flywheelD = 5;

        // kS from characterization tool, used here as an arbitrary feed forward to
        // overcome friction. This is divided by 12 to get the value as a percent of the
        // motor's max output.
        public static final double flywheelKs = 0.7983;
        // kA from characterization tool, used here as the velocity feed forward.
        // Conversion from v / (rotation/sec) to CTRE units (v (0-1023) / (rotations /
        // 0.1sec) ) is needed.
        // public static final double flywheelKf = 0.05103092783505154639175257731959;
        public static final double flywheelKv = 0.11287;

        public static final double flywheelMaxError = 200; // 50
        
        public static final double lowGoalRPM = 1000;
        public static final double highGoalRPM = 2450;
        
        public static final int kickerWheelID = 10;
        public static final double kickerWheelSpeed = 1;

        public static final int pistionChannel = 15;
        public static final double limeLightRPM = 2725;
        
    }

    public static final class FollowPathConstants {
        public static final double kMaxSpeed_MetersPerSecond = 3;
        public static final double kMaxAcceleration_MetersPerSecondSquared = 3.5;

        public static final DifferentialDriveVoltageConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(DriveConstants.ks_Volts, DriveConstants.kv_VoltSecondsPerMeters,
                        DriveConstants.ka_VoltSecondsSquaredPerMeters),
                DriveConstants.kDriveKinematics, DriveConstants.maxVoltage);

        public static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                FollowPathConstants.kMaxSpeed_MetersPerSecond,
                FollowPathConstants.kMaxAcceleration_MetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(DriveConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(voltageConstraint);
    }

    public static final class ClimberConstants {
        public static final int motor1ID = 17;
        public static final int motor2ID = 16;

        public static final double kP = 0.5;
        public static final double kI = 0.1;
        public static final double kD = 0;

        public static final double maxHight_Rotations = 82; // 6.9;
        public static final double rotationToExtend = 0;
        public static final int limitSwitchID = 2;
        public static final double conversenFactor_SensorUnitsPerInch = 8192;

        public static final double maxRoatation = 90;
        public static final double minRoatation = -45;
        
        public static final int pistionChannel = 13;

        public static final double maxVelocity = 200;
        public static final double maxAcceleration = 200;

        public static final double PositonTolerace = 0.05;

        public static final double pistionEstenchonTime = 5.0;
    }

    public static final int pnumaticHub = 8;
    public static final int PDH = 13;
}
