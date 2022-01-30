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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TowerConstants;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 *
 */
public class TowerSubsystem extends SubsystemBase {

    private CANSparkMax transferWheel;

    /**
    *
    */
    public TowerSubsystem() {
        transferWheel = new CANSparkMax(TowerConstants.transferWheelID, MotorType.kBrushless);

        transferWheel.restoreFactoryDefaults();
        transferWheel.setInverted(false);
        transferWheel.setIdleMode(IdleMode.kBrake);

        SmartDashboard.putNumber("Transfer Wheel Speed", 0.8);
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

    public void setTransferWheel(double speed) {
        transferWheel.set(speed);
    }
}
