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


import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class DriveTrainSubsystem extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private WPI_TalonFX leftMotor1;
private WPI_TalonFX leftMotor2;
private MotorControllerGroup leftDrive;
private WPI_TalonFX rightMotor1;
private WPI_TalonFX rightMotor2;
private MotorControllerGroup rightDrive;
private DifferentialDrive driveTrain;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    
    /**
    *
    */
    public DriveTrainSubsystem() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
leftMotor1 = new WPI_TalonFX(4);
 
 

leftMotor2 = new WPI_TalonFX(3);
 
 

leftDrive = new MotorControllerGroup(leftMotor1, leftMotor2  );
 addChild("LeftDrive",leftDrive);
 

rightMotor1 = new WPI_TalonFX(0);
 
 

rightMotor2 = new WPI_TalonFX(1);
 
 

rightDrive = new MotorControllerGroup(rightMotor1, rightMotor2  );
 addChild("RightDrive",rightDrive);
 

driveTrain = new DifferentialDrive(leftDrive, rightDrive);
 addChild("DriveTrain",driveTrain);
 driveTrain.setSafetyEnabled(true);
driveTrain.setExpiration(0.1);
driveTrain.setMaxOutput(1.0);



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

}

