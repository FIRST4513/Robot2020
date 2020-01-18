// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package robot.subsystems;


import robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class storageSubSys extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private WPI_TalonSRX storageBottomRollerMotor;
private WPI_TalonSRX storageEjectMotor;
private DigitalInput storageBallSensor1;
private DigitalInput storageBallSensor2;
private DigitalInput storageBallSensor3;
private DigitalInput storageBallSensor4;
private DigitalInput storageBallSensor5;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public storageSubSys() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
storageBottomRollerMotor = new WPI_TalonSRX(0);


        
storageEjectMotor = new WPI_TalonSRX(3);


        
storageBallSensor1 = new DigitalInput(0);
addChild("storageBallSensor1",storageBallSensor1);

        
storageBallSensor2 = new DigitalInput(1);
addChild("storageBallSensor2",storageBallSensor2);

        
storageBallSensor3 = new DigitalInput(2);
addChild("storageBallSensor3",storageBallSensor3);

        
storageBallSensor4 = new DigitalInput(3);
addChild("storageBallSensor4",storageBallSensor4);

        
storageBallSensor5 = new DigitalInput(4);
addChild("storageBallSensor5",storageBallSensor5);

        

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        setDefaultCommand(new storageHoldCmd());

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop

    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    
    // Ball Storage Sensor Checks

    public boolean getBallSensorOne() {
        return storageBallSensor1.get();
    }
    public boolean getBallSensorTwo() {
        return storageBallSensor2.get();
    }
    public boolean getBallSensorThree() {
        return storageBallSensor3.get();
    }
    public boolean getBallSensorFour() {
        return storageBallSensor4.get();
    }
    public boolean getBallSensorFive() {
        return storageBallSensor5.get();
    }

    // Set Motor Methods
    
    public void moveBottomRollers(double power) {
        storageBottomRollerMotor.set(power);
    }

    public void moveEjectMotor(double power) {
        storageEjectMotor.set(power);
    }
}

