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
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
    
    import com.revrobotics.CANSparkMax;
    import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 *
 */
public class shooterSubSys extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private WPI_TalonSRX flywheelMotor;
private WPI_TalonSRX turretMotor;
private Encoder turretEncoder;
private DigitalInput turretRightLimitSwitch;
private DigitalInput turretLeftLimitSwitch;
private WPI_TalonSRX hoodMotor;
private Encoder hoodEncoder;
private DigitalInput hoodExtendLimitSwitch;
private DigitalInput hoodRetractLimitSwitch;
private PWMVictorSPX loadMotor;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private CANSparkMax shooterMotor;

    public shooterSubSys() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
flywheelMotor = new WPI_TalonSRX(10);


        
turretMotor = new WPI_TalonSRX(12);


        
turretEncoder = new Encoder(5, 6, false, EncodingType.k4X);
addChild("turretEncoder",turretEncoder);
turretEncoder.setDistancePerPulse(1.0);
turretEncoder.setPIDSourceType(PIDSourceType.kRate);
        
turretRightLimitSwitch = new DigitalInput(7);
addChild("turretRightLimitSwitch",turretRightLimitSwitch);

        
turretLeftLimitSwitch = new DigitalInput(8);
addChild("turretLeftLimitSwitch",turretLeftLimitSwitch);

        
hoodMotor = new WPI_TalonSRX(13);


        
hoodEncoder = new Encoder(9, 10, false, EncodingType.k4X);
addChild("HoodEncoder",hoodEncoder);
hoodEncoder.setDistancePerPulse(1.0);
hoodEncoder.setPIDSourceType(PIDSourceType.kRate);
        
hoodExtendLimitSwitch = new DigitalInput(12);
addChild("HoodExtendLimitSwitch",hoodExtendLimitSwitch);

        
hoodRetractLimitSwitch = new DigitalInput(13);
addChild("HoodRetractLimitSwitch",hoodRetractLimitSwitch);

        
loadMotor = new PWMVictorSPX(0);
addChild("loadMotor",loadMotor);
loadMotor.setInverted(false);
        

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    shooterMotor = new CANSparkMax(13, MotorType.kBrushless);
    }

    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        setDefaultCommand(new shooterTestMotorStopCmd());

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
public void shooterMotorOn(double speed) {
    shooterMotor.set(speed);
}
public void shooterMotorOff(){
    shooterMotor.set(0);
}
}

