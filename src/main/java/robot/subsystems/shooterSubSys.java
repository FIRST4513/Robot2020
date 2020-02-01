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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.MotorCommutation;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
    

    import com.revrobotics.CANEncoder;
    import com.revrobotics.CANPIDController;
    import com.revrobotics.CANSparkMax;
    import com.revrobotics.ControlType;
    import com.revrobotics.CANSparkMaxLowLevel.MotorType;
/**
 *
 */
public class shooterSubSys extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
    private static final double HoodConversion = 0.3; //Need tuning!
    private static final double TurretConversion = 0.3; //Need tuning!

    private static final double HandoffEjectionPower = 0.7; //Need tuning!

    private static final int deviceID = 13;
    private CANSparkMax flywheelLeftMotor;
    private CANSparkMax flywheelRightMotor;
    private CANPIDController flywheelPidController;
    private CANEncoder flywheelEncoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private WPI_TalonSRX turretMotor;
private Encoder turretEncoder;
private DigitalInput turretRightLimitSwitch;
private DigitalInput turretLeftLimitSwitch;
private Encoder hoodEncoder;
private WPI_TalonSRX hoodMotor;
private DigitalInput hoodExtendLimitSwitch;
private DigitalInput hoodRetractLimitSwitch;
private WPI_TalonSRX handoffMotor;

public static enum TurretSwitchPressed {
    NEITHER, LEFT, RIGHT
}

public static enum HoodSwitchPressed {
    NEITHER, RETRACTED, EXTENDED
}

private double flywheelPower = 0;
private double turretPower = 0;
private double handoffPower = 0;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
   

    public shooterSubSys() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
turretMotor = new WPI_TalonSRX(12);


        
turretEncoder = new Encoder(5, 6, false, EncodingType.k4X);
addChild("turretEncoder",turretEncoder);
turretEncoder.setDistancePerPulse(1.0);
turretEncoder.setPIDSourceType(PIDSourceType.kRate);
        
turretRightLimitSwitch = new DigitalInput(7);
addChild("turretRightLimitSwitch",turretRightLimitSwitch);

        
turretLeftLimitSwitch = new DigitalInput(8);
addChild("turretLeftLimitSwitch",turretLeftLimitSwitch);

        
hoodEncoder = new Encoder(9, 10, false, EncodingType.k4X);
addChild("HoodEncoder",hoodEncoder);
hoodEncoder.setDistancePerPulse(1.0);
hoodEncoder.setPIDSourceType(PIDSourceType.kRate);
        
hoodMotor = new WPI_TalonSRX(13);


        
hoodExtendLimitSwitch = new DigitalInput(12);
addChild("HoodExtendLimitSwitch",hoodExtendLimitSwitch);

        
hoodRetractLimitSwitch = new DigitalInput(13);
addChild("HoodRetractLimitSwitch",hoodRetractLimitSwitch);

        
handoffMotor = new WPI_TalonSRX(6);


        

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS


    

    // initialize motors
       flywheelLeftMotor = new CANSparkMax(deviceID, MotorType.kBrushless);
       flywheelRightMotor = new CANSparkMax(deviceID, MotorType.kBrushless);
       
        /**
         * The RestoreFactoryDefaults method can be used to reset the configuration parameters
         * in the SPARK MAX to their factory default state. If no argument is passed, these
         * parameters will not persist between power cycles
         */
        flywheelLeftMotor.restoreFactoryDefaults();
        flywheelRightMotor.restoreFactoryDefaults();
    
        /**
         * In order to use PID functionality for a controller, a CANPIDController object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        //m_pidController = shooterMotor.getPIDController();
    
        // Encoder object created to display position values
        flywheelEncoder = flywheelRightMotor.getEncoder();

    
        // PID coefficients
        kP = 5e-5; 
        kI = 1e-6;
        kD = 0; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5700;
    
        // set PID coefficients
        //m_pidController.setP(kP);
        //m_pidController.setI(kI);
        //m_pidController.setD(kD);
        //m_pidController.setIZone(kIz);
        //m_pidController.setFF(kFF);
        //m_pidController.setOutputRange(kMinOutput, kMaxOutput);

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
        updateSmartdash();

    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    /*     Misc. Methods     */

    public void resetEncoders() {
        //FlywheelEncoder reset here.
        turretEncoder.reset();
        hoodEncoder.reset();
    }

    public void resetTurretEncoder() {
        turretEncoder.reset();
    }

    public void resetHoodEncoder() {
        hoodEncoder.reset();
    }

    /*     Handoff Methods     */

    public void stopHandoff() {
        handoffMotor.set(0);
        handoffPower = 0;
    }

    public void startHandoff() {
        handoffMotor.set(HandoffEjectionPower);
        handoffPower = HandoffEjectionPower;
    }

    /*     Flywheel Methods     */

    public void stopFlywheels() {
        flywheelLeftMotor.set(0);
        flywheelRightMotor.set(0);
        flywheelPower = 0;
    }

    public double getFlywheelEncoder() {
        return flywheelEncoder.getPosition();
    }

    public void moveFlywheels(double power) {
        flywheelLeftMotor.set(power);
        flywheelRightMotor.set(power);

        flywheelPower = power;
    }

    /*     Turret Methods     */
        
    public void stopTurret() {
        turretMotor.set(0);
        turretPower = 0; 
    }

    public int getTurretEncoder() {
        return turretEncoder.get();
    }

    public double getTurretAngle() {
         // Turret conversion not tuned.
        return getTurretEncoder() * TurretConversion;
    }


    public TurretSwitchPressed checkTurretLimitSwitches() {
        TurretSwitchPressed whichLimitSwitchPressed = TurretSwitchPressed.NEITHER; // If variable isn't updated we know neither limit switch is pressed.
   
        if (turretLeftLimitSwitch.get() == true) {
            whichLimitSwitchPressed = TurretSwitchPressed.LEFT;
        } else if (turretRightLimitSwitch.get() == true) {
            whichLimitSwitchPressed = TurretSwitchPressed.RIGHT;
        }

        return whichLimitSwitchPressed;
    }

    public void moveTurret(double power) {
        switch(checkTurretLimitSwitches()) {
            case LEFT:
                if (power <= 0) {
                    stopTurret();
                    resetTurretEncoder();
                }
            case RIGHT:
                if (power >= 0) {
                    stopTurret();
                }
            // Assuming right is posiive and left is negative.
            // We want to be able to move away from the limit switch.

            case NEITHER:
                // No limit switches pressed, so give turret power.
                turretMotor.set(power);
                turretPower = power;
        }
    }

    /*     Hood Methods     */

    public void stopHood() {
        hoodMotor.set(0); 
    }

    public HoodSwitchPressed checkHoodLimitSwitches() {
        HoodSwitchPressed whichLimitSwitchPressed = HoodSwitchPressed.NEITHER; // If variable isn't updated we know neither limit switch is pressed.

        if (hoodRetractLimitSwitch.get() == true) {
            whichLimitSwitchPressed = HoodSwitchPressed.RETRACTED;
        } else if (hoodExtendLimitSwitch.get() == true) {
            whichLimitSwitchPressed = HoodSwitchPressed.EXTENDED;
        }

        return whichLimitSwitchPressed;
    }

    public int getHoodEncoder() {
        return hoodEncoder.get();
    }

    public double getHoodAngle() {
        // HoodConversion isn't tuned. 
        return getHoodEncoder() * HoodConversion;
    }

    public void moveHood(double power) {
        switch(checkHoodLimitSwitches()) {
            case RETRACTED:
                if (power <= 0) {
                    stopHood();
                    resetHoodEncoder();
                }
            case EXTENDED:
                if (power >= 0) {
                    stopHood();
                }
            // Assuming extending is posiive and retracting is negative.
            // We want to be able to move away from the limit switch.

            case NEITHER:
                // No limit switches pressed, so give hood power.
                hoodMotor.set(power);
        }
    }

private void updateSmartdash(){
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

    switch(checkHoodLimitSwitches()) {
        case RETRACTED:
            SmartDashboard.putString("Hood Limit Switch Pressed", "Retracted");
            break;
        case EXTENDED:
            SmartDashboard.putString("Hood Limit Switch Pressed", "Extended");
            break;
        case NEITHER:
            SmartDashboard.putString("Hood Limit Switch Pressed", "Neither");
            break;
    }

    switch(checkTurretLimitSwitches()) {
        case LEFT:
            SmartDashboard.putString("Turret Limit Switch Pressed", "Left");
            break;
        case RIGHT:
            SmartDashboard.putString("Turret Limit Switch Pressed", "Right");
            break;
        case NEITHER:
            SmartDashboard.putString("Turret Limit Switch Pressed", "Neither");
            break;
    }

    SmartDashboard.putNumber("Turret Angle", getTurretAngle());
    SmartDashboard.putNumber("Hood Angle", getHoodAngle());
    
    SmartDashboard.putNumber("Turret Raw Encoder", getTurretEncoder());
    SmartDashboard.putNumber("Hood Raw Encoder", getHoodEncoder());

    SmartDashboard.putNumber("Flywheel Power", flywheelPower);
    SmartDashboard.putNumber("Turret Power", turretPower);

    SmartDashboard.putNumber("Handoff Power", handoffPower);

    SmartDashboard.putNumber("Flywheel Raw Encoder", flywheelEncoder.getPosition());
    SmartDashboard.putNumber("Flywheel RPM", flywheelEncoder.getVelocity());
    }
}

