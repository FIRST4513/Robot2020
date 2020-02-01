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
    private double handoffPower = 0;
    private double turretPower = 0;

    private double flywheel_P;
    private double flywheel_I;
    private double flywheel_D;
    private double flywheel_Iz;
    private double flywheel_FF;
    private double flywheelMinOutput;
    private double flywheelMaxOutput;
    private double flywheelSetPoint = 0;


    private double previousFlywheel_P, previousFlywheel_I, previousFlywheel_D, previousFlywheel_Iz, previousFlywheel_FF, previousFlywheelMinOutput, previousFlywheelMaxOutput;

    private static final double HoodConversion = 0.3; //Need tuning!
    private static final double TurretConversion = 0.3; //Need tuning!

    private static final double HandoffEjectionPower = 0.7; //Need tuning!

    private static final double DEFAULT_FLYWHEEL_P = 5e-5; 
    private static final double DEFAULT_FLYWHEEL_I = 1e-6;
    private static final double DEFAULT_FLYWHEEL_D = 0; 
    private static final double DEFAULT_FLYWHEEL_IZ = 0; 
    private static final double DEFAULT_FLYWHEEL_FF = 0; 
    private static final double DEFAULT_FLYWHEEL_MAX_OUTPUT = 1; 
    private static final double DEFAULT_FLYWHEEL_MIN_OUTPUT = -1;

    private static final double MAX_SHOOTER_SPEED = 5200; // This is in RPM

    private static final int FLYWHEEL_RIGHT_CAN_ID = 18;
    private static final int FLYWHEEL_LEFT_CAN_ID = 19;

    private CANSparkMax flywheelLeftMotor;
    private CANSparkMax flywheelRightMotor;
    private CANPIDController flywheelPidController;
    private CANEncoder flywheelEncoder;

    public static enum TurretSwitchPressed{LEFT, RIGHT, NEITHER};

    public static enum HoodSwitchPressed{EXTENDED, RETRACTED, NEITHER};

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
       flywheelLeftMotor = new CANSparkMax(FLYWHEEL_LEFT_CAN_ID, MotorType.kBrushless);
       flywheelRightMotor = new CANSparkMax(FLYWHEEL_RIGHT_CAN_ID, MotorType.kBrushless);

        flywheelLeftMotor.restoreFactoryDefaults();
        flywheelRightMotor.restoreFactoryDefaults();
    
        // Encoder object created to display position values
        flywheelEncoder = flywheelRightMotor.getEncoder();
    
        // set PID coefficients
        factoryResetFlywheelPid();

        // set set point to 0
        flywheelPidController.setReference(0, ControlType.kVelocity);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("Flywheel P Gain", flywheel_P);
        SmartDashboard.putNumber("Flywheel I Gain", flywheel_I);
        SmartDashboard.putNumber("Flywheel D Gain", flywheel_D);
        SmartDashboard.putNumber("Flywheel I Zone", flywheel_Iz);
        SmartDashboard.putNumber("Flywheel Feed Forward", flywheel_FF);
        SmartDashboard.putNumber("Flywheel Max Output", flywheelMaxOutput);
        SmartDashboard.putNumber("Flywheel Min Output", flywheelMinOutput);
        SmartDashboard.putNumber("Flywheel Target RPM", flywheelSetPoint);
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
        getFlywheelPid();
        updateFlywheelPid();

    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    /*     Flywheel PID Methods     */
    public void factoryResetFlywheelPid () {
        flywheel_P = DEFAULT_FLYWHEEL_P;
        flywheel_I = DEFAULT_FLYWHEEL_I;
        flywheel_D = DEFAULT_FLYWHEEL_D;
        flywheel_Iz = DEFAULT_FLYWHEEL_IZ;
        flywheel_FF = DEFAULT_FLYWHEEL_FF;
        flywheelMinOutput = DEFAULT_FLYWHEEL_MIN_OUTPUT;
        flywheelMaxOutput = DEFAULT_FLYWHEEL_MAX_OUTPUT;
    }

    public void getFlywheelPid() {
        flywheel_P = SmartDashboard.getNumber("Flywheel P Gain", DEFAULT_FLYWHEEL_P);
        flywheel_I = SmartDashboard.getNumber("Flywheel I Gain", DEFAULT_FLYWHEEL_I);
        flywheel_D = SmartDashboard.getNumber("Flywheel D Gain", DEFAULT_FLYWHEEL_D);
        flywheel_Iz = SmartDashboard.getNumber("Flywheel I Zone", DEFAULT_FLYWHEEL_IZ);
        flywheel_FF = SmartDashboard.getNumber("Flywheel Feed Forward", DEFAULT_FLYWHEEL_FF);
        flywheelMinOutput = SmartDashboard.getNumber("Flywheel Max Output", DEFAULT_FLYWHEEL_MAX_OUTPUT);
        flywheelMaxOutput = SmartDashboard.getNumber("Flywheel Min Output", DEFAULT_FLYWHEEL_MIN_OUTPUT);
    }

    public void updateFlywheelPid() {
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((flywheel_P != previousFlywheel_P)) { 
            flywheelPidController.setP(flywheel_P); 
            previousFlywheel_P = flywheel_P;
            SmartDashboard.putNumber("Flywheel P Gain", flywheel_P);
        }

        if((flywheel_I != previousFlywheel_I)) { 
            flywheelPidController.setI(flywheel_I); 
            previousFlywheel_I = flywheel_I;
            SmartDashboard.putNumber("Flywheel I Gain", flywheel_I);
        }

        if((flywheel_D != previousFlywheel_D)) { 
            flywheelPidController.setD(flywheel_D); 
            previousFlywheel_D = flywheel_D;
            SmartDashboard.putNumber("Flywheel D Gain", flywheel_D);
        }

        if((flywheel_Iz != previousFlywheel_Iz)) { 
            flywheelPidController.setIZone(flywheel_Iz); 
            previousFlywheel_Iz = flywheel_Iz;
            SmartDashboard.putNumber("Flywheel I Zone", flywheel_Iz);
        }

        if((flywheel_FF != previousFlywheel_FF)) { 
            flywheelPidController.setFF(flywheel_FF); 
            previousFlywheel_FF = flywheel_FF;
            SmartDashboard.putNumber("Flywheel Feed Forward", flywheel_FF);
        }
        
        if((flywheelMaxOutput != previousFlywheelMaxOutput) || (flywheelMinOutput != previousFlywheelMinOutput)) { 
            flywheelPidController.setOutputRange(flywheelMinOutput, flywheelMaxOutput);
            previousFlywheelMinOutput = flywheelMinOutput; previousFlywheelMaxOutput = flywheelMaxOutput;
            SmartDashboard.putNumber("Flywheel Max Output", flywheelMaxOutput);
            SmartDashboard.putNumber("Flywheel Min Output", flywheelMinOutput);
        }
    }

    public void updateFlywheelSetPoint(double setPoint) {
        if((setPoint != flywheelSetPoint)) { 
            flywheelPidController.setReference(setPoint, ControlType.kVelocity); 
            flywheelSetPoint = setPoint;
            SmartDashboard.putNumber("Flywheel Target RPM", flywheelSetPoint);
        }
    }

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
        updateFlywheelSetPoint(0);
    }

    public double getFlywheelEncoder() {
        return flywheelEncoder.getPosition();
    }

    public void moveFlywheels(double power) {
        flywheelLeftMotor.set(power);
        flywheelRightMotor.set(power);
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

    SmartDashboard.putNumber("Flywheel Set Point", flywheelSetPoint);
    SmartDashboard.putNumber("Turret Power", turretPower);

    SmartDashboard.putNumber("Handoff Power", handoffPower);

    SmartDashboard.putNumber("Flywheel Raw Encoder", flywheelEncoder.getPosition());
    SmartDashboard.putNumber("Flywheel RPM", flywheelEncoder.getVelocity());
    }
}
