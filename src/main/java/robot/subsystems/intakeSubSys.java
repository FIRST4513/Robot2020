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


//import robot.commands.*;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Subsystem;
//import edu.wpi.first.wpilibj.PIDOutput;
//import edu.wpi.first.wpilibj.PIDSource;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Solenoid;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


public class intakeSubSys extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    public static enum IntakeSmardashState {UPDATE, NOUPDATE};
    public static IntakeSmardashState intakeSmardashState = IntakeSmardashState.NOUPDATE;

    private final static double ROLLERSTOPPEDSPEED = 0.0;
    private final static double ROLLEREJECTSPEED = -0.5;
    private final static double ROLLERRETRACTSPEED = 0.65;

    // ---------- Intake Motor variables ------------
    public enum MixerByIntakeState {FEED, STOP}
    public MixerByIntakeState mixerByIntakeState = MixerByIntakeState.STOP;
    private static final double  MIXER_INTAKE_FEED_SPEED = -0.5;

    private enum IntakeDelayState {WAITING, DONE}
    private IntakeDelayState intakeDelayState = IntakeDelayState.DONE;
    private Timer intakeRollerTimer;
    //private static final double  INTAKE_OFF_DELAY = 3.0;  // delay stop rollers on retract
    
    // --------- Mixer by Shooter Motor variables ------
    public enum MixerByShooterState {FEED, STOP, EJECT};
    public MixerByShooterState mixerByShooterState = MixerByShooterState.STOP;
    private static final double MIXER_SHOOTER_FEED_SPEED = 0.5;
    private static final double MIXER_SHOOTER_CLEAR_SPEED = -0.5;
    private static final double MIXER_MOTOR_DELAY_TIME = 0.25;

    private enum MixerMotorDelayState { OFF, RUNNING, WAITING}
    private MixerMotorDelayState mixerMotorDelayState = MixerMotorDelayState.OFF;
    private static Timer mixerMotorTimer;

    // --------- Colorwheel Motor variables ------
    public static enum MixerByColorwheelState {STOP, COLORSPEED, ROTATESPEED};
    private MixerByColorwheelState mixerByColorwheelState = MixerByColorwheelState.STOP;
    private final double COLORWHEEL_ROTATE_SPEED = 0.7;
    private final double COLORWHEEL_COLOR_SPEED = 0.2;
    private double colorWheelSpeed = 0;

    private static final boolean VALVE_EXTEND_STATE = true;
    private static final boolean VALVE_RETRACT_STATE = false;

    public enum RollerState {RETRACT, EJECT, STOPPED} ;
    private RollerState rollerState = RollerState.STOPPED;


    public enum IntakeValveState {EXTENDED, RETRACTED} ;
    private IntakeValveState  intakeValveState = IntakeValveState.RETRACTED; 

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private WPI_VictorSPX intakeRollerMtr;
private Solenoid intakeExtentionValve;
private WPI_VictorSPX mixerMotor;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    private double displayCnt = 2;
    //private boolean intakeExtended = false;
    private double mixerPower = 0;

    public intakeSubSys() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
intakeRollerMtr = new WPI_VictorSPX(6);


        
intakeExtentionValve = new Solenoid(0, 2);
addChild("IntakeExtentionValve",intakeExtentionValve);

        
mixerMotor = new WPI_VictorSPX(12);


        

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

    mixerMotorTimer = new Timer();    

}

    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop\
        if(rollerState == RollerState.STOPPED) { intakeRollerMtr.set (ROLLERSTOPPEDSPEED); }
        else if(rollerState == RollerState.EJECT) { intakeRollerMtr.set (ROLLEREJECTSPEED); }
        else if(rollerState == RollerState.RETRACT) { intakeRollerMtr.set (ROLLERRETRACTSPEED); }

        if(intakeValveState == IntakeValveState.RETRACTED) { intakeExtentionValve.set (VALVE_RETRACT_STATE) ; }
        if(intakeValveState == IntakeValveState.EXTENDED) { intakeExtentionValve.set (VALVE_EXTEND_STATE) ; }

        if (intakeSmardashState == IntakeSmardashState.UPDATE) updateSmartDashboard();

        // --------- Process Mixer Motor State -----------
        processMixerMotorState();
        
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void setSmartdashUpdatesOn(){
        intakeSmardashState = IntakeSmardashState.UPDATE;
    }
    public void setSmartdashUpdatesOf(){
        intakeSmardashState = IntakeSmardashState.NOUPDATE;
    }

    // --------- Process Mixer Motor State -----------
    private void processMixerMotorState(){   
        if (mixerByShooterState == MixerByShooterState.STOP) {
             mixerMotorDelayState = MixerMotorDelayState.OFF;
        }

        if ((mixerByIntakeState == MixerByIntakeState.STOP) &&
            (mixerByShooterState == MixerByShooterState.STOP) &&
            (mixerByColorwheelState == MixerByColorwheelState.STOP)){
                // All systems want to shut down the motor
                mixerMotor.set( 0.0 );
                colorWheelSpeed = 0;
                mixerMotorDelayState = MixerMotorDelayState.OFF;
                return;
        }

        // Both systems do not want to stop so keep running
        if (mixerByColorwheelState == MixerByColorwheelState.COLORSPEED) {
            mixerMotor.set(COLORWHEEL_COLOR_SPEED);
            colorWheelSpeed = COLORWHEEL_COLOR_SPEED;
            return;
        }
        if (mixerByColorwheelState == MixerByColorwheelState.ROTATESPEED) {
            mixerMotor.set(COLORWHEEL_ROTATE_SPEED);
            colorWheelSpeed = COLORWHEEL_ROTATE_SPEED;
            return;
        }
        if (mixerByIntakeState == MixerByIntakeState.FEED) {
            mixerMotor.set(-MIXER_INTAKE_FEED_SPEED);
            return;
        }              

        if (mixerByShooterState == MixerByShooterState.EJECT) {
            // We need to clear out a jambed ball .. reverse mixer motor
            mixerMotor.set(MIXER_SHOOTER_CLEAR_SPEED);
            return;   
        }

        // The only mode left is MixerByShooterState.FEED
        if (mixerMotorDelayState == MixerMotorDelayState.OFF) {
            // we are just starting for the first time
            mixerMotorDelayState = MixerMotorDelayState.WAITING;
            mixerMotorTimer.reset();
            mixerMotorTimer.start();
        }

        if (mixerMotorDelayState == MixerMotorDelayState.WAITING) {
            if (mixerMotorTimer.get() > MIXER_MOTOR_DELAY_TIME){
                // our delay time has finaly expired OK to Startup motor
                mixerMotorDelayState = MixerMotorDelayState.RUNNING;
                mixerMotor.set(MIXER_SHOOTER_FEED_SPEED);    
            } else {
                // were still waiting for timer to time out so dont run motor
                mixerMotor.set(0);
            }
        } else {
            // No timer still running OKto start motor
            mixerMotor.set(MIXER_SHOOTER_FEED_SPEED); // its OK to start up motor
        }

    }


        public void stopMixer() {
            mixerMotor.set(0);
            mixerPower = 0;
        }

        public void setMixerPower(double power) {
            mixerMotor.set(power);
            mixerPower = power;
        }

        public void setMixerByShooterState(MixerByShooterState state){
            mixerByShooterState = state;
        }

        public void setMixerByIntakeState(MixerByIntakeState state){
            mixerByIntakeState = state;
        }

        public void setMixerByColorwheelState(MixerByColorwheelState state){
            mixerByColorwheelState = state;
        }

        public void rollerMotorEject() {
            rollerState = RollerState.EJECT;
        }

        public void rollerMotorRetract() {
            rollerState = RollerState.RETRACT; 
        }

        public void rollerMotorOff() {
            rollerState = RollerState.STOPPED;
        }

        public void extendIntake() {
            intakeValveState = IntakeValveState.EXTENDED;
        }

        public void retractIntake() {
            intakeValveState = IntakeValveState.RETRACTED;
        }

        public void updateSmartDashboard() {
            if ( (displayCnt % 10) != 0) {
                // only update dashboard every 200 ms so get out until time up
                displayCnt++;
                return;
            }
            displayCnt = 0;

            if (rollerState == RollerState.EJECT)        { SmartDashboard.putString("Intake Roller State", "Ejecting") ; }
            else if (rollerState == RollerState.RETRACT) { SmartDashboard.putString("Intake Roller State", "Retracting") ; }
            else if (rollerState == RollerState.STOPPED) { SmartDashboard.putString("Intake Roller State", "Stopped") ; }

            if (intakeValveState == IntakeValveState.EXTENDED)       { SmartDashboard.putString("Intake Valve State", "Extended") ; }
            else if (intakeValveState == IntakeValveState.RETRACTED) { SmartDashboard.putString("Intake Valve State", "Retracted") ; }

            SmartDashboard.putNumber("Mixer Power", mixerPower);

            if (mixerByIntakeState == MixerByIntakeState.STOP) SmartDashboard.putString("Mixer By Instake State", "STOP" );
            if (mixerByIntakeState == MixerByIntakeState.FEED) SmartDashboard.putString("Mixer By Instake State", "FEED" );
            if (mixerByShooterState == MixerByShooterState.STOP) SmartDashboard.putString("Mixer By Shooter State", "STOP" );
            if (mixerByShooterState == MixerByShooterState.FEED) SmartDashboard.putString("Mixer By Shooter State", "FEED" );
            if (mixerByColorwheelState == MixerByColorwheelState.STOP) SmartDashboard.putString("Mixer By Colorwheel State", "STOP" );
            if (mixerByColorwheelState == MixerByColorwheelState.COLORSPEED) SmartDashboard.putString("Mixer By Colorwheel State", "Color Speed" );
            if (mixerByColorwheelState == MixerByColorwheelState.ROTATESPEED) SmartDashboard.putString("Mixer By Colorwheel State", "Rotate Speed" );
        
            SmartDashboard.putNumber("ColorWheel Speed", colorWheelSpeed);
        }
}

