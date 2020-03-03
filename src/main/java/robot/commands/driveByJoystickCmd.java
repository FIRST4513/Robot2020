// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package robot.commands;
//import robot.subsystems.*;
import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

import edu.wpi.first.wpilibj.Joystick;
//import robot.subsystems.Drivetrain.DriverControllerMode;


public class driveByJoystickCmd extends Command {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    private enum JoyType {JOY, XBOX, PLAY};
    private JoyType joyType;

     // --------- Joystick Assignments ---------
     private Joystick        driverJoy, coDriverJoy;

     // driver
     private static final int PLAY_REV_AXIS = 3;         // Left trigger 
     private static final int PLAY_FWD_AXIS = 4;         // right triger
     private static final int PLAY_TWIST_AXIS = 0;    
     private static final int PLAY_SET_LOW_GEAR_BTN = 1;

     private static final int CAPTURE_ON_BTN = 2;
     private static final int CAPTURE_OFF_BTN = 3;
     private static final int CAMERA_FRONT_BTN = 5;
     private static final int CAMERA_REAR_BTN = 6;
     private static final int COMPRESSOR_ON_BTN = 9;
     private static final int COMPRESSOR_OFF_BTN = 10;

	private static double currVel, lastVel;
	public static final double TWISTTHROTTLEMOD = 0.78;			//reserves memory for variable/attribute TWISTTHROTTLEMOD which is a constant
    private static final double TWISTDEADBAND = 0.05;		    //reserves memory for variable/attribute TWISTDEADBAND which is a constant
    private static final double XYDEADBAND = 0.1;				//reserves memory for variable/attribute XYDEADBAND which is a constant
    private static final double UP_SHIFT_VELOCITY_PT = 12;		// Speed less than this will use low gear else jumps into high gear 
    private static final double DOWN_SHIFT_VELOCITY_PT = 6;

    enum ACCELMODE {ACCEL, DECEL};
    ACCELMODE velocityMode;

    // Joystick Mappings
    // private static final int JOY_TRIGGER_BTN = 1;
    // private static final int JOY_LOW_GEAR_BTN = 7;
    // //private static final int JOY_HIGH_GEAR_BTN = 8;
    // private static final int JOY_REVERSE_BTN = 11;
    // //private static final int JOY_LOG_BTN = 7;

    // XBox Mappings
    //private static final int XBOX_TRIGGER_BTN = 1;
    // private static final int XBOX_REV_AXIS = 2;
    // private static final int XBOX_FWD_AXIS = 3;

    // private static final int XBOX_LEFT_TWIST_AXIS = 0;    
    // private static final int XBOX_RIGHT_TWIST_AXIS = 4;
    
    // private static final int XBOX_LEFT_LOW_GEAR_BTN = 9;
    // private static final int XBOX_RIGHT_LOW_GEAR_BTN = 10;
    //private static final int XBOX_HIGH_GEAR_BTN = 8;
    //private static final int XBOX_LOG_BTN = 7;

    // Playstation Mappings

    // private static final int PLAY_LEFT_TWIST_AXIS = 0;    
    // private static final int PLAY_RIGHT_TWIST_AXIS = 0;
    
    // private static final int PLAY_LEFT_LOW_GEAR_BTN = 10;
    // private static final int PLAY_RIGHT_LOW_GEAR_BTN = 10;

    // private static final int PLAY_REVERSE_BTN = 9;
    //private static final int PLAY_HIGH_GEAR_BTN = 8;
    //private static final int PLAY_LOG_BTN = 7;

    //private static int REVERSE_BTN = 11;
    //private static int LOW_GEAR_BTN = 7;
    //private static int HIGH_GEAR_BTN = 8;
    //private static LOG_BTN = 7;
    //private static int TRIGGER_BTN = 1;

    private double throttle, yValue, twistValue;				// joystick working variable	
    //private Joystick joy, xBox;
    private Joystick play, ctrlr;
	private double joyYaxis, joyTwist, joyThrottle;
    public double motorSpeed,motorAngle;
	
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public driveByJoystickCmd() {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.drivetrain);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES


    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        //joy = Robot.oi.driverJoystick;      // port 0
        //xBox = Robot.oi.driverXboxCtlr;     // port 2
        play = Robot.oi.driverPlaystation;  // port 3
        //System.out.println(" DrivebyJoystickCmd Triggrred!");

        /*
        if (Robot.drivetrain.isControllerJoy()){
            joyType =JoyType.JOY;
            ctrlr = joy;
            REVERSE_BTN = JOY_REVERSE_BTN;
            LOW_GEAR_BTN = JOY_LOW_GEAR_BTN;
        } else if (Robot.drivetrain.isControllerXbox()){ 
            joyType = JoyType.XBOX;
            ctrlr = xBox;
            ctrlr.setXChannel(XBOX_FWD_AXIS);        // Forward
            ctrlr.setYChannel(XBOX_REV_AXIS);        // Reverse
            if (Robot.drivetrain.isXboxModeLeft()) {
                ctrlr.setZChannel(XBOX_LEFT_TWIST_AXIS);
                LOW_GEAR_BTN = XBOX_LEFT_LOW_GEAR_BTN;
            } else {              
                ctrlr.setZChannel(XBOX_RIGHT_TWIST_AXIS);
                LOW_GEAR_BTN = XBOX_RIGHT_LOW_GEAR_BTN;
            }
        } else {
            */

            // We are useing a Playstation controller 

            joyType = JoyType.PLAY;
            ctrlr = play;
            ctrlr.setXChannel(PLAY_FWD_AXIS);        // Forward
            ctrlr.setYChannel(PLAY_REV_AXIS);        // Reverse
            ctrlr.setZChannel(PLAY_TWIST_AXIS);
            
            //REVERSE_BTN = PLAY_REVERSE_BTN;
            // if (Robot.drivetrain.isXboxModeLeft()) {
            //     ctrlr.setZChannel(PLAY_LEFT_TWIST_AXIS);
            //     LOW_GEAR_BTN = PLAY_LEFT_LOW_GEAR_BTN;
            // } else {              
            //     ctrlr.setZChannel(PLAY_RIGHT_TWIST_AXIS);
            //     LOW_GEAR_BTN = PLAY_RIGHT_LOW_GEAR_BTN;
            // }
        //}

        lastVel = Robot.drivetrain.getRunningAverageVelocity();
        currVel = 0;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        // lets verify we have a plugged in controller
        // if ((ctrlr.getRawButton(COMPRESSOR_ON_BTN)   == true) &&
        //     (ctrlr.getRawButton(COMPRESSOR_OFF_BTN)) == true) {
        //         // force a recheck of the existance of a controller
        //         Robot.checkForPS4Contoller();    
        //     }
        if (Robot.joystickStatus == Robot.JoystickStatus.NOTCONNECTED){
            Robot.drivetrain.stopMtrs();
            return;
        }

        if (joyType == JoyType.PLAY) {
            if ((ctrlr.getX() > -0.9) ||
                (ctrlr.getY() > -0.9) || 
                (Math.abs(ctrlr.getZ()) > 0.1)) {
                    // DRIVING by PS4 Controller
                    //System.out.printf(" x=%g  y=%g z=%g\n\r ", ctrlr.getX(), ctrlr.getY(), ctrlr.getZ() );
                    updateDrivetrainGearSetting();                      // Automatic transmission routine (change to High speed as needed)! 
                    calcPlayMotorSpeedAngle();                          // Get joystick position values and calculate speed and direction

                    Robot.drivetrain.driveByJoystick(motorSpeed,motorAngle); // Send command to Drivetrain to run motors
                    //System.out.printf(" motorSpeed = %g  motorAngle = %g \n\r ", motorSpeed, motorAngle );
                } else {
                    Robot.drivetrain.stopMtrs();   
                }
        }
        //if (ctrlr..getRawButton(LOG_BTN) == true) logData();
                /*
        if (joyType == JoyType.JOY) {
            if (Robot.oi.driverJoystick.getRawButton(JOY_TRIGGER_BTN) == true){
                //System.out.printf(" y=%g  twist=%g \n\r ",ctrlr.getY(),ctrlr.getTwist());
                updateDrivetrainGearSetting();              // Automatic transmission routine (change to High speed as needed)! 
                calcJoyMotorSpeedAngle();                   // Get joystick position values and calculate speed and direction
                Robot.drivetrain.driveByJoystick(motorSpeed,motorAngle);  // Send command to Drivetrain to run motors
                return;
            } else {
                // Joystick is not activated !
                Robot.drivetrain.stopMtrs();
                return;
            }
        }

        if (joyType == JoyType.XBOX) {
            if ((ctrlr.getX() > 0.1) ||
                (ctrlr.getY() > 0.1) || 
                (Math.abs(ctrlr.getZ()) > 0.1)) {
                    // DRIVING by Xbox Controller
                    //System.out.printf(" y=%g  twist=%g \n\r ", ctrlr.getX(), ctrlr.getY(), ctrlr.getZ() );
                    updateDrivetrainGearSetting();                      // Automatic transmission routine (change to High speed as needed)! 
                    calcXboxMotorSpeedAngle();                          // Get joystick position values and calculate speed and direction
                    Robot.drivetrain.driveByJoystick(motorSpeed,motorAngle); // Send command to Drivetrain to run motors
                } else {
                    Robot.drivetrain.stopMtrs();   
                }
        }
        */
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }
    
    // Called once after isFinished returns true
    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }


    // -------------------------------------------------------------------------------
    // --------------------- Automatic Transmission Methods --------------------------
    // -------------------------------------------------------------------------------

    private void updateDrivetrainGearSetting(){
        // ----- Check if we need to change gears -----
        currVel = Robot.drivetrain.getRunningAverageVelocity();
        

        // Check for Manual gear mode
        // if ( ctrlr.getRawButton(PLAY_SET_LOW_GEAR_BTN) == true) {
        //     // if button is held down disregard velocity and acceleration and just lock in low
        //     Robot.drivetrain.setGearHI();
        //     lastVel = currVel;
        //     return;
        // } else {
        //     Robot.drivetrain.setGearLO();
        //     return;
        // }
    }

    /*
        // Calculate if we are Accelerating or Decelerating
        if (Math.abs(currVel) > Math.abs(lastVel)) {
            velocityMode = ACCELMODE.ACCEL;
        } else {
            velocityMode = ACCELMODE.DECEL;            
        }
        lastVel = currVel;

        // Automatic Transmission Code 
        if (velocityMode == ACCELMODE.ACCEL) {
            // we are accelerating .. Check if we need to Shift to High Gear
            if (Math.abs(currVel) >= UP_SHIFT_VELOCITY_PT ) {
                shiftHigh();
            } else {
                shiftLow();
            }
            return;
        }

        if (Math.abs(currVel) <= DOWN_SHIFT_VELOCITY_PT) {
            shiftLow();
        }
    }

    private void shiftHigh() {
        if (Robot.drivetrain.isInLowGear()) {
            Robot.drivetrain.setGearHI();
            //Robot.logger.appendLog("Shifted to HIGH GEAR");
        }
    }

    private void shiftLow() {
        if (!Robot.drivetrain.isInLowGear()) {
            Robot.drivetrain.setGearLO();
            //Robot.logger.appendLog("Shifted to LOW GEAR");
        }
    }
*/

    // --------------------------------------------------------------------------------
    //     Calculate for Motor Reversing, Slow-Speed, Deadband and Throttle Adjustment
    // --------------------------------------------------------------------------------    
      
    void calcPlayMotorSpeedAngle() {
        // ------------ Xbox Controller Operation ------------
        // Left REV trigger = axis 2 moves 0 - 1
        // Right FWD trigger = axis 3 moves 0 - 1
        // Twist = axis 4  moves -1 to 1 -1 full left +1 full right
        // Low Gear manual btn 9

        double speed = 0;
        double turn = 0;

        if (ctrlr.getX() > -1){
            //were being driven forward
            speed = (((ctrlr.getX()) + 1) / 2);  // this converts speed from (-1 to +1) into ( 0 to 1)     
        } else {
            //were being driven backwards
            speed = (((ctrlr.getY() +1) /2 )* -1); // this converts speed from (-1 to +1) into ( 0 to -1) 
        }
        
        //if (ctrlr.getRawButton(REVERSE_BTN) == true){	 joyYaxis = joyYaxis * -1; }
        //motorSpeed = limitCurrent(joyYaxis);      // Limit speed to prevent voltage drops (Testing)
        motorSpeed = speed * -1;
 
        // Calculate Twist value
        turn = -ctrlr.getZ() * TWISTTHROTTLEMOD ;
        if((turn <= TWISTDEADBAND) && (turn >= -TWISTDEADBAND)) {
            turn=0;									// if YValue is within the deadband, don't move
        } else {
            if (turn >= 0) {
                turn=(turn - TWISTDEADBAND) * (1 + TWISTDEADBAND);		// Scale Yvalue t0 0 to + 1
            } else {
                turn = - (-turn - TWISTDEADBAND) * (1 + TWISTDEADBAND);	// Scale Yvalue t0 0 to -1
            }
        }
        motorAngle = turn;
    }

    /*
    void calcXboxMotorSpeedAngle() {
        // ------------ Xbox Controller Operation ------------
        // Left REV trigger = axis 2 moves 0 - 1
        // Right FWD trigger = axis 3 moves 0 - 1
        // Twist = axis 4  moves -1 to 1 -1 full left +1 full right
        // Low Gear manual btn 9

        if (ctrlr.getX() > 0){
            //were being driven forward
            joyYaxis = ctrlr.getX();
        } else {
            //were being driven backwards
            joyYaxis = ctrlr.getY();
            joyYaxis *= -1;
        }

        // Limit speed to prevent voltage drops
        //motorSpeed = limitCurrent(joyYaxis);
        motorSpeed = joyYaxis;

        //motorSpeed = joyYaxis;	// sets y value of joystick to variable YValue and Correct for Axis polarity 
        // Calculate Twist value
        joyTwist = -ctrlr.getZ();
        twistValue = joyTwist * TWISTTHROTTLEMOD;					// sets twist value of joystick to variable TwistValue
        if((twistValue <= TWISTDEADBAND) && (twistValue >= -TWISTDEADBAND)) {
            twistValue=0;									// if YValue is within the deadband, don't move
        } else {
            if (twistValue >= 0) {
                twistValue=(twistValue - TWISTDEADBAND) * (1 + TWISTDEADBAND);		// Scale Yvalue t0 0 to + 1
            } else {
                twistValue = - (-twistValue - TWISTDEADBAND) * (1 + TWISTDEADBAND);	// Scale Yvalue t0 0 to -1
            }
        }
        //motorAngle = twistValue * -1;
        motorAngle = twistValue;
    }


    void calcJoyMotorSpeedAngle() {
        // ------------ Joystick Operation ------------
        // Y axis forward -> -1 Drive Fwd    Pull Back -> +1 Drive reverse
        // X Axis Left    -> -1                    right -> +1  NOT USED
        // Twist CCW -> -1  Turn Left    CW -> +1   Turn Right
        // Throttle Forward -> -1  Faster    Back -> +1    Slower
        //
        joyYaxis = ctrlr.getY();
        if (ctrlr.getRawButton(REVERSE_BTN) == true){					// Check if Reverse Orientation Button Pressed
            joyYaxis = joyYaxis * -1;						// reverse direction
        }
        yValue=joyYaxis;									// sets y value of joystick to variable YValue
        joyTwist = -ctrlr.getTwist();
        joyThrottle = ctrlr.getThrottle();
        throttle=((joyThrottle-1)/-2);						// converts range to 0 to +1

        // ---------------------------------------------------------------
        // ----- Calculate Motor Speed using XYDEADBAND and throttle   ---
        // ---------------------------------------------------------------
        // Deadband Calculation
        if((yValue <= XYDEADBAND) && (yValue >= -XYDEADBAND)){
            yValue=0;									// if YValue is within the deadband, don't move
        }
        else {
            if (yValue >= 0) {
                yValue=(yValue - XYDEADBAND) * (1 + XYDEADBAND);		// Scale Yvalue t0 0 to + 1
            }
            else {
                yValue = - (-yValue - XYDEADBAND) * (1 + XYDEADBAND);	// Scale Yvalue t0 0 to -1
            }
        }
        motorSpeed = yValue * throttle;     // Throttle Calculation
        motorSpeed = motorSpeed*-1;         // Correct for Axis polarity

        // ------------------------------------------------------------------------
        // ----------------- Calculate Twist Value --------------------------------
        // ------------------------------------------------------------------------
        twistValue = joyTwist * TWISTTHROTTLEMOD;					// sets twist value of joystick to variable TwistValue
        if((twistValue <= TWISTDEADBAND) && (twistValue >= -TWISTDEADBAND)) {
            twistValue=0;									// if YValue is within the deadband, don't move
        } else {
            if (twistValue >= 0) {
                twistValue=(twistValue - TWISTDEADBAND) * (1 + TWISTDEADBAND);		// Scale Yvalue t0 0 to + 1
            } else {
                twistValue = - (-twistValue - TWISTDEADBAND) * (1 + TWISTDEADBAND);	// Scale Yvalue t0 0 to -1
            }
        }
        motorAngle = twistValue * throttle;
    }
    
    */
    
    private double limitCurrent(double joyValue){
        double tempPwr;
        double currSpeed = Robot.drivetrain.getAverageSpeed();
        if (joyValue > 0) {
            // were trying to drive fwd 0 -> +1
            tempPwr = joyValue;
            //if      ((currSpeed < 5.0) && (joyValue > 0.6)) tempPwr = 0.6;
            //else if ((currSpeed < 10.0) && (joyValue > 0.7)) tempPwr = 0.7;
            //else if ((currSpeed < 18.0) && (joyValue > 0.80)) tempPwr = 0.80;
            if      ((currSpeed < 5.0) && (joyValue > 0.60)) tempPwr = 0.6;
            else if ((currSpeed < 10.0) && (joyValue > 0.65)) tempPwr = 0.65;
            else if ((currSpeed < 15.0) && (joyValue > 0.70)) tempPwr = 0.70;
            else if ((currSpeed < 20.0) && (joyValue > 0.75)) tempPwr = 0.75;
            else if ((currSpeed < 25.0) && (joyValue > 0.80)) tempPwr = 0.80;
            else if ((currSpeed < 30.0) && (joyValue > 0.85)) tempPwr = 0.85;
            return tempPwr;
        } else {
            // were trying to drive in reverse 0 -> -1
            tempPwr = joyValue;
            if      ((currSpeed > -5.0) && (joyValue < -0.60)) tempPwr = -0.60;
            else if ((currSpeed > -10.0) && (joyValue < -0.65)) tempPwr = -0.65;
            else if ((currSpeed > -18.0) && (joyValue < -0.70)) tempPwr = -0.70;
            else if ((currSpeed > -20.0) && (joyValue < -0.75)) tempPwr = -0.75;
            else if ((currSpeed > -25.0) && (joyValue < - 0.80)) tempPwr = -0.80;
           else if ((currSpeed > -30.0)  && (joyValue < -0.85)) tempPwr = -0.85;
            return tempPwr;
        }
    }

    public void logData(){
        Robot.drivetrain.setLoggingOn();		// set flag so logging can occur
        Robot.drivetrain.logDrivetrain();		// send log record
        Robot.drivetrain.setLoggingOff();		// set flag so logging can occur
    }
}
