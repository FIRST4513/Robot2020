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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
//import robot.commands.DriveFwd2Cmd.DriveDir;
//import robot.commands.DriveFwd2Cmd.DriveMode;

/**
 *
 */
public class DriveBrakeCmd extends Command {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    private double m_TO;
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    
    String line;
    Timer startTmr = new Timer();
    int mLeftStallCtr = 0;
    int mRightStallCtr = 0;    
    final double START_TIME = 0.08;				// Time to wait for motors to start moving robot
    final double STALL_VELOCITY = 2.0;			// Speed less than this will indicate stopped 
    final double STOP_PWR  = - 0.50;			// Power to reverse motors to brake
    double currStop_pwr = STOP_PWR;
    
    public enum DriveState { START, BRAKING, DONE };
    private DriveState mLeftDriveState = DriveState.START;	// this keeps track of our current drive mode
    private DriveState mRightDriveState = DriveState.START;	// this keeps track of our current drive mode
    
    public enum DriveDir { FWD, REV };
    private DriveDir mLeftDriveDir  = DriveDir.FWD;	// this keeps track of our drive direction
    private DriveDir mRightDriveDir = DriveDir.FWD;	// this keeps track of our drive direction
    
    private double mCurrLeftVelocity, mCurrRightVelocity, mDist;
    private double mLeftBrakePwr, mRightBrakePwr;
    
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public DriveBrakeCmd(double TO) {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        m_TO = TO;

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.drivetrain);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    	line = " ********** Starting DriveBrakeCmd ***********";
    	Robot.logger.appendLog(line);
    	System.out.println(line);
    	setTimeout(m_TO);
    	
    	mLeftStallCtr = 0;
    	mRightStallCtr = 0;
    	mLeftDriveState = DriveState.START;
    	mRightDriveState = DriveState.START;    	
    	
    	mCurrLeftVelocity = Robot.drivetrain.getLeftSpeed();
    	mCurrRightVelocity = Robot.drivetrain.getRightSpeed();
    	
    	if (mCurrLeftVelocity >= 0)  mLeftDriveDir =  DriveDir.FWD;
    	else						 mLeftDriveDir =  DriveDir.REV;
    	if (mCurrRightVelocity >= 0) mRightDriveDir = DriveDir.FWD;
    	else						 mRightDriveDir = DriveDir.REV;
    	
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    	mDist = Robot.drivetrain.getAverageDist();
    	mCurrLeftVelocity = Robot.drivetrain.getLeftSpeed();
    	mCurrRightVelocity = Robot.drivetrain.getRightSpeed();

    	// ------ Brake harder if motor is going faster ------
    	if 		(Math.abs(mCurrLeftVelocity) > 50 ) mLeftBrakePwr = 0.80;
    	else if (Math.abs(mCurrLeftVelocity) > 30 ) mLeftBrakePwr = 0.75;
    	else if (Math.abs(mCurrLeftVelocity) > 20 ) mLeftBrakePwr = 0.70;
    	else if (Math.abs(mCurrLeftVelocity) > 10 ) mLeftBrakePwr = 0.65;
    	else if (Math.abs(mCurrLeftVelocity) > 05 ) mLeftBrakePwr = 0.55;
    	else    									mLeftBrakePwr = 0.40;
    	
    	if 		(Math.abs(mCurrRightVelocity) > 50 ) mRightBrakePwr = 0.80;
    	else if (Math.abs(mCurrRightVelocity) > 30 ) mRightBrakePwr = 0.75;
    	else if (Math.abs(mCurrRightVelocity) > 20 ) mRightBrakePwr = 0.70;
    	else if (Math.abs(mCurrRightVelocity) > 10 ) mRightBrakePwr = 0.65;    	
    	else if (Math.abs(mCurrRightVelocity) > 05 ) mRightBrakePwr = 0.55;
    	else    									 mRightBrakePwr = 0.40;
    	    	
    	// ----- Check to see if Left Motor has stopped -----
    	if ((mLeftDriveState == DriveState.START) || (mLeftDriveState == DriveState.BRAKING)) {
    		if  (((mLeftDriveDir == DriveDir.FWD) && (mCurrLeftVelocity <= STALL_VELOCITY)) ||
    			 ((mLeftDriveDir == DriveDir.REV) && (mCurrLeftVelocity >= -STALL_VELOCITY))) {
    			// we have stopped moving
	    		mLeftStallCtr++; 
    			if (mLeftStallCtr >= 3) {
    				mLeftDriveState = DriveState.DONE;
    			}
    		} else {
    			// we are still moving so reset stall Ctr
    			mLeftStallCtr = 0;
    		}
    	}
    	
    	// ----- Check to see if Left Motor has stopped -----
    	if ((mRightDriveState == DriveState.START) || (mRightDriveState == DriveState.BRAKING)) {
    		if  (((mRightDriveDir == DriveDir.FWD) && (mCurrRightVelocity <= STALL_VELOCITY)) ||
    			 ((mRightDriveDir == DriveDir.REV) && (mCurrRightVelocity >= -STALL_VELOCITY))) {
    			// we have stopped moving
				mRightStallCtr++;
    			if (mRightStallCtr >= 3) {
    				mRightDriveState = DriveState.DONE;
    			}
    		} else {
    			// we are moving so reset stall Ctr
    			mRightStallCtr = 0;
    		}
    	}
    	
    	if (mLeftDriveState == DriveState.DONE) mLeftBrakePwr = 0;
    	if (mLeftDriveDir == DriveDir.FWD) mLeftBrakePwr *= -1;
    	
    	if (mRightDriveState == DriveState.DONE) mRightBrakePwr = 0;
    	if (mRightDriveDir == DriveDir.FWD) mRightBrakePwr *= -1;
    	
    	//System.out.println(" LFT PWR = " + mLeftBrakePwr + " RT PWR=" + mRightBrakePwr);
       	Robot.drivetrain.tankDrive(mLeftBrakePwr, mRightBrakePwr);
       	Robot.drivetrain.putZoneData( 9, mDist, 0, 0, 0 );	// Log distance to tgt 
    }
  


    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
      	if (isTimedOut()) {
            line = "DriveBrakeCmd - has Timed out !!";
    		Robot.logger.appendLog(line);
    		System.out.println(line) ;    
            return true;				// used in all modes
            }

    	if ((mLeftDriveState == DriveState.DONE) && (mRightDriveState == DriveState.DONE)) {
    		// Cmd has completed its distance
    		line = "DriveBrakeCmd - Done - has stopped the robot !!";
    		Robot.logger.appendLog(line);
    		System.out.println(line) ;   		
    		return true;
    	}
    	
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
		line = "DriveBrakeCmd has ended !!";
		Robot.logger.appendLog(line);
		System.out.println(line) ;
       	Robot.drivetrain.stopMtrs();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    	end();
    }
}
