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
import robot.subsystems.Drivetrain.DrivePIDStatus;

/**
 *
 */
public class DrivePointTurnPidCmd extends Command {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    private double m_turnAngle;
    private int m_mode;
    private double m_TO;
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    
    private String line;			// For logging
    
    Timer angleTmr = new Timer();					// To time how long we are at the end for possible stopping
    boolean firstTimeFlag;
    double ENDTIME = 0.25;			// Time since hitting the end to shutdown routine
    int state;						// State machine 0=init, 1=driving, 2=time to stop
    int startCnt, stopCnt;
    double currTime, lastTime;
    double currYaw, lastYaw;
    double MOVEANGLE = 1;			// Minimum angle cjange to indicate we are turning 
	double GYROSTOPRT = 1.6;		// Anything slower than this (degrees/sec)looks like a stop
	double GYROSTARTRT = 2.0 ;		// Anything faster than this looks like a start

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public DrivePointTurnPidCmd(double turnAngle, int mode, double TO) {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        m_turnAngle = turnAngle;
        m_mode = mode;
        m_TO = TO;

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.drivetrain);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    	Robot.drivetrain.setLoggingOn();
    	line = "Drive Point Turn PID Command Init (TurnAngle=" + m_turnAngle + " Mode=" + m_mode + "  TO=" + m_TO;
    	System.out.println(line) ;
    	Robot.logger.appendLog(line) ;
    	angleTmr.reset();
    	angleTmr.start();
    	setTimeout(m_TO);
    	firstTimeFlag = true;
    	state = 0;								// Initial startup state
    	lastYaw =  Robot.drivetrain.getGyroYaw();
    	lastTime = angleTmr.get();
    	
		Robot.drivetrain.pidRotateTo(m_turnAngle) ;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
   		currYaw =  Robot.drivetrain.getGyroYaw();
   		currTime = angleTmr.get();
   		//System.out.println("PIDRotCMd state= " + state + "currYaw=" + currYaw);
       	if (state == 0) {
    		// Look to see if we have started moving yet (angle changed by more than 1.5 degree)
    		if (Math.abs(lastYaw - currYaw) > MOVEANGLE ) {
    			// We are moving
    			state = 1;
    		}
       	} // end of state 0
    		
    	if (state == 1) {
       		// look to see if we have hit something or have stopped turning
       		double rate =calcRateOfTurn();
       		//System.out.println("Turn rate = " + rate);
       		if (Math.abs(rate) < GYROSTOPRT) {
    			// anything slower than xx degrees per second looks like a stall for 60 ms
    			stopCnt ++;
    			if (stopCnt >= 2) {
    				 state = 2;
    			}
       		} else { 
       			stopCnt = 0;	// reset counter low speed must have been a fluke
       		}					
    	   	if (Robot.drivetrain.getDrivePIDStatus() == DrivePIDStatus.STOPPED) {
    		    // PID has indicated its complete
    	   		state = 3;	
    	   	}
    	  	if (Robot.drivetrain.getDrivePIDStatus() == DrivePIDStatus.ATEND) {
    		    // PID has indicated its at the end but hasnt stopped
    	  		// consider a Timer to give time for final corection and then end ie) after 0.25 seconds end run
    	  		//state = 4;
    	  	}
    	} // end of state 1
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
    	if (isTimedOut()) {
    		line = "DrivePtTurnPIDCmd - Has timed out !!";
    		Robot.logger.appendLog(line);
    		System.out.println(line) ;
    		return true;							// used in all modes
    	}
    	
    	if (state == 2) {
    		// We have stopped turning time to end
       		line = "DrivePtTurnPIDCmd - Has stopped turning or is too slow !!";
    		Robot.logger.appendLog(line);
    		System.out.println(line) ;
    		return true;
    	}

    	if (state == 3) {
    		// PID has indicated its complete
       		line = "DrivePtTurnPIDCmd - PID has indicated it is STOPPED !!";
    		Robot.logger.appendLog(line);
    		System.out.println(line) ;
    		return true;
    	}
	
   		// consider testing for termination conditions such as
   		// 1. Timer - when distance is within tolerance start timer and after 0.25 seconds end run
   		// 2. Look at velocity when in tolerance zone if less than ~5 inches per sec. end run
   		// 3. Look at velocity even if not in tol-zone if it has dropped below a certain threshold then were done (assumming we had been moving)
    	
    	return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
		line = "DrivePtTurnPIDCmd has ended !!";
		Robot.logger.appendLog(line);
		System.out.println(line) ;
    	Robot.drivetrain.stopPID();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    	end();    	
    }
    
    public double calcRateOfTurn() {
   		double deltaTime = currTime - lastTime;
   		double deltaAngle = lastYaw - currYaw;
   		lastTime = currTime;
   		lastYaw = currYaw;
   		if (deltaTime == 0) deltaTime = 0.00000001;				// prevent divide by zero
   		double angleRate = 1 / deltaTime * deltaAngle;
   		//double angleRate = Robot.drivetrain.getGyroRate() ;		// this doesnt return correct data	
   		System.out.println("angleRate = " + angleRate);
   		return angleRate;
    }
}