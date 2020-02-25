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
import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

/**
 *
 */
public class climbToPositionCmd extends Command {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    public static final double raiseClimberTopHeight  = 41;// 
    private static final double lowerSpeedConstraint = 0.1;//Keeping motor speeds at safe levels.
    private static final double raiseSpeedConstraint = 0.4;

    private double raisedHeight = 0;

    enum heightPositions {
        TOP, BOTTOM, MIDDLE;
    }

    private double direction;
    private double moveSpeed;

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public climbToPositionCmd(heightPositions position) {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.climberSubSys);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        
  
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        
        moveSpeed = 0;
        direction = Robot.climberSubSys.getHeight() - raisedHeight;
        
        if(Math.abs(direction) > 0.01) {
            
            if(direction < -lowerSpeedConstraint) {
                direction = -lowerSpeedConstraint;
            }else if(direction > raiseSpeedConstraint) {
                direction = raiseSpeedConstraint;
            }

            moveSpeed = direction;
        }else {
            moveSpeed = 0;    
        }

        if(moveSpeed > 0) {
            Robot.climberSubSys.climbMotorSetAtSpeed(moveSpeed, false);
        }
    
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        if(Math.abs(direction) < 0.01) {
            end();
        }
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

    public void getHeightToRaise(heightPositions position) {
        
        if(position == heightPositions.BOTTOM) {
            raisedHeight = 0;
        }else if(position == heightPositions.MIDDLE) {
            raisedHeight = Math.round(((raiseClimberTopHeight/2)*10))/10;//Gets the middle height of the climber, can go to 1/10 of an inch.
        }else if(position == heightPositions.TOP) {
            raisedHeight = raiseClimberTopHeight;
        }

    }

}
