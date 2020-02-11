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
public class colorWheelRotateToColorCmd extends Command {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    enum State {NOCONTACT,SAME,CHANGE,DONE};
    State state = State.SAME;
    double changeCount = 0;
    String lastColor;
    
    final double ROTATESPEED = 0.2;
    // double reverseTimer ;
     String currentColor;
     String targetColor;

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public colorWheelRotateToColorCmd() {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.controlPanelSubSys);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        if (Robot.controlPanelSubSys.getContactSwitch()==false){
            end();
        }
        //  reverseTimer = 0;
        setTimeout(4);
        targetColor = Robot.controlPanelSubSys.getRequiredColor();
        currentColor = Robot.controlPanelSubSys.getSensorColor();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        currentColor = Robot.controlPanelSubSys.getSensorColor();
        if (currentColor != targetColor){
            Robot.controlPanelSubSys.spinnerTurn(ROTATESPEED);
        } else {
            Robot.controlPanelSubSys.spinnerStop();
           /* reverseTimer--;
            Robot.controlPanelSubSys.spinnerTurn(-ROTATESPEED); */
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        if(isTimedOut()){
            return true;
        }    
        if( (Robot.controlPanelSubSys.getSensorColor() == targetColor) ){ //&&(reverseTimer < 1)
            return true;
        }
        
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.controlPanelSubSys.spinnerStop();
        //Robot.controlPanelSubSys.spinnerRetract();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
