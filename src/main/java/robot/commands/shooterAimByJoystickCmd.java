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
//import robot.subsystems.shooterSubSys.HoodSwitchPressed;
//import robot.subsystems.shooterSubSys.TurretSwitchPressed;

/**
 *
 */
public class shooterAimByJoystickCmd extends Command {

    private static final double HOOD_ROTATE_SCALER = 0.5;    // slowing

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public shooterAimByJoystickCmd() {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.shooterSubSys);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES


    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        
        double joyTwist = Robot.oi.getcoDriverJoystick().getTwist();
        //double joyY = Robot.oi.getcoDriverJoystick().getY();
        double joyThrottle = Robot.oi.getcoDriverJoystick().getThrottle();
        //joyThrottle = (joyThrottle + 1)/2 ;  // this changes -1 to +1 into 0 to +1
        joyThrottle= ((joyThrottle-1)/-2);      // this changes +1 to -1 into 0 to +1
        boolean buttonPressed = Robot.oi.getcoDriverJoystick().getRawButton(2);
        boolean overide = Robot.oi.getcoDriverJoystick().getRawButton(9);

        if(buttonPressed == true) {
            Robot.shooterSubSys.turretRotateMotorSet(((joyTwist * HOOD_ROTATE_SCALER) * joyThrottle) , overide);
            //Robot.shooterSubSys.turretHoodMotorSet(joyY, overide);
            //Robot.shooterSubSys.turretRotateMotorSet((joyTwist * ((joyThrottle + 1)/2)) , overide);
            //Robot.shooterSubSys.turretHoodMotorSet(joyY * ((joyThrottle + 1)/2), overide);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.shooterSubSys.turretRotateMotorStop();
        //Robot.shooterSubSys.turretHoodMotorStop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
