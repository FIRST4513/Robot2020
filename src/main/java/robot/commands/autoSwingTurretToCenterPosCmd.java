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
public class autoSwingTurretToCenterPosCmd extends Command {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    private double currAngle = 0;
   
    private static final double TARGET_ANGLE = 90;

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public autoSwingTurretToCenterPosCmd() {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

    // Turrent Angle will intitialize to zero degrees when pointed to the left
    // Turrent Angle will be 90 degrees when facing straight forward

        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        setTimeout(5.0);
        
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        currAngle = Robot.shooterSubSys.getRotateAngle();
        
        System.out.println("autoCtrTurret currAngle=" + currAngle + "  Remaining Angle=" + (TARGET_ANGLE - currAngle));

        if (currAngle < TARGET_ANGLE) {
            // we need to rotate right
            Robot.shooterSubSys.turretRotateMotorSet( 0.4, false);
        } else {
            // we need to rotate left
            Robot.shooterSubSys.turretRotateMotorSet( -0.4, false);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {

        if (Math.abs(TARGET_ANGLE - currAngle) < 1.5) {
            // were close enough to target angle to end
            return true;
        }

        if (isTimedOut()) {
            System.out.println("autoSwingToCtrCmd has Timed out !!!!!");
            return true;
        }
        
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.shooterSubSys.turretRotateMotorStop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
