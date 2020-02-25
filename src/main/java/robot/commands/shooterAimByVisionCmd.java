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
public class shooterAimByVisionCmd extends Command {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    private double targetX, targetY, targetDistance, hoodPosition;

    private static final float TARGET_X_DEADBAND = 1;      // TODO Change this number to get tighter
    private static final float TARGET_Y_DEADBAND = 1;

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public shooterAimByVisionCmd() {

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

        // Bring fly wheel up to speed and leave running ???????
        //Robot.shooterSubSys.flywheelSetOn(Robot.shooterSubSys.HIGH_GOAL_SPEED, Robot.shooterSubSys.HIGH_GOAL); 
        //System.out.println("shooterAimByVisionCmd starting up !");
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {       

        if (Robot.udpSubSys.isValidVisionTarget() == false) {
            Robot.shooterSubSys.turretRotateMotorStop();
            Robot.shooterSubSys.turretHoodMotorStop();
            return;
        }
        // We have a target in sight so continue !
        

        // *********************** Process Turret Rotate to Target ************************

        targetX = Robot.udpSubSys.getXangle();

        if (targetX >= Math.abs(TARGET_X_DEADBAND)) {
            // were not centered on target yet !
            if (targetX > 0) {
                // we need to turn to the right
                if      (targetX >= 20.0) { Robot.shooterSubSys.turretHoodMotorSet(0.7 , false); }
                else if (targetX >= 10.0) { Robot.shooterSubSys.turretHoodMotorSet(0.5, false); }
                else if (targetX >= 5.0)  { Robot.shooterSubSys.turretHoodMotorSet(0.4, false); }
                else if (targetX >= 2.5)  { Robot.shooterSubSys.turretHoodMotorSet(0.2, false); }
            } else {
                // we need to turn to the left
                if      (targetX <= -20.0) { Robot.shooterSubSys.turretHoodMotorSet(0.7, false); } 
                else if (targetX <= -10.0) { Robot.shooterSubSys.turretHoodMotorSet(0.5, false); }
                else if (targetX <= -5.0)  { Robot.shooterSubSys.turretHoodMotorSet(0.4, false); }
                else if (targetX <= -2.5)  { Robot.shooterSubSys.turretHoodMotorSet(0.2, false); }
            }
        } else {
            // we are centered on target
            Robot.shooterSubSys.turretRotateMotorStop();
        }

        //if (targetY >= Math.abs(TARGET_Y_DEADBAND)) {
        //    if (targetDistance >= 20) { Robot.shooterSubSys.turretHoodMotorSet(0.7, false); }
        //    else if (targetDistance >= 10) { Robot.shooterSubSys.turretHoodMotorSet(0.5, false); }
        //    else if (targetDistance >= 5) { Robot.shooterSubSys.turretHoodMotorSet(0.4, false); }
        //    else if (targetDistance >= 2.5) { Robot.shooterSubSys.turretHoodMotorSet(0.3, false); }
        //}
        
        
        
        // *********************** Process Turret Hood to Target ************************
        // targetY = Robot.udpSubSys.getYangle();  // vision distance not used
        // targetDistance = Robot.udpSubSys.getLidarDistance();

        // if (targetDistance == 0){
        //     // we dont have valid distance data so just get out
        //     return;
        // }

        // hoodPosition = Robot.shooterSubSys.getHoodPot();

        // if (targetDistance < 60.0) {}

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
        Robot.shooterSubSys.turretHoodMotorStop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
