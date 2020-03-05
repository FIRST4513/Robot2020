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
import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

/**
 *
 */
public class shooterAimByVisionCmd extends Command {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    String line = "";
    boolean kill = false;
    private double targetX, targetY, targetDistance;
    private double  currentHoodPosition, targetHoodPosition, deltaHoodPosition;

    private static final double TARGET_X_DEADBAND = 0.75;      // TODO Change this number to get tighter
    private static final double HOOD_DEADBAND = 0.01;        // hood pot position deadband 

    private static double visionTgtBearing = 0;
    private static double currentHoodAngle = 0;
    private static double targetHoodAngle = 0;


    // *************************************************************
    //                  Turret Data
    // fully Extended raises ball for a closer target
    // 112 inches distance to goal - Pot 4.99  for a ctr shot    ie) fully extended for CLOS Goal
    // 
    // fully Retracted lowers ball for a farther target
    // 380 inches distance to goal - Pot 0.0049 for a ctr shot   ie) fully retracted for Further Goal
    //
    //       Distance      Hood Pot     (9000 RPM Target)
    //      min travel      0.0049
    //
    //      380 inches      0.005
    //      277 inches      0.152
    //      225 inches      0.946
    //      177 inches      1.33
    //      144 inches      2.49
    //      120 inches      3.2
    //      112 inches      4.99
    //
    //      max travel      5.0  
    //  
    //      Autonomous Line shot set to 3.6 hood pot (between 112 inches and 120 inches)
    //
    
    private static final double HOOD_EXTENDED_MIN = 0.005; //4.8;        // measured 4.85 
    private static final double HOOD_RETRACTED_MAX = 4.8; // 0.005;     // measured 0.005
    private static final double MAX_SHOOTING_DISTANCE = 380;    // measired at this distance
    private static final double MIN_SHOOTING_DISTANCE = 112;    // measured at this distance

    //private static double hoodExtendedMax;
    //private static double hoodRetractedMin;
    //private double hoodRange;
    //private static double shootingSlope;
    //private static double shooting_Y_Intercept;

    private static final double HOOD_RANGE = HOOD_RETRACTED_MAX - HOOD_EXTENDED_MIN;

    private static final double SHOOTING_SLOPE =
        (HOOD_RETRACTED_MAX - HOOD_EXTENDED_MIN) / ( MAX_SHOOTING_DISTANCE - MIN_SHOOTING_DISTANCE);
    
    private static final double SHOOTING_Y_INTERCEPT = (SHOOTING_SLOPE * MIN_SHOOTING_DISTANCE) + HOOD_EXTENDED_MIN;

    private double currHoodPos = 0;
    private final double MOVESPEED = 0.3;

    private enum HoodDirState {RAISE, LOWER, DONE};
    private HoodDirState hoodDirState = HoodDirState.DONE;

    // *************************************************************

    private enum TurretRotateState {ROTATING_RIGHT, ROTATING_LEFT, DONE};
    private TurretRotateState turretRotateState  = TurretRotateState. DONE;

    private enum TurretHoodState {MOVING, DONE};
    private TurretHoodState turretHoodState  = TurretHoodState. DONE;

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public shooterAimByVisionCmd() {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING5

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.shooterSubSys);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }




    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        line = "shooterAimByVisionCmd starting up !";
        System.out.println(line);
        Robot.logger.appendLog(line); 
    }


    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        
        // *********************** Process Turret Hood to Target ************************
        targetDistance = Robot.shooterSubSys.getLidarDistance();
        if (targetDistance > 0){ 
            // we have valid distance so we can process it
            proccesAutoHood();                           
        } else {
            // we dont have good lidar data so stop motor
            Robot.shooterSubSys.turretHoodMotorStop();  
        }

        // *********************** Process Turret Rotate to Target ************************    
        proccesAutoTurretRotate();
    
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        // Runs while button is held
        
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.shooterSubSys.turretRotateMotorStop();
        Robot.shooterSubSys.turretHoodMotorStop();
        line = "shooterAimByVisionCmd ended !";
        System.out.println(line);
        Robot.logger.appendLog(line); 
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }

    // -----------------------------------------------------------------------
    //                       Hood Methods 
    // -----------------------------------------------------------------------

    private void proccesAutoHood(){
        currentHoodPosition = Robot.shooterSubSys.getHoodPot();
        if ( targetDistance < 80 ) {
            // We are too close to wall so we need to go for low shot
            targetHoodPosition = HOOD_EXTENDED_MIN;
            line = ("AutoHood - Too close to wall going for low goal - Hood target Distance(Lidar)=" + targetDistance +
                    "   currentHoodPosition=" + currentHoodPosition + " targetHoodPosition=" + targetHoodPosition );
        } else {
            targetHoodPosition = calcHoodPosition(); // This is based on Lidar distance
            line = ("AutoHood - Hood target Distance(Lidar)=" + targetDistance + 
                    "   currentHoodPosition=" + currentHoodPosition + " targetHoodPosition=" + targetHoodPosition );
        }
        determineHoodDir();
        setHoodMotorForPosition();
        //System.out.println(line);
        //Robot.logger.appendLog(line); 
    }

    private double calcHoodPosition(){
        // Linear straight line equation
        double calcHoodPotPos = (SHOOTING_SLOPE * targetDistance ) + SHOOTING_Y_INTERCEPT;

        // Polynomial 2
        double a = 18.31317924;
        double b = -0.1877388154 * targetDistance;
        double c = 6.524960901e-4 * (Math.pow(targetDistance, 2));
        double d = -7.511797993e-7 * (Math.pow(targetDistance, 3));
        calcHoodPotPos = a +  b + c + d;

        // prevent overdriving position
        if(calcHoodPotPos < HOOD_EXTENDED_MIN)   {   calcHoodPotPos = HOOD_EXTENDED_MIN; }
        if(calcHoodPotPos > HOOD_RETRACTED_MAX)  {   calcHoodPotPos = HOOD_RETRACTED_MAX;  }
        return calcHoodPotPos;
    }

    public void determineHoodDir(){
        if (currHoodPos == targetHoodPosition) {
            hoodDirState = HoodDirState.DONE;
            line = line + "  Hood Direction = DONE";
        }

        if (currHoodPos < targetHoodPosition) {
            hoodDirState = HoodDirState.RAISE;
            line = line + "  Hood Direction = RAISE";
            // Raise power = +
        } else {
            hoodDirState = HoodDirState.LOWER;
            line = line + "  Hood Direction = LOWER";
            // Lowere power = -
        }

        //hoodDirState = HoodDirState.DONE;
        //line = line + "  Hood Direction = Done";
    }

    private void setHoodMotorForPosition(){
        currHoodPos = Robot.shooterSubSys.getHoodPot();
        //System.out.println("Hood to pos CurrPos=" + currHoodPos);
        double delta = 0;
        double moveSpeed = 0;
        if (hoodDirState == HoodDirState.RAISE){
            if (currHoodPos < targetHoodPosition) {
                // Continue Raising
                delta = targetHoodPosition - currHoodPos;
                if      (delta > 2)     moveSpeed = 0.5;
                else if (delta > 1)     moveSpeed = 0.45;
                else if (delta > 0.5)   moveSpeed = 0.35;
                else if (delta > 0.25)  moveSpeed = 0.25;
                else                    moveSpeed = 0.1;
                Robot.shooterSubSys.turretHoodMotorSet( -moveSpeed, false);
                //Robot.shooterSubSys.turretHoodMotorSet( -MOVESPEED, false);
                return;
            } else {
                // we have reached our target
                System.out.println("State = Done !");
                hoodDirState = HoodDirState.DONE;
                return;
            }
        }

        if (hoodDirState == HoodDirState.LOWER){
            if (currHoodPos > targetHoodPosition) {
                // Continue Lowereing
                delta = Math.abs(targetHoodPosition - currHoodPos);
                if      (delta > 2)     moveSpeed = 0.5;
                else if (delta > 1)     moveSpeed = 0.45;
                else if (delta > 0.5)   moveSpeed = 0.35;
                else if (delta > 0.25)  moveSpeed = 0.25;
                else                    moveSpeed = 0.1;
                Robot.shooterSubSys.turretHoodMotorSet( moveSpeed, false);
                //Robot.shooterSubSys.turretHoodMotorSet( MOVESPEED, false);
                return;
            } else {
                // we have reached our target
                System.out.println("State = Done !");
                hoodDirState = HoodDirState.DONE;
                return;
            }
        }
    }

    
    // -----------------------------------------------------------------------
    //                   Vision Driven Turret Rotate Methods 
    // -----------------------------------------------------------------------

    private void proccesAutoTurretRotate(){
        // ************************ Turret Rotate by Vision ***********************
        double turretRotateSpeed = 0;
        double delta = 0;
        currentHoodAngle = Robot.shooterSubSys.getRotateAngle();

        if (Robot.udpSubSys.isValidVisionTarget() == true) {
            // We have a valid target in sight so lets update our target data
            visionTgtBearing = Robot.udpSubSys.getXangle();    
            targetHoodAngle = currentHoodAngle + visionTgtBearing;

            line = ("We have a valid vision Target XAngle=" + visionTgtBearing +
                     " CurrentHoodAngle=" + currentHoodAngle +
                     " VisionTgtBearing=" + visionTgtBearing +
                     " targetHoodAngle =" + targetHoodAngle );

            if ( Math.abs(visionTgtBearing) <= TARGET_X_DEADBAND ) {
                turretRotateState = TurretRotateState.DONE;
                line = line + "  Rotating DONE !";
            } else if (visionTgtBearing > 0) {
                turretRotateState = TurretRotateState.ROTATING_RIGHT;
                line = line + "  Rotating RIGHT !";
            } else {
                turretRotateState = TurretRotateState.ROTATING_LEFT;
                line = line + "  Rotating LEFT !";
            }
            System.out.println(line);
            Robot.logger.appendLog(line); 
        }

        if (turretRotateState == TurretRotateState.DONE) {
            // Nothning to do so get out
            turretRotateSpeed =  0.0;
            Robot.shooterSubSys.turretRotateMotorStop(); 
            return;
        }

        // Were not done yet
        delta = targetHoodAngle - currentHoodAngle;

        if ( delta > 0 ) {
            // we need to turn to the right
            turretRotateState = TurretRotateState.ROTATING_RIGHT;
            if      (delta >= 20.0)               { turretRotateSpeed = 0.5; }
            else if (delta >= 10.0)               { turretRotateSpeed = 0.4; }
            else if (delta >= 5.0)                { turretRotateSpeed = 0.25; }
            else if (delta >= 2.5)                { turretRotateSpeed = 0.22; }
            else if (delta >= 1.5)                { turretRotateSpeed = 0.18; }
            else if (delta >= TARGET_X_DEADBAND)  { turretRotateSpeed = 0.15; }
            else                                  { turretRotateSpeed = 0.0;
                                                    Robot.shooterSubSys.turretRotateMotorStop(); 
                                                    turretRotateState = TurretRotateState.DONE;
                                                  }
        } else {
            // we need to turn to the left
            turretRotateState = TurretRotateState.ROTATING_LEFT;
            if      (delta <= -20.0)              { turretRotateSpeed = -0.5; } 
            else if (delta <= -10.0)              { turretRotateSpeed = -0.4; }
            else if (delta <= -5.0)               { turretRotateSpeed = -0.25; }
            else if (delta <= -2.5)               { turretRotateSpeed = -0.22; }
            else if (delta <= -1.5)               { turretRotateSpeed = -0.20; }
            else if (delta <= -TARGET_X_DEADBAND) { turretRotateSpeed = -0.17; }
            else                                  { turretRotateSpeed =  0.0;
                                                    Robot.shooterSubSys.turretRotateMotorStop(); 
                                                    turretRotateState = TurretRotateState.DONE;
                                                  }
        }

        // send power to motor
        Robot.shooterSubSys.turretRotateMotorSet(turretRotateSpeed, false);

        line = ("We are rotating to our Target " +
        " CurrentHoodAngle=" + currentHoodAngle +
        " targetHoodAngle=" + targetHoodAngle +
        " Delta=" + delta +
        " TurretRotateSpeed= " + turretRotateSpeed);

        if (turretRotateState == TurretRotateState.ROTATING_RIGHT) {
            line = line + "  Rotating RIGHT !";
        } else if (turretRotateState == TurretRotateState.ROTATING_LEFT) {
            line = line + "  Rotating LEFT !";
        } else {
            line = line + "  Rotating DONE !";
        }
        System.out.println(line);
        Robot.logger.appendLog(line); 
    }

}
