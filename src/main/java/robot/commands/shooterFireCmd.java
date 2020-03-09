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

//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.Robot;
//import robot.Robot.JoystickStatus;
import robot.subsystems.intakeSubSys.MixerByShooterState;
import robot.subsystems.shooterSubSys.FlywheelSpeedState;
import robot.subsystems.shooterSubSys.FlywheelState;
//import robot.subsystems.shooterSubSys.GoalTarget;
import robot.subsystems.shooterSubSys.FlywheelGoalTarget;

//import robot.subsystems.shooterSubSys.FlywheelSpeedState;

public class shooterFireCmd extends Command {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    //private static final double HIGH_GOAL_SPEED = 9000.0;
    //private static final double LOW_GOAL_SPEED = 3500.0;
    //private static final double PID_FIRE_DELTA = 100.0;     // The range +- OK to Fire

    //private static final boolean LOW_GOAL = false;
    //private static final boolean HIGH_GOAL = true;

    String line = "";

    private enum GoalTarget {HIGH, LOW};
    private GoalTarget goalTarget = GoalTarget.HIGH;

    private enum ModeState{Joystck, AUTO_10Foot};
    private ModeState modeState = ModeState.Joystck;
    private int firstTimeFlag = 0;

    private double targetRPM = Robot.shooterSubSys.HIGH_GOAL_SPEED;  // default to high goal

    private enum ShooterState { STARTUP, UP_TO_SPEED, UNDER_SPEED, FEEDING }
    ShooterState shooterState = ShooterState.STARTUP;

    // private static final double SHOOTER_SPEED = 5200; // In RPM
    // private static final double SHOOTER_PWR = 0.65;
    // private static final double RPM_DEADBAND = 100;

    private static final int SHOOTER_LOW_GOAL_BTN = 10;
    private static final int MANUAL_OVERIDE_BTN = 7;
    private boolean overide = false;

    private int m_mode = 0;
    private double m_timeout = 0;
    private double currentRPM = 0;
    private double delta = 0;

    // --------- Handoff Motor variables ------
    private static final double  HANDOFF_MOTOR_FEED_SPEED = 1.0;


    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public shooterFireCmd(int mode, double timeout) {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        m_mode = mode;
        m_timeout = timeout;

        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        firstTimeFlag = 0;      // This is used to make sure we at least get up to speed first
        if (m_mode == 0) {
            // fire by Joystick (no timeout)
            line = "shooterFireCmd has been started by the Joystick trigger!";
            modeState = ModeState.Joystck;
        } 
        if (m_mode == 1) {
            // fire by Autonomous from 10 foot mark
            setTimeout(m_timeout);
            line = ("shooterFireCmd has been started by Autonomous Firing -------- Starting RPM ="+ Robot.shooterSubSys.getFlywheelRPM());
            modeState = ModeState.AUTO_10Foot;
        }
        System.out.println(line);
        Robot.logger.appendLog(line); 

        shooterState = ShooterState.STARTUP;

        // Check to see if flywheel is already turned on
        if (Robot.shooterSubSys.getFlywheelState() != FlywheelState.ON) {
            // no pre spin up done so assume High target since this is used mostly
            goalTarget = GoalTarget.HIGH;
            targetRPM = Robot.shooterSubSys.HIGH_GOAL_SPEED;
            Robot.shooterSubSys.flywheelSetOn(targetRPM, Robot.shooterSubSys.HIGH_GOAL);
            shooterState = ShooterState.UNDER_SPEED;
            return;
        }

        // Determine High or Low goal and lookup target RPM
        if (Robot.shooterSubSys.getFlywheelTarget() == FlywheelGoalTarget.HIGH) {
            // If target is High goal
            targetRPM = Robot.shooterSubSys.HIGH_GOAL_SPEED;
            line = ("   Shooting HIGH Goal !");
        } else {
            goalTarget = GoalTarget.LOW;
            targetRPM = Robot.shooterSubSys.LOW_GOAL_SPEED;
            line = ("    Shooting LOW Goal !");
            return; // We dont care about comming up to speed
        }

        // High goal only - Check to see if up to speed
        currentRPM = Robot.shooterSubSys.getFlywheelRPM();
        if ((Math.abs(delta) < Robot.shooterSubSys.PID_FIRE_DELTA)){
            // We are up to speed already
            shooterState = ShooterState.UP_TO_SPEED;  
        } else {
            shooterState = ShooterState.UNDER_SPEED;      
        }
        
        line = line + (" CurrentRPM=" + currentRPM + " TargetRPM=" + targetRPM + " FireDelta=" + Robot.shooterSubSys.PID_FIRE_DELTA);
        System.out.println(line);
        Robot.logger.appendLog(line);  
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        line =  ( "Shooting flywheel CurrentRPM=" + currentRPM + "  Target RPM=" + targetRPM + "  Delta=" + delta);
        //Robot.logger.appendLog(line);  

        if (goalTarget == GoalTarget.LOW) {
            // were going for low gear so just continue shooting balls 
            // no need to wait for things to be up to speed
            Robot.shooterSubSys.handoffMotorSet(HANDOFF_MOTOR_FEED_SPEED);
            Robot.intakeSubSys.mixerByShooterState = MixerByShooterState.FEED;
            //line = ("   LOW Goal shooter is Up to speed !");
            //Robot.logger.appendLog(line);  
            return;
        }
        
        // ------- High Goal --------
        overide = Robot.oi.coDriverJoystick.getRawButton(MANUAL_OVERIDE_BTN);  
        currentRPM = Robot.shooterSubSys.getFlywheelRPM();
        delta = currentRPM-targetRPM;

        if (( firstTimeFlag == 0 ) && (currentRPM < Robot.shooterSubSys.HIGH_GOAL_MIN_SPEED) && (overide==false)) {
            // We are NOT up to speed yet so wait
            Robot.shooterSubSys.handoffMotorStop();
            Robot.intakeSubSys.mixerByShooterState = MixerByShooterState.STOP;
            return;
        }

        // We have reached speed so OK to Fire
        firstTimeFlag = 1;  
        Robot.shooterSubSys.handoffMotorSet(HANDOFF_MOTOR_FEED_SPEED );
        Robot.intakeSubSys.mixerByShooterState = MixerByShooterState.FEED;
        line = ("   HIGH Goal shooter is Up to speed !");
        //Robot.logger.appendLog(line);

        // Were here because we are shooting High goal not during autonomous
        /*
        if (((Math.abs(delta) < Robot.shooterSubSys.PID_FIRE_DELTA)) || (overide)) {
            // were up to speed good to fire
            Robot.shooterSubSys.handoffMotorSet(HANDOFF_MOTOR_FEED_SPEED );
            Robot.intakeSubSys.mixerByShooterState = MixerByShooterState.FEED;
            line = ("   HIGH Goal shooter is Up to speed !");
            Robot.logger.appendLog(line);
        } else {
            // were not in speed 
            Robot.shooterSubSys.handoffMotorStop();
            Robot.intakeSubSys.mixerByShooterState = MixerByShooterState.STOP;
            line = "   HIGH Goal shooter is NOT Up to speed 1";
            Robot.logger.appendLog(line);
            }
        */
        }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        if ((m_mode != 0) && (isTimedOut())){
            // we are shooting by auto we have timedout so end command
            line = "shooterFireCmd has Timed out !!!!!";
            System.out.println(line);
            Robot.logger.appendLog(line);
            return true;
        }
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        //Robot.shooterSubSys.flywheelSetOff();
        Robot.shooterSubSys.handoffMotorStop();
        Robot.intakeSubSys.setMixerByShooterState(MixerByShooterState.STOP);
        line = "shooterFireCmd has ended!";
        System.out.println(line);
        Robot.logger.appendLog(line);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
