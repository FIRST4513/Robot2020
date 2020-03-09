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
import robot.subsystems.intakeSubSys.MixerByIntakeState;

/**
 *
 */
public class intakeCaptureOnCmd extends Command {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
        private int m_mode;
        private double m_timeout;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public intakeCaptureOnCmd( int mode, double timeout) {
        m_mode = mode;
        m_timeout = timeout;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.intakeSubSys);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.intakeSubSys.extendIntake();      // this drops the intake
        if (m_mode == 1) {
            // this is an automated call so set timeout
            setTimeout(m_timeout);
        }
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        Robot.intakeSubSys.extendIntake();      // this drops the intake
        Robot.intakeSubSys.rollerMotorRetract();
        //Robot.intakeSubSys.setMixerByIntakeState(MixerByIntakeState.FEED);
        
        //Robot.intakeSubSys.SetMixerMotorMIX();

    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        if (m_mode == 0) {
            // we are triggering this by joystick, this keeps running
            return false;
        }

        if (isTimedOut()) {
            // we end when timed out m_mode == 1 auto
        }

        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.intakeSubSys.rollerMotorOff();
        //Robot.intakeSubSys.setMixerMotorOff();
        Robot.intakeSubSys.setMixerByIntakeState(MixerByIntakeState.STOP);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
