// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package robot;

import robot.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.*;
import robot.subsystems.*;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);

    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.

    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:

    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());

    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());

    // Start the command when the button is released  and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());


    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
public JoystickButton shooterByJoystickBtn;
public Joystick driverJoystick;
public JoystickButton shooterAimByJoystickBtn;
public JoystickButton climbActivateBrakeBtn;
public JoystickButton climbReleaseBrakeBtn;
public JoystickButton climbByJoystickBtn;
public JoystickButton colorWheelThreeRotateBtn;
public JoystickButton colorWheelRotateColorBtn;
public JoystickButton colorWheelRaiseBt;
public JoystickButton coloWheelLowerBtn;
public Joystick coDriverJoystick;
public JoystickButton intakeCaptureOnBtn;
public JoystickButton intakeCaptureOffBtn;
public Joystick driverXboxCtlr;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public OI() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

driverXboxCtlr = new Joystick(2);

intakeCaptureOffBtn = new JoystickButton(driverXboxCtlr, 5);
intakeCaptureOffBtn.whenPressed(new intakeCaptureOffCmdGrp());
intakeCaptureOnBtn = new JoystickButton(driverXboxCtlr, 4);
intakeCaptureOnBtn.whenPressed(new intakeCaptureOnCmdGrp());
coDriverJoystick = new Joystick(1);

coloWheelLowerBtn = new JoystickButton(coDriverJoystick, 1);
coloWheelLowerBtn.whenPressed(new colorWheelLowerCmd());
colorWheelRaiseBt = new JoystickButton(coDriverJoystick, 1);
colorWheelRaiseBt.whenPressed(new colorWheelRaiseCmd());
colorWheelRotateColorBtn = new JoystickButton(coDriverJoystick, 1);
colorWheelRotateColorBtn.whenPressed(new colorWheelRotateToColorCmd());
colorWheelThreeRotateBtn = new JoystickButton(coDriverJoystick, 1);
colorWheelThreeRotateBtn.whenPressed(new colorWheelRotateThreeTimeCmd());
climbByJoystickBtn = new JoystickButton(coDriverJoystick, 2);
climbByJoystickBtn.whileHeld(new climbByJoystickCmd());
climbReleaseBrakeBtn = new JoystickButton(coDriverJoystick, 4);
climbReleaseBrakeBtn.whenPressed(new climbReleaseBrakeCmd());
climbActivateBrakeBtn = new JoystickButton(coDriverJoystick, 3);
climbActivateBrakeBtn.whenPressed(new climbActivateBrakeCmd());
shooterAimByJoystickBtn = new JoystickButton(coDriverJoystick, 1);
shooterAimByJoystickBtn.whileHeld(new shooterAimByJoystickCmd());
driverJoystick = new Joystick(0);

shooterByJoystickBtn = new JoystickButton(driverJoystick, 3);


        // SmartDashboard Buttons
        SmartDashboard.putData("Autonomous Command", new AutonomousCommand());
        SmartDashboard.putData("shooterCalibrateCmd", new shooterCalibrateCmd());
        SmartDashboard.putData("shooterAimByVisionCmd", new shooterAimByVisionCmd());
        SmartDashboard.putData("shooterFireSingleCmd", new shooterFireSingleCmd());
        SmartDashboard.putData("shooterFireContinousCmd", new shooterFireContinousCmd());
        SmartDashboard.putData("drivePointTurnCmd", new drivePointTurnCmd());
        SmartDashboard.putData("driveForwardCmd", new driveForwardCmd());
        SmartDashboard.putData("driveArcCmd", new driveArcCmd());
        SmartDashboard.putData("intakeValveExtendCmd", new intakeValveExtendCmd());
        SmartDashboard.putData("intakeValveRetractCmd", new intakeValveRetractCmd());
        SmartDashboard.putData("intakeCaptureOnCmdGrp", new intakeCaptureOnCmdGrp());
        SmartDashboard.putData("intakeRollerMotorEjectCmd", new intakeRollerMotorEjectCmd());
        SmartDashboard.putData("intakeRollerMotorRetractCmd", new intakeRollerMotorRetractCmd());
        SmartDashboard.putData("intakeRollerMotorStopCmd", new intakeRollerMotorStopCmd());
        SmartDashboard.putData("intakeCaptureOffCmdGrp", new intakeCaptureOffCmdGrp());
        SmartDashboard.putData("storageHoldCmd", new storageHoldCmd());
        SmartDashboard.putData("storageEjectCmd", new storageEjectCmd());
        SmartDashboard.putData("udpStartServerCmd", new udpStartServerCmd());
        SmartDashboard.putData("udpStopServerCmd", new udpStopServerCmd());
        SmartDashboard.putData("cameraACmdGrp", new cameraACmdGrp());
        SmartDashboard.putData("cameraBCmdGrp", new cameraBCmdGrp());
        SmartDashboard.putData("cameraFlipCmdGrp", new cameraFlipCmdGrp());
        SmartDashboard.putData("ResetGyroCmd", new ResetGyroCmd());
        SmartDashboard.putData("ResetEncodersCmd", new ResetEncodersCmd());
        SmartDashboard.putData("Reset_Robot_Pos_Cmd", new Reset_Robot_Pos_Cmd());
        SmartDashboard.putData("climbReleaseBrakeCmd", new climbReleaseBrakeCmd());
        SmartDashboard.putData("climbActivateBrakeCmd", new climbActivateBrakeCmd());
        SmartDashboard.putData("shooterTestMotorStopCmd", new shooterTestMotorStopCmd());
        SmartDashboard.putData("colorWheelLowerCmd", new colorWheelLowerCmd());
        SmartDashboard.putData("colorWheelRaiseCmd", new colorWheelRaiseCmd());
        SmartDashboard.putData("colorWheelRotateThreeTimeCmd", new colorWheelRotateThreeTimeCmd());
        SmartDashboard.putData("colorWheelRotateToColorCmd", new colorWheelRotateToColorCmd());
        SmartDashboard.putData("colorWheelThreeRotateCmdGrp", new colorWheelThreeRotateCmdGrp());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
public Joystick getdriverJoystick() {
        return driverJoystick;
    }

public Joystick getcoDriverJoystick() {
        return coDriverJoystick;
    }

public Joystick getdriverXboxCtlr() {
        return driverXboxCtlr;
    }


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
}

