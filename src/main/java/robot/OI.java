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

import robot.utils.AxisButton;


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
public Joystick driverPlaystation;
public JoystickButton shooterAimByJoystickBtn;
public JoystickButton climbActivateBrakeBtn;
public JoystickButton climbReleaseBrakeBtn;
public JoystickButton climbByJoystickBtn;
public JoystickButton colorWheelThreeRotateBtn;
public JoystickButton colorWheelRotateColorBtn;
public JoystickButton colorWheelRaiseBt;
public JoystickButton coloWheelLowerBtn;
public JoystickButton shooterAimByVision;
public JoystickButton shooterFireBtn;
public Joystick coDriverJoystick;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    
    AxisButton xBox_Drive_Fwd_Btn;
    AxisButton xBox_Drive_Rev_Btn;
    AxisButton xBox_Drive_LTwist_Btn;
    AxisButton xBox_Drive_RTwist_Btn;

    public OI() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

coDriverJoystick = new Joystick(1);

shooterFireBtn = new JoystickButton(coDriverJoystick, 1);
shooterFireBtn.whileHeld(new shooterFireCmd());
shooterAimByVision = new JoystickButton(coDriverJoystick, 12);
shooterAimByVision.whenPressed(new shooterAimByVisionCmd());
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
shooterAimByJoystickBtn = new JoystickButton(coDriverJoystick, 11);
shooterAimByJoystickBtn.whenPressed(new shooterAimByJoystickCmd());
driverPlaystation = new Joystick(0);



        // SmartDashboard Buttons
        SmartDashboard.putData("Autonomous Command", new AutonomousCommand());
        SmartDashboard.putData("shooterCalibrateCmd", new shooterCalibrateCmd());
        SmartDashboard.putData("shooterAimByVisionCmd", new shooterAimByVisionCmd());
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
        SmartDashboard.putData("climbActivateBrakeCmd", new climbActivateBrakeCmd());
        SmartDashboard.putData("climbReleaseBrakeCmd", new climbReleaseBrakeCmd());
        SmartDashboard.putData("udpStartServerCmd", new udpStartServerCmd());
        SmartDashboard.putData("udpStopServerCmd", new udpStopServerCmd());
        SmartDashboard.putData("cameraACmdGrp", new cameraACmdGrp());
        SmartDashboard.putData("cameraBCmdGrp", new cameraBCmdGrp());
        SmartDashboard.putData("cameraFlipCmdGrp", new cameraFlipCmdGrp());
        SmartDashboard.putData("ResetGyroCmd", new ResetGyroCmd());
        SmartDashboard.putData("ResetEncodersCmd", new ResetEncodersCmd());
        SmartDashboard.putData("Reset_Robot_Pos_Cmd", new Reset_Robot_Pos_Cmd());
        SmartDashboard.putData("shooterTestDriveCmd", new shooterTestDriveCmd());
        SmartDashboard.putData("shooterTestMotorStopCmd", new shooterTestMotorStopCmd());
        SmartDashboard.putData("colorWheelLowerCmd", new colorWheelLowerCmd());
        SmartDashboard.putData("colorWheelRaiseCmd", new colorWheelRaiseCmd());
        SmartDashboard.putData("colorWheelRotateThreeTimeCmd", new colorWheelRotateThreeTimeCmd());
        SmartDashboard.putData("colorWheelRotateToColorCmd", new colorWheelRotateToColorCmd());
        SmartDashboard.putData("colorWheelThreeRotateCmdGrp", new colorWheelThreeRotateCmdGrp());
        SmartDashboard.putData("driveSoftBrakeCmd", new driveSoftBrakeCmd());
        SmartDashboard.putData("shooterTestRPMCmd", new shooterTestRPMCmd());
        SmartDashboard.putData("shooterTestPidStopCmd", new shooterTestPidStopCmd());
        SmartDashboard.putData("shooterFireCmd", new shooterFireCmd());
        SmartDashboard.putData("compressorOnCmd", new compressorOnCmd());
        SmartDashboard.putData("compressorOffCmd", new compressorOffCmd());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    
    /*
    xBox_Drive_Fwd_Btn = new AxisButton(driverXboxCtlr, 0.02, 3);
    xBox_Drive_Fwd_Btn.whenPressed(new driveByJoystickCmd());
    xBox_Drive_Fwd_Btn.whenReleased(new driveSoftBrakeCmd());
    xBox_Drive_Fwd_Btn.whenPressed(new cameraACmdGrp());

    xBox_Drive_Rev_Btn = new AxisButton(driverXboxCtlr, 0.02, 2);
    xBox_Drive_Rev_Btn.whenPressed(new driveByJoystickCmd());
    xBox_Drive_Rev_Btn.whenReleased(new driveSoftBrakeCmd());

    //xBox_Drive_Rev_Btn.whenPressed(new cameraBCmdGrp());

    xBox_Drive_RTwist_Btn = new AxisButton(driverXboxCtlr, 0.1, 4);
    xBox_Drive_RTwist_Btn.whenPressed(new driveByJoystickCmd());
    xBox_Drive_RTwist_Btn.whenReleased(new driveSoftBrakeCmd());  
    xBox_Drive_RTwist_Btn.whenPressed(new cameraACmdGrp());

    xBox_Drive_LTwist_Btn = new AxisButton(driverXboxCtlr, 0.1, 0);
    xBox_Drive_LTwist_Btn.whenPressed(new driveByJoystickCmd());
    xBox_Drive_LTwist_Btn.whenReleased(new driveSoftBrakeCmd());
    xBox_Drive_LTwist_Btn.whenPressed(new cameraACmdGrp());
    */
    }


    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
public Joystick getdriverPlaystation() {
        return driverPlaystation;
    }

public Joystick getcoDriverJoystick() {
        return coDriverJoystick;
    }


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
}

