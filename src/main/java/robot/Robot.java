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

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.commands.*;
import robot.subsystems.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in 
 * the project.
 */
public class Robot extends TimedRobot {

    Command autonomousCommand;
    Command autoCmd;
    
    SendableChooser<Command> chooser = new SendableChooser<Command>();
    SendableChooser<String> locChooser = new SendableChooser<String>();
    SendableChooser<String> orientChooser = new SendableChooser<String>();
    SendableChooser<String> xBoxLRChooser = new SendableChooser<String>();
    SendableChooser<String> dualChooser = new SendableChooser<String>();  
    SendableChooser<String> firstChooser = new SendableChooser<String>(); 
    SendableChooser<String> rocketChooser = new SendableChooser<String>(); 

    public static Timer sysTimer = new Timer();
	String fmsGameData;
	Boolean switchLeftLit, scaleLeftLit, switchRightLit, scaleRightLit;
	Boolean printBatVoltFlag;
	public static Preferences prefs;
    String line;
    double logCntr = 0;

    // Robot Position States enums
    public static enum RobotPosState {TRAVEL, EJECT_MAIN, EJECT_ROCKET_MID, EJECT_ROCKET_HIGH, EJECT_HATCH_FWD,
                                              RETREIVE_CARGO, RETREIVE_HATCH_FLOOR, RETREIVE_HATCH_LS, 
                                              CARGO_HOLD, HATCH_HOLD, OTHER};

    public static RobotPosState robotPosState = RobotPosState.TRAVEL;

    public static OI oi;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
public static Drivetrain drivetrain;
public static climberSubSys climberSubSys;
public static intakeSubSys intakeSubSys;
public static shooterSubSys shooterSubSys;
public static storageSubSys storageSubSys;
public static controlPanelSubSys controlPanelSubSys;
public static logger logger;
public static udpSubSys udpSubSys;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {

        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
drivetrain = new Drivetrain();
climberSubSys = new climberSubSys();
intakeSubSys = new intakeSubSys();
shooterSubSys = new shooterSubSys();
storageSubSys = new storageSubSys();
controlPanelSubSys = new controlPanelSubSys();
logger = new logger();
udpSubSys = new udpSubSys();

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // OI must be constructed after subsystems. If the OI creates Commands
        //(which it very likely will), subsystems are not guaranteed to be
        // constructed yet. Thus, their requires() statements may grab null
        // pointers. Bad news. Don't move it.
        oi = new OI();

        HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_RobotBuilder);

        // Add commands to Autonomous Sendable Chooser
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

        chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS
        SmartDashboard.putData("Auto mode", chooser);
        userInit();
    }

    /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    @Override
    public void disabledInit(){

    }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = chooser.getSelected();
        // schedule the autonomous command (example)
        if (autonomousCommand != null) autonomousCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        //initPrefs();
        if (autonomousCommand != null) autonomousCommand.cancel();
    }

    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

            
    public void userInit() {
    	Robot.drivetrain.resetGyro();
    	Robot.drivetrain.resetEncodersAndStats();
    	Robot.drivetrain.resetPosition(true); 
    	sysTimer.reset();			// System timer for Competition run
    	sysTimer.start();  
    }

    public void initPrefs(){
        Robot.prefs.putDouble( "/Preferences/Drive_10_PID_Fwd_Tgt", 48);
        Robot.prefs.putDouble( "/Preferences/Drive_10_PID_Fwd_Hdg",0);
        Robot.prefs.putDouble( "/Preferences/Drive_11_PID_Fwd_P",0.076);
        Robot.prefs.putDouble( "/Preferences/Drive_12_PID_Fwd_I",0);
        Robot.prefs.putDouble( "/Preferences/Drive_13_PID_Fwd_D",0.36);
        Robot.prefs.putDouble( "/Preferences/Drive_14_PID_Fwd_F",0);
        Robot.prefs.putDouble( "/Preferences/Drive_15_PID_Fwd_MaxOut",0.80);
        Robot.prefs.putDouble( "/Preferences/Drive_16_PID_Tol_In",1);
        Robot.prefs.putDouble( "/Preferences/Drive_17_PID_Fwd_Drift",0.015);
        Robot.prefs.putDouble( "/Preferences/Drive_18_PID_Fwd_MoveRt",3.6);
        Robot.prefs.putDouble( "/Preferences/Drive_19_PID_Fwd_BB_Lwr",0.38);
        Robot.prefs.putDouble( "/Preferences/Drive_19_PID_Fwd_BB_Upr",0.01);
        Robot.prefs.putDouble( "/Preferences/Drive_20_PID_Rot_Tgt",45);
        Robot.prefs.putDouble( "/Preferences/Drive_21_PID_Rot_P",0.048);
        Robot.prefs.putDouble( "/Preferences/Drive_22_PID_Rot_I",0);
        Robot.prefs.putDouble( "/Preferences/Drive_23_PID_Rot_D",0.14);
        Robot.prefs.putDouble( "/Preferences/Drive_24_PID_Rot_F",0);
        Robot.prefs.putDouble( "/Preferences/Drive_25_PID_Rot_MaxOut",0.75);
        Robot.prefs.putDouble( "/Preferences/Drive_26_PID_Tol_Deg",1);
        Robot.prefs.putDouble( "/Preferences/Drive_30_PID_Rot_BB_Lwr",0.5);
        Robot.prefs.putDouble( "/Preferences/Drive_31_PID_Rot_BB_Upr",0.1);
        Robot.prefs.putDouble( "/Preferences/Drive_32_PID_Rot_MoveRt",3.0);
        Robot.prefs.putDouble( "/Preferences/Drive_50_GyroCor",0.5);
        Robot.prefs.putDouble( "/Preferences/Drive_51_GyroMax",0.5);
        Robot.prefs.putDouble( "/Preferences/Arm_10_PID_Tgt",2.3);
        Robot.prefs.putDouble( "/Preferences/Arm_20_PID_P",1.0);
        Robot.prefs.putDouble( "/Preferences/Arm_21_PID_I",0.0);
        Robot.prefs.putDouble( "/Preferences/Arm_22_PID_D",0.0);
        Robot.prefs.putDouble( "/Preferences/Arm_23_PID_F",0.0);
        Robot.prefs.putDouble( "/Preferences/Arm_30_PID_Tol_In",0.05);
        Robot.prefs.putDouble( "/Preferences/Arm_31_PID_MoveRt",3.6);
        Robot.prefs.putDouble( "/Preferences/Arm_32_BB_Lwr",0.38);
        Robot.prefs.putDouble( "/Preferences/Arm_33_PID_BB_Upr",0.01);
        Robot.prefs.putDouble( "/Preferences/Arm_34_PID_MaxOut",0.80);
        Robot.prefs.putDouble( "/Preferences/Arm_40_Volt_Upper",0.88);
        Robot.prefs.putDouble( "/Preferences/Arm_41_Volt_Lower",3.74);
        Robot.prefs.putDouble( "/Preferences/Arm_50_Raise_Pwr",2.71860);
        Robot.prefs.putDouble( "/Preferences/Arm_51_Lower_Pwr",2.71860);
        Robot.prefs.putDouble( "/Preferences/Arm_52_Hold_Pwr",2.71860);
        Robot.prefs.putDouble( "/Preferences/Elev_10_PID_Tgt",2.3);
        Robot.prefs.putDouble( "/Preferences/Elev_20_PID_P",1.0);
        Robot.prefs.putDouble( "/Preferences/Elev_21_PID_I",0.0);
        Robot.prefs.putDouble( "/Preferences/Elev_22_PID_D",0.0);
        Robot.prefs.putDouble( "/Preferences/Elev_23_PID_F",0.0);
        Robot.prefs.putDouble( "/Preferences/Elev_30_PID_Tol_In",0.05);
        Robot.prefs.putDouble( "/Preferences/Elev_31_PID_MoveRt",3.6);
        Robot.prefs.putDouble( "/Preferences/Elev_32_BB_Lwr",0.38);
        Robot.prefs.putDouble( "/Preferences/Elev_33_PID_BB_Upr",0.01);
        Robot.prefs.putDouble( "/Preferences/Elev_34_PID_MaxOut",0.80);
        Robot.prefs.putDouble( "/Preferences/Elev_40_Volt_Upper",0.441);
        Robot.prefs.putDouble( "/Preferences/Elev_41_Volt_Lower",3.935);
        Robot.prefs.putDouble( "/Preferences/Elev_50_Raise_Pwr",2.71860);
        Robot.prefs.putDouble( "/Preferences/Elev_51_Lower_Pwr",2.71860);
        Robot.prefs.putDouble( "/Preferences/Elev_52_Hold_Pwr",2.71860);
        Robot.prefs.putDouble( "/Preferences/LEDS_10_Test_Value",0.78);
        Robot.prefs.putDouble( "/Preferences/Talon_10_PID_P",0.5);
        Robot.prefs.putDouble( "/Preferences/Talon_11_PID_F",0.5);
        Robot.prefs.putDouble( "/Preferences/Test_10_Fwd2Dist",12.0);
        Robot.prefs.putDouble( "/Preferences/Test_11_Fwd2Pwr",0.5);
        Robot.prefs.putDouble( "/Preferences/Test_12_Fwd2Brake",0.5);
    }
}
