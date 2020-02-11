// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package robot.subsystems;


import robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
/**
 *
 */
public class controlPanelSubSys extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

          /**
   * Change the I2C port below to match the connection of your color sensor
   */

  final I2C.Port i2cPort = I2C.Port.kOnboard;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);


    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private DigitalInput contactLimitSwitch;
private WPI_TalonSRX spinnerWheelMotor;
private Solenoid extendRetractValve;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
enum ExtendRetractState {EXTENDED,RETRACTED};
ExtendRetractState extendRetractState = ExtendRetractState.RETRACTED;
double motorSpeed = 0;

private boolean EXTENDVALVEVALUE = true;
private boolean RETRACTVALVEVALUE = false;

    public controlPanelSubSys() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
contactLimitSwitch = new DigitalInput(9);
addChild("contactLimitSwitch",contactLimitSwitch);

        
spinnerWheelMotor = new WPI_TalonSRX(4);


        
extendRetractValve = new Solenoid(0, 3);
addChild("extendRetractValve",extendRetractValve);

        

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    int mCountUp = 0;
    @Override
    public void periodic() {
        // Put code here to be run every loop
       if (mCountUp++ > 10){
            updateSmartdashboard();
            mCountUp = 0;
       }
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public boolean getContactSwitch() {
        return contactLimitSwitch.get();
    }

    public void spinnerTurn(double speed){
        spinnerWheelMotor.set(speed);
        motorSpeed = speed;
    }

    public void spinnerStop() {
        spinnerWheelMotor.set(0);
    }

    public void spinnerExtend() {
        extendRetractValve.set(EXTENDVALVEVALUE);
    }

    public void spinnerRetract() {
        extendRetractValve.set(RETRACTVALVEVALUE);
    }

    private double[] readColor() {
        double detectedRed = m_colorSensor.getRed();
        double detectedBlue = m_colorSensor.getBlue();
        double detectedGreen = m_colorSensor.getGreen();

        double magnitude = java.lang.Math.sqrt( (detectedRed*detectedRed + detectedBlue*detectedBlue + detectedGreen*detectedGreen) );
        double vector[] = { detectedRed/magnitude, detectedGreen/magnitude, detectedBlue/magnitude };
        return vector; 
    }


    public String getSensorColor() {
        
        double redVector[]= {0.63,0.69,0.34}; 
        double greenVector[]= {0.34,0.84,0.42};
        double blueVector[]= {0.29,0.74,0.61}; 
        double yellowVector[]= {0.48,0.84,0.25};

        double detectedColor[] = readColor();

        double redDist = calcColorDist(detectedColor, redVector);
        double blueDist = calcColorDist(detectedColor, blueVector);
        double greenDist = calcColorDist(detectedColor, greenVector);
        double yellowDist = calcColorDist(detectedColor, yellowVector);

        if ( (redDist < blueDist) && (redDist < greenDist) && (redDist < yellowDist) ){
            return "R";
        }
        if ( (greenDist < blueDist) && (greenDist < redDist) && (greenDist < yellowDist) ){
            return "G";
        }
        if ( (blueDist < redDist) && (blueDist < greenDist) && (blueDist < yellowDist) ){
            return "B";
        }
        if ( (yellowDist < blueDist) && (yellowDist < redDist) && (yellowDist < greenDist) ){
            return "Y";
        }

        return "U";
    }

    private double calcColorDist(double detected[],double vector[]){
       double redDistance = detected[0] - vector[0];
       double greenDistance = detected[1] - vector[1];
       double blueDistance = detected[2] - vector[2];
      
       double distanceSquared = redDistance*redDistance + greenDistance*greenDistance + blueDistance*blueDistance;
       return distanceSquared;
    }

    public void updateSmartdashboard(){
        double detectedColor[] = readColor();

        SmartDashboard.putNumber("ColorWheel red", detectedColor[0]);
        SmartDashboard.putNumber("ColorWheel green", detectedColor[1]);
        SmartDashboard.putNumber("ColorWheel blue", detectedColor[2]);

        SmartDashboard.putString("ColorWheel color", getSensorColor());

        SmartDashboard.putNumber("ColorWheel speed",motorSpeed);

        if (extendRetractState == ExtendRetractState.EXTENDED){
            SmartDashboard.putString("ColorWheel State","Extended" );
        }
        else{
            SmartDashboard.putString("ColorWheel State","Retracted" );
        }
    }

    public String getRequiredColor() {

        String gameData;
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        if(gameData.length() > 0){

        switch (gameData.charAt(0)){

            case 'B' : //Blue
            return "B";
            
            case 'G' : //Green
            return "G";

            case 'R' : //Red
            return "R";
    
            case 'Y' : //Yellow 
            return "Y";

            default : //This is corrupt data
            return "U";
  
        }
        } else { //Code for no data received yet
            return "U";
        }
        
    }
}

