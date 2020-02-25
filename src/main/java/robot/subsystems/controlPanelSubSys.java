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


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Subsystem;
import robot.Robot;
import robot.subsystems.intakeSubSys.MixerByColorwheelState;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

//import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
//import com.revrobotics.ColorMatchResult;
//import com.revrobotics.ColorMatch;

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

  private double displayCnt = 4;
  final I2C.Port i2cPort = I2C.Port.kOnboard;

  private static final boolean CONTACT_SWITCH_PRESSED = true;
  private enum ContactSwitchState {PRESSED, NOTPRESSED};
  private ContactSwitchState contactSwitchState = ContactSwitchState.NOTPRESSED;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);


    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private DigitalInput contactLimitSwitch;
private Solenoid extendRetractValve;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
enum ExtendRetractState {EXTENDED,RETRACTED};
ExtendRetractState extendRetractState = ExtendRetractState.RETRACTED;
double motorSpeed = 0;

private boolean EXTENDVALVEVALUE = true;
private boolean RETRACTVALVEVALUE = false;

    public controlPanelSubSys() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
contactLimitSwitch = new DigitalInput(6);
addChild("contactLimitSwitch",contactLimitSwitch);

        
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

        updateSmartDashboard();
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public boolean getContactSwitch()       { return contactLimitSwitch.get(); }

    public boolean isContactSwitchPressed(){
        if (contactLimitSwitch.get() == CONTACT_SWITCH_PRESSED) {
            contactSwitchState = ContactSwitchState.PRESSED;
            return true;
        } else {
            contactSwitchState = ContactSwitchState.NOTPRESSED;
            return false;
        }
    }
    public void spinnerExtend() { 
        extendRetractValve.set(EXTENDVALVEVALUE);
        extendRetractState = ExtendRetractState.EXTENDED;
     }
    public void spinnerRetract() {
         extendRetractValve.set(RETRACTVALVEVALUE);
         extendRetractState = ExtendRetractState.RETRACTED;
    }

    public void spinnerTurnByColorSpeed() {
        Robot.intakeSubSys.setMixerByColorwheelState(MixerByColorwheelState.COLORSPEED);
    }
    public void spinnerTurnByRotateSpeed(){
        Robot.intakeSubSys.setMixerByColorwheelState(MixerByColorwheelState.ROTATESPEED);
    }
    public void spinnerStop() {
        Robot.intakeSubSys.setMixerByColorwheelState(MixerByColorwheelState.STOP);
    }

    private double[] readColor() {
    m_colorSensor.getColor();
    
        double detectedRed = m_colorSensor.getRed();
        double detectedBlue = m_colorSensor.getBlue();
        double detectedGreen = m_colorSensor.getGreen();

        double magnitude = java.lang.Math.sqrt( (detectedRed*detectedRed + detectedBlue*detectedBlue + detectedGreen*detectedGreen) );
        double vector[] = { detectedRed/magnitude, detectedGreen/magnitude, detectedBlue/magnitude };
        return vector; 
    }

    String mLastColor = "Y";
    public String getSensorColor() {
        
        double redVector[]= {0.53,0.74,0.41}; 
        double greenVector[]= {0.38,0.81,0.45};
        double blueVector[]= {0.35,0.76,0.55}; 
        double yellowVector[]= {0.46,0.82,0.35};

        double detectedColor[] = readColor();

        double redDist = calcColorDist(detectedColor, redVector);
        double blueDist = calcColorDist(detectedColor, blueVector);
        double greenDist = calcColorDist(detectedColor, greenVector);
        double yellowDist = calcColorDist(detectedColor, yellowVector);

        SmartDashboard.putNumber("GreenDistance", greenDist);

        if ( (redDist < blueDist) && (redDist < greenDist) && (redDist < yellowDist) ){
            mLastColor = "R";
            return "R";
        }
        if ( (greenDist < blueDist) && (greenDist < redDist) && (greenDist < yellowDist) ){
           //if (greenDist < 0.0007)
           if (mLastColor.equals("Y")){
               mLastColor = "Y";
               return "B";
           }
           mLastColor = "G"; 
           return "G";
        }
        if ( (blueDist < redDist) && (blueDist < greenDist) && (blueDist < yellowDist) ){
            mLastColor = "B";
            return "B";
        }
        if ( (yellowDist < blueDist) && (yellowDist < redDist) && (yellowDist < greenDist) ){
            mLastColor = "Y";
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

    public void updateSmartDashboard(){
        if ( (displayCnt % 10) != 0) {
            // only update dashboard every 200 ms
            displayCnt++;
            return;
        }
        displayCnt = 0;

        if (isContactSwitchPressed() == true ){
            SmartDashboard.putString("ColorWheel Contact Switch","Pressed" );
        } else {
                SmartDashboard.putString("ColorWheel Contact Switch","Not Pressed" );
        } 

        if (extendRetractState == ExtendRetractState.EXTENDED){
            SmartDashboard.putString("ColorWheel State","Extended" );
        }
        else{
            SmartDashboard.putString("ColorWheel State","Retracted" );
        }

        //double detectedColor[] = readColor();
        // SmartDashboard.putNumber("ColorWheel red", detectedColor[0]);
        // SmartDashboard.putNumber("ColorWheel green", detectedColor[1]);
        // SmartDashboard.putNumber("ColorWheel blue", detectedColor[2]);
        // SmartDashboard.putString("ColorWheel color", getSensorColor());
    }

    public String getRequiredColor() {
        String gameData;
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        if(gameData.length() > 0){
            switch (gameData.charAt(0)){
                case 'B' : //Blue
                    return "R";
                case 'G' : //Green
                    return "Y";
                case 'R' : //Red
                    return "B";
                case 'Y' : //Yellow 
                    return "G";
                default : //This is corrupt data
                    return "U";
            }
        } else { //Code for no data received yet
            return "U";
        }
    }
}
