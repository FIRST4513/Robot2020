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

import edu.wpi.first.wpilibj.command.CommandGroup;
//import robot.subsystems.*;

/**
 *
 */
public class autoDriveFwdTest1CmdGrp extends CommandGroup {


    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PARAMETERS
    public autoDriveFwdTest1CmdGrp() {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PARAMETERS
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=COMMAND_DECLARATIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=COMMAND_DECLARATIONS
 
    	// ******************************************************************************************
    	// 						Cross-field Left to Right Scale Place Cube							* 
    	// ******************************************************************************************  

    	
    	// ******** UNCOMMENT NEXT 3 COMMANDS PRIOR TO COMPETION ******!!!!!!!
    	// 			      				 		(tgtDist,    Mode,   TO,   Hdg)    	
    	//addSequential(new DriveFwdPidCmd		(  36.0,      1,    5.0,   0.0));

    	// ******** The following line was for testing only ***********
    	//addSequential(new 	DriveFwdPidCmd		(  60.0,      1,    5.0,   0.0));
    	
    	//										( turnAngle,  mode, TO )
    	addSequential(new 	DrivePointTurnPidCmd(  90,        0,    3.0));

    	
    	// ******** UNCOMMENT NEXT 3 COMMANDS PRIOR TO COMPETION ******!!!!!!!    	
    	// 			      				 		(tgtDist,    Mode,   TO,   Hdg)    	
    	//addSequential(new DriveFwdPidCmd		(  24.0,      1,    5.0,   90.0));
    	

        // 			      				 	    	(tgtDist,  Pwr,   Hdg,  Mode,  TO,  Brake, LimitOveride)
    	//addSequential(new 	DriveFwd2Cmd		( -23.0,   0.7,   0.0,   1,    3.0,  false,  false)); 
        
        // 			      				 		(tgtDist,  Pwr,   Hdg,  Mode,  TO,  Brake, LimitOveride)
    	//addSequential(new 	DriveFwd2Cmd		( 23.0,    0.7,   0.0,   1,    3.0,  false,  false));    	

        
    	addSequential(new 	DriveBrakeCmd		(  3.0));
    	

    	 
    	    	
    
    } 
}
