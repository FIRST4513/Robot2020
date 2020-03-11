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
public class autoVisionFwdDriveCmdGrp extends CommandGroup {


    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PARAMETERS
    public autoVisionFwdDriveCmdGrp() {

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

        //addParallel (new driveSetLowGearCmd());
        addParallel (new driveSetLowGearCmd());

        addParallel(new flywheelOnHighCmd()); 

        addSequential(new autoTurretPos0CmdGrp());

        // Shooter aim by vision - mode = 1 look for target align and get out
        //                shooterAimByVisionCmd(int mode, double timeout)
        addSequential(new shooterAimByVisionCmd(      1,            4));  
        
        // Shoot balls for 5 seconds
        //             shooterFireCmd(int mode, double timeout)
        addSequential (new shooterFireCmd( 1,         4.0));
        
        addSequential(new flywheelOffCmd()); 

        // Move off line
        //		 		DriveFwd2Cmd		( tgtDist,   Pwr,     Hdg,   Mode,   TO,   Brake, LimitOverideFlag))
        addSequential(new DriveFwd2Cmd		(  56.0,    1.0,     0.0,    1,     4.0,  false,      false));
        addSequential(new DriveBrakeCmd		(  3.0));  
    } 
}
