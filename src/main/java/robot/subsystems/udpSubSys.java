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


//import robot.commands.*;
//import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.command.Subsystem;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketTimeoutException;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import robot.commands.udpStartServerCmd;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class udpSubSys extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private double visionTargetX = -5; // Placeholder
    private double visionTargetY = 7; // Placeholder
    private double visionTargetDistance = 15; // Placeholder
    private boolean foundVisionTarget = true; // Placeholder
    
    private UdpServer mServer=null ;

    public udpSubSys() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop

        if (isRunning()){
            SmartDashboard.putString("UDP Server Status", "Is Running");
            SmartDashboard.putString("UDP Server Last Rcvd Msg", getLastMessage(false));
        } else {
            SmartDashboard.putString("UDP Server Status", "Is NOT Running");
            SmartDashboard.putString("UDP Server Last Rcvd Msg", "");
        }

        SmartDashboard.putNumber("Vision Target X", visionTargetX);
        SmartDashboard.putNumber("Vision Target Y", visionTargetY);

        if (foundVisionTarget == true) {
            SmartDashboard.putString("Found Vision Target?", "Vision Target Acquired");
        } else {
            SmartDashboard.putString("Found Vision Target?", "No Vision Target");
        }
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    

    // start the server thread on a given port
    public void startUdpServer(int port) {
        System.out.println("Starting Udp server on port " + port) ;
        try {
            mServer = new UdpServer(port);
            mServer.start();
        } catch(IOException e) {
            e.printStackTrace() ;
        }
    }
    
    public void stopUdpServer() {
    	if (mServer != null) {
    		mServer.interrupt();
            System.out.println("Stopping UDP Server");    		
    	}
    	mServer = null ;        // allow garbage collection
    }
    
    public boolean isRunning() {
    	return (mServer != null) ;
    }
    
    public String getLastMessage(boolean clearIt) {
      String temp ;
    	if (mServer != null)
    	   temp = mServer.getDgramString(clearIt) ;
    	else
         temp = "(Not Running)" ;
      return temp ;
    }

    // Vision Target Methods
    public double getVisionTargetX() {
        return visionTargetX;
    }

    public double getVisionTargetY() {
        return visionTargetY;
    }

    public double getVisionTargetDistance() {
        return visionTargetDistance;
    }

    public boolean foundVisionTarget() {
        return foundVisionTarget;
    }
}

// Server class is only visible to this file
// another thread can shut down this server by calling server.interrupt()
class UdpServer extends Thread 
{ 
    private DatagramSocket mSocket;
    // buffer for receiving socket data
    private byte[] mBuf = new byte[256] ;
    // string version of what was received
    private String mDgram = "(No Msg)" ;
 
	
	 // PHS NOTE: port>=1024 to avoid 
    // java.net.BindException: Permission denied
    public UdpServer(int port) throws IOException {
        // open a datagram socket on the indicated port
        // we accept connections from any client on that port
        mSocket = new DatagramSocket(port);
        // allow receive to block for only this amount of msec
        // this allws the trhead to periodically check for interrupted()
        // timeout will fire SocketTimeoutException below, but the socket will remain valid
        mSocket.setSoTimeout(1000);
    }
    
    // another thread can read the datagram by calling this
    // synchronization is not necessary because this thread only fills mBuf with DatagramSocket.receive()
    // which is atomic (or so we're told), as is the assignment to mDgram below
    public String getDgramString(boolean clearIt) {
      String temp ;
      if (clearIt) {
          // we need to do the read and the clear atomically, so don't 
          // let the server thread change it while we're doing that
          synchronized(this) {
              temp = mDgram ;
              mDgram = "" ;
          }
      }
      // but don't do a lock if we don't need to
      else 
          temp = mDgram ;                
      return temp ;
    } 
 
    // we extended thread, so all communication occurs in this run method    
    public void run() {
        while (!isInterrupted()) {
            // create a packet for up to our buffer length
            // if the message is longer than this, it will be truncated
            // if shorter, the packet length will shrink (thus we create a new one each time)
            DatagramPacket packet = new DatagramPacket(mBuf, mBuf.length);
            
            try {
                // Receive the packet. This will block or timeout and should be atomic.
                mSocket.receive(packet);

                // do not change this while an outside thread is doing
                // an atomic read-and-clear
                synchronized(this) {
                    mDgram = new String(packet.getData(), 0, packet.getLength());
                }
                
                // System.out.println("RCVD: " + mDgram) ;
                // we allow a client to shut us down as well
                if (mDgram.equals("*** STOP ***")) {
                    interrupt();   // yes, we can interrupt ourself, which will exit this loop
                    System.out.println("Udp server quitting") ;
                }
            } 
            catch (SocketTimeoutException to) {
                // for a timeout, just make a note and try again
                // having a timeout allows the thread to be interrupted
                // System.out.println("Udp server receive timed out") ;
            }
            catch (IOException e) {
                // for any other exception, maybe we should quit?
                e.printStackTrace() ;
            }
        }
        mSocket.close();
    }
}

