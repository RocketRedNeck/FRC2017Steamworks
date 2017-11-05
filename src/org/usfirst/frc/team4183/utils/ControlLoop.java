package org.usfirst.frc.team4183.utils;
import java.io.PrintWriter;

import org.usfirst.frc.team4183.utils.LogWriterFactory;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Yes this is just a re-invention (very simplified) of PIDController.
// I just don't trust the code in PIDController - overly complicated
// and the synchronization looks VERY ad-hoc and buggy. 
// My faith in WPILib is not running real high right now.
// At least when it doesn't work, I'll have nobody else to blame! --tjw

/**
 * Simple Control Loop implementation
 * Useage:
 * 
 * 1) Construct, providing your implementation of ControlLoopUser
 * 2) Call start()
 * 3) ControlLoop will periodically call ControlLoopUser.getFeedback(), you provide the loop feedback value;
 *    and ControlLoopUser.setError(), you use the error value to drive the plant.
 *    Note these calls are happening in a ControlLoop's separate thread.
 * 4) Call stop() when appropriate (you have to determine when that is).
 *    
 * @author twilson
 * 
 * Modified: mkessel
 *
 */
public class ControlLoop {

	private final static long DEFAULT_MSECS = 10;	// Change value in construction to meet control needs  
	private final long msecs;
	private volatile double setPoint;
	
	private volatile String loggingKey = "";

	private final LoopThread loopThread;
	private final ControlLoopUser user;
	
	private LogWriterFactory logFactory;
	private LogWriterFactory.Writer logWriter;
	
	private class LoggerClient implements ThreadLogger.Client 
	{
		@Override
		public void writeLine( PrintWriter writer, long millis) 
		{
			writer.format("%6d %9.1f %9.1f\n", 
					      millis,
					      setPoint,
					      user.getFeedback());
		}
	}
	
	/**
	 * User of ControlLoop must provide an implementation of this interface
	 * @author twilson
	 */
	public interface ControlLoopUser {
		/**
		 * User must provide the loop feedback value
		 * @return The feedback value
		 */
		public double getFeedback();
		
		/**
		 * Give the User the value of loop error
		 * @param error
		 */
		public void setError( double error);
	}
	
	/**
	 * Constructor
	 * @param user  The user of this class
	 * @param setPoint  The loop set point
	 */
	public ControlLoop( ControlLoopUser user, double setPoint, String loggerFileSpec) 
	{
		this( user, setPoint, DEFAULT_MSECS, loggerFileSpec);
	}
	
	/**
	 * Constructor
	 * @param user  The user of this class
	 * @param setPoint  The loop set point
	 * @param msecs  The loop interval
	 * @param loggerFileSpec base file name used by the logger (caller should increment something to clarify records)
	 */
	public ControlLoop( ControlLoopUser user, double setPoint, long msecs, String loggerFileSpec) 
	{
		this.user = user;
		this.setPoint = setPoint;
		this.msecs = msecs;
	
		// Create a logger for control loops, actual use will depend
		// on other flags to enable/disable logging (which can impact control timing
		// if you are not careful on how often it is used)
		logFactory = new LogWriterFactory(loggerFileSpec);
		logWriter = logFactory.create(true);
		
		loopThread = new LoopThread();
		loopThread.setPriority(Thread.NORM_PRIORITY+2);
		
	}
	
	/**
	 * Sets the logging key to display on dashboard
	 * An empty string ("") will disable logging
	 * @param loggingKey
	 */
	public void setLoggingKey( String loggingKey) 
	{
		this.loggingKey = loggingKey;
	}
	
	
	/**
	 * Set the loop setpoint (normally done in constructor)
	 * but can be used to change the setpoint on the fly
	 * @param setPoint
	 */
	public void setSetpoint( double setPoint) 
	{
		this.setPoint = setPoint;
	}
		
	
	/**
	 * Start operation
	 */
	public void start() 
	{
		loopThread.start();
	}
	
	/**
	 * Stop operation
	 * Does not return until loop thread has exited.
	 */
	public void stop() {
		
		// Signal control loop to quit
		loopThread.quit();
		
		// Wait for thread to exit
		try {
			loopThread.join();
		} catch (InterruptedException e) {
			// Per this excellent article:
			// http://www.ibm.com/developerworks/library/j-jtp05236/
			Thread.currentThread().interrupt();
		}
		
		user.setError(0.0);
	}
	
	
	// This Thread implements the control loop
	private class LoopThread extends Thread 
	{
						
		private void quit() 
		{
			interrupt();
		}
		
		@Override
		public void run( ) 
		{
									
			// Loop until signaled to quit
			while( !isInterrupted()) 
			{

				double feedback = user.getFeedback();
				double error = setPoint - feedback;
				
				// TODO: Consider separating dashboard control from file record control
				if( ! loggingKey.equals(""))
				{
					SmartDashboard.putNumber(loggingKey, error);
					
					// Timestamp is automatic at writeLine
				    logWriter.writeLine(String.format("%9.1f %9.1f\n", 
					                                  setPoint, 
					                                  feedback));
				}
					
				user.setError( error);				
				
				// Delay
				/// TODO: sleep does not guarantee a schedule
				/// Need some form of scheduled timer
				try 
				{
					Thread.sleep(msecs);
				} 
				catch (InterruptedException e) 
				{
					interrupt();
				}				
			}			
		}
	}		
}
