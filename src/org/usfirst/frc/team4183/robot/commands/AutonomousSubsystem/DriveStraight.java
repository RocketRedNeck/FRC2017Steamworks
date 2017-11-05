package org.usfirst.frc.team4183.robot.commands.AutonomousSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.utils.ControlLoop;
import org.usfirst.frc.team4183.utils.LogWriterFactory;
import org.usfirst.frc.team4183.utils.RateLimit;
import org.usfirst.frc.team4183.utils.SettledDetector;

import edu.wpi.first.wpilibj.command.Command;



public class DriveStraight extends Command implements ControlLoop.ControlLoopUser {
		
	// Largest drive that will be applied
	private final double MAX_DRIVE = 1.0;
	
	// Smallest drive that will be applied 
	// (unless error falls within dead zone, then drive goes to 0)
	// THIS MUST BE LARGE ENOUGH TO MOVE THE ROBOT from stopped position
	// if it isn't, you can get stuck in this state.
	// But if this is TOO BIG, you'll get limit cycling, and also get stuck.
	private final double MIN_DRIVE = RobotMap.DRIVESTRAIGHT_MIN_DRIVE;
	
	// Distance at which we reduce drive from MAX to MIN
	private final double MIN_DRIVE_DISTANCE_INCH = 26.0;
	
	// Size of dead zone in inches - also used to determine when done.
	private final double DEAD_ZONE_INCH = 0.5;

	// Dither signal
	private final double DITHER_AMPL = 0.07;
	private final double DITHER_FREQ = 8.0;

	// Settled detector lookback for dead zone
	// I would NOT go lower than 150 because of Java thread jitter
	private final long SETTLED_MSECS = 150;
	
	// Used to determine when done.
	// Also used to detect if we've hit something and stopped short of final distance.
	private final double STOPPED_RATE_IPS = 0.2;
	
	// Settled detector lookback for hangup
	// (must be long compared to dead zone lookback to avoid spurious exit)
	private final long HANGUP_MSECS = 800;
	
	// Limits ramp rate of drive signal
	private final double RATE_LIM_PER_SEC = 3.0;
				
	private final double distanceInch;
	
	private ControlLoop cloop;
	
	private RateLimit rateLimit;
	private SettledDetector settledDetector; 
	private SettledDetector hangupDetector;
	
	// Create a counting system to append to the file name each time
	// command runs; this means the control loop or other logging
	// form can simply use the counter to create uniqueness
	private static long instanceCount = 0;
	
	public DriveStraight( double distanceInch) 
	{		
		requires( Robot.autonomousSubsystem);
		
		this.distanceInch = distanceInch;
		
		DriveStraight.instanceCount++;	// Count each instance so we know which command this was in the recordings
	}

	@Override
	protected void initialize() 
	{
		// Compute setPoint
		double setPoint = distanceInch + Robot.driveSubsystem.getPosition_inch();
		
		// Make helpers
		/// TODO: These could be static since they never change
		rateLimit = new RateLimit( RATE_LIM_PER_SEC);
		settledDetector = new SettledDetector( SETTLED_MSECS, DEAD_ZONE_INCH);
		hangupDetector = new SettledDetector( HANGUP_MSECS, STOPPED_RATE_IPS);
						
		// Put DriveSubsystem into "Align Lock" (drive straight)
        Robot.driveSubsystem.setAlignDrive(true);
		
		// Fire up the loop at 100 Hz and record the control loop setpoint and feedback
        /// TODO: Truthfully I don't like this approach (lazy initialization patterns)
        /// I would create persistent loops in the subsystem that have enable/disable 
        /// (blocking logic) along with settings to be adjusted by the various commands. 
        /// Why? Simply because we waste time and create more non-determinism in a system 
        /// that creates/destroys objects as it goes.
        /// If we were constantly calling a drive straight command for various reason then
        /// eventually a garbage collection cycle will cause a hiccup that just adds uncertainty
        /// to a world that prefers deterministic responses.
		cloop = new ControlLoop( this, 
				                 setPoint, 
				                 10, 
				                 String.format("DriveStraight_%d", DriveStraight.instanceCount));
		cloop.setLoggingKey("DriveStraight");	// Displays this key on dashboard AND writes files (set to "" to disable)
		cloop.start();
	}
		
	@Override
	protected boolean isFinished() 
	{
		
		if( settledDetector.isSettled()
			&&
			Robot.driveSubsystem.getFwdVelocity_ips() < STOPPED_RATE_IPS
		) {
			return true;
		}
		
		if( hangupDetector.isSettled()) {
			return true;
		}

		return false;
	}
	
	@Override
	protected void end() 
	{
	
		// Don't forget to stop the control loop!
		cloop.stop();
		
		// Put DriveSubsystem out of "Align Lock"
        Robot.driveSubsystem.setAlignDrive(false);
						
		// Set output to zero before leaving
    	Robot.driveSubsystem.doAutoStraight(0.0);
	}
	
	@Override
	protected void interrupted() 
	{
		end();
	}
	
	
	@Override
	public double getFeedback() 
	{
		return Robot.driveSubsystem.getPosition_inch();
	}
	
	@Override
	public void setError( double error) 
	{

		settledDetector.set(error);
		hangupDetector.set( Robot.driveSubsystem.getFwdVelocity_ips());
		
		double x;		
		if( Math.abs(error) <= MIN_DRIVE_DISTANCE_INCH)
		{
			x = Math.signum(error)*MIN_DRIVE;
		}
		else
		{
			x = Math.signum(error)*MAX_DRIVE;
		}
		
		x = rateLimit.f(x);
		
		if( Math.abs(error) <= DEAD_ZONE_INCH)
		{
			x = 0.0;
		}
		else
		{
			x += DITHER_AMPL*ditherSignal();
		}
		
		// Set the output
    	Robot.driveSubsystem.doAutoStraight(x);
	}

	double ditherSignal() 
	{
		return Math.sin( DITHER_FREQ*(2.0*Math.PI)*System.currentTimeMillis()/1000.0);
	}
}
