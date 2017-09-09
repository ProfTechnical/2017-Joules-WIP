package org.usfirst.frc.team4910.robot;

import org.usfirst.frc.team4910.iterations.Iterate;
import org.usfirst.frc.team4910.subsystems.DriveTrain;
import org.usfirst.frc.team4910.util.ContinuousAngleTracker;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 
 * @author Jason Cobb
 *
 * This is meant to keep track of sensors and robot position, so we aren't recalculating multiple times.
 * While this may seem like a useless micro-optimization at first, implementing this will save both time and load.
 * This will also keep track of all robot parts e.g. shooter speed.
 * 
 * Idea and class name stolen from team 254, but almost none of the class contents are based off their code.
 */
public class RobotState {
	//I keep navX stuff separate just so I can test accuracy.
	//Chances are, we're going to average the results for compbot
	private static double posX=0.0, posY=0.0,velX=0.0,velY=0.0, encAccelX=0.0, encAccelY=0.0, posXNavX=0.0, posYNavX=0.0, lastLeftPos=0.0, lastRightPos=0.0, currentLeftPos=0.0, currentRightPos=0.0;
	private static double lastLeftSpeed=0.0, lastRightSpeed=0.0, leftEncAccel=0.0, rightEncAccel=0.0;
	private static double accelerationX=0.0, accelerationY=0.0, jerkX=0.0, jerkY=0.0; //likely unused
	private static double lastAccelX=0.0, lastAccelY=0.0;
	private static double leftSpeed, rightSpeed=0.0; 
	private static double navxFullHeading=0.0;
	private static double spigHeading=0.0;
	private static double spigHeadingOffset=0.0; //Value heading is subtracted by (when reset)
	private static double protectedSpigHeading=0.0;//This can only be zeroed at the beginning of the match
												//or whenever enabled, otherwise it will always give a full heading
	private static double protectedLeftEnc=0.0, protectedRightEnc=0.0, leftEncOffset=0.0, rightEncOffset=0.0;
	
	////////Following lines are for testing purposes only, they're probably wrong, but I want to test with it////////
	private static double VoltL=0.0, VoltR=0.0, CurrL=0.0, CurrR=0.0, OhmL=0.0, OhmR=0.0, WattL=0.0, WattR=0.0,
			ForceL=0.0, ForceR=0.0, MassL=0.0, MassR=0.0, lastWattL=0.0, lastWattR=0.0, ForceTotal=0.0, MassTotal=0.0, TorqueL=0.0, TorqueR=0.0;
	private static double WattX=0.0, WattY=0.0, WattTotal=0.0;
	
	////////
	
	private static double shooterSpeed=0.0; //Inaccurate. I think we broke the CIMcoder. I wouldn't recommend it anyway.
	private static ContinuousAngleTracker fusedAngle = new ContinuousAngleTracker();
	private static double time;
	private static double iteration=0.0;
	
	protected static final Iterate iter = new Iterate(){

		@Override
		public void init() {
			RobotMap.left1.setEncPosition(0);
			RobotMap.right1.setEncPosition(0);
			RobotMap.spig.reset();
			reset();
			iteration=0.0;
			//fusedAngle.setAngleAdjustment(RobotMap.navxGyro.getFusedHeading());
			posX=posY=velX=velY=encAccelX=encAccelY=posXNavX=posYNavX=lastLeftPos=lastRightPos=currentLeftPos=currentRightPos;
			protectedSpigHeading=spigHeadingOffset=spigHeading=0.0;
			protectedLeftEnc=protectedRightEnc=lastLeftSpeed=lastRightSpeed=leftEncAccel=rightEncAccel=0.0;
			
			////////
			VoltL=VoltR=CurrL=CurrR=OhmL=OhmR=WattL=WattR=ForceL=ForceR=MassL=MassR=lastWattL=lastWattR=ForceTotal=MassTotal=TorqueL=TorqueR=0.0;
			WattX=WattY=WattTotal=0.0;
			////////
			leftEncOffset=rightEncOffset=0.0;
			time = Timer.getFPGATimestamp();
			//TODO: check if we're just now coming out of auto so we can save any valuable coordinate value
			//low priority - auto is the only real time where coordinates matter
		}

		@Override
		public void run() {
			synchronized(this){
				protectedSpigHeading=RobotMap.spig.getAngle();
				protectedLeftEnc = DriveTrain.countsToInches(-RobotMap.left1.getEncPosition());
				protectedRightEnc = DriveTrain.countsToInches(RobotMap.right1.getEncPosition());
				spigHeading=protectedSpigHeading-spigHeadingOffset;
				//fusedAngle.nextAngle(RobotMap.navxGyro.getFusedHeading());
				navxFullHeading = (double)-fusedAngle.getAngle();
				leftSpeed = DriveTrain.rpmToInchesPerSecond(RobotMap.left1.getSpeed());
				rightSpeed = DriveTrain.rpmToInchesPerSecond(RobotMap.right1.getSpeed());
				
				leftEncAccel=(leftSpeed-lastLeftSpeed)/(Timer.getFPGATimestamp()-time);
				rightEncAccel=(rightSpeed-lastRightSpeed)/(Timer.getFPGATimestamp()-time);
				
				currentLeftPos = protectedLeftEnc-leftEncOffset;
				currentRightPos = protectedRightEnc-rightEncOffset;
				//System.out.println("LP: "+currentLeftPos);
				//System.out.println("RP: "+currentRightPos);
				accelerationX = 385.827*RobotMap.RIOAccel.getX(); //inches per second^2
				accelerationY = 385.827*RobotMap.RIOAccel.getY();
				//shooterSpeed = 600.0*(RobotMap.shootControl.getEncVelocity()/80.0);
		        shooterSpeed = RobotMap.isCompBot ? -RobotMap.shootControl.getSpeed() : RobotMap.shootControl.getSpeed();
		        //System.out.println("SS: "+shooterSpeed);
		        
		        
//		        SmartDashboard.putNumber("ShooterSpeed", shooterSpeed);
				//We want it so that relative to the start, x is left and right, y is up and down.
		        //Also, the navX gyro is backwards
		        posY += (Math.cos(Math.toRadians(spigHeading))
		        		*(protectedLeftEnc-lastLeftPos+protectedRightEnc-lastRightPos)/2.0); //I would integrate instead, but for whatever reason
		        																		//position is more accurate than velocity.
		        posX -= (Math.sin(Math.toRadians(spigHeading))
		        		*(protectedLeftEnc-lastLeftPos+protectedRightEnc-lastRightPos)/2.0);
		        encAccelX = (velX-(Math.sin(Math.toRadians(spigHeading))
		        		*(protectedLeftEnc-lastLeftPos+protectedRightEnc-lastRightPos)/2.0))
		        		/Math.pow(Timer.getFPGATimestamp()-time,2);
		        encAccelY = (velY+(Math.cos(Math.toRadians(spigHeading))
		        		*(protectedLeftEnc-lastLeftPos+protectedRightEnc-lastRightPos)/2.0))
		        		/Math.pow(Timer.getFPGATimestamp()-time,2);
		        velX = -(Math.sin(Math.toRadians(spigHeading))
		        		*(protectedLeftEnc-lastLeftPos+protectedRightEnc-lastRightPos)/2.0)
		        		/(Timer.getFPGATimestamp()-time);
		        velY = (Math.cos(Math.toRadians(spigHeading))
		        		*(protectedLeftEnc-lastLeftPos+protectedRightEnc-lastRightPos)/2.0)
		        		/(Timer.getFPGATimestamp()-time);
		        posYNavX += (Math.cos(Math.toRadians(navxFullHeading))
		        		*(protectedLeftEnc-lastLeftPos+protectedRightEnc-lastRightPos)/2.0);
		        posXNavX -= (Math.sin(Math.toRadians(navxFullHeading))
		        		*(protectedLeftEnc-lastLeftPos+protectedRightEnc-lastRightPos)/2.0);
		        lastLeftPos=protectedLeftEnc;
		        lastRightPos=protectedRightEnc;
		        
		        
		        
		        
		        ////////////////////////
		        if(RobotMap.testerCodeEnabled){
		        	VoltL=RobotMap.left1.getOutputVoltage();
		        	VoltR=RobotMap.right1.getOutputVoltage();
		        	CurrL=-RobotMap.left1.getOutputCurrent()-RobotMap.left2.getOutputCurrent();
		        	CurrR=RobotMap.right1.getOutputCurrent()+RobotMap.right2.getOutputCurrent();
		        	OhmL=VoltL/CurrL;
		        	OhmR=VoltR/CurrR;
		        	WattL=VoltL*CurrL;
		        	WattR=VoltR*CurrR;
		        
		        	WattX -= (Math.sin(Math.toRadians(spigHeading))
		        			*(WattL-lastWattL+WattR-lastWattR)/2.0);
		        	WattY += (Math.cos(Math.toRadians(spigHeading))
		        			*(WattL-lastWattL+WattR-lastWattR)/2.0);
		        	lastWattL=WattL;
		        	lastWattR=WattR;
		        	WattTotal=Math.hypot(WattX, WattY);
		        	ForceL=WattL/(0.0254*leftSpeed);
		        	ForceR=WattR/(0.0254*rightSpeed);
		        	ForceTotal=WattTotal/Math.hypot(0.0254*velX, 0.0254*velY);
		        	TorqueL=ForceL*6.5; //This was the point of all that, if you didn't get it before.
		        	TorqueR=ForceR*6.5; //The other stuff like WattX was just to see the full extent at which this will work
		        	
		        	MassL=ForceL/(0.0254*leftEncAccel);
		        	MassR=ForceR/(0.0254*rightEncAccel);
		        	MassTotal=ForceTotal/Math.hypot(0.0254*encAccelX, 0.0254*encAccelY);
		        }
		        ////////////////////////
		        
//		        SmartDashboard.putNumber("PositionX", posX);
//		        SmartDashboard.putNumber("PositionY", posY);
//		        SmartDashboard.putNumber("PositionXNavX", posXNavX);
//		        SmartDashboard.putNumber("PositionYNavX", posYNavX);
//		        SmartDashboard.putNumber("SPIG full heading", protectedSpigHeading);
//		        SmartDashboard.putNumber("SPIG zeroed heading", spigHeading);
				
		        SmartDashboard.putString("Gates status", RobotMap.gates.get().equals(DoubleSolenoid.Value.kForward) ? "Open" : "Closed");
		        
				jerkX=(accelerationX-lastAccelX)/(Timer.getFPGATimestamp()-time); //For whatever reason, NavX just leaves out the whole "dt" part in their collision detector.
				jerkY=(accelerationY-lastAccelY)/(Timer.getFPGATimestamp()-time);
				lastAccelX=accelerationX;
				lastAccelY=accelerationY;
				time = Timer.getFPGATimestamp();
				iteration++;
				//Timer.delay(.04);
			}
		}

		@Override
		public void end() {
		}
		
	};
	
	public static void reset(){
		//fusedAngle.reset();
		resetGyro();
		//RobotMap.navxGyro.reset();
		RobotMap.left1.setEncPosition(0);
		RobotMap.right1.setEncPosition(0);
	}
	//All the getters are automatically generated, thank God.
	public static double getPosX() {
		return posX;
	}

	public static double getPosY() {
		return posY;
	}
	public static double getVelX() {
		return velX;
	}
	public static double getVelY() {
		return velY;
	}
	public static double getEncAccelX() {
		return encAccelX;
	}
	public static double getEncAccelY() {
		return encAccelY;
	}
	public static double getPosXNavX() {
		return posXNavX;
	}

	public static double getPosYNavX() {
		return posYNavX;
	}
	/**
	 * 
	 * @return Left distance in inches
	 */
	public static double getLeftPos() {
		return currentLeftPos;
	}

	/**
	 * 
	 * @return Right distance in inches
	 */
	public static double getRightPos() {
		return currentRightPos;
	}
	
//	public static double getLastLeftPos() {
//		
//	}
//	
//	public static double getLastRightPos() {
//		
//	}
	
	public static double getAccelerationX() {
		return accelerationX;
	}

	public static double getAccelerationY() {
		return accelerationY;
	}

	public static double getJerkX() {
		return jerkX;
	}

	public static double getJerkY() {
		return jerkY;
	}

	public static double getLeftSpeed() {
		return leftSpeed;
	}

	public static double getRightSpeed() {
		return rightSpeed;
	}

	public static double getNavxFullHeading() {
		return navxFullHeading;
	}

	public static double getSpigHeading() {
		return spigHeading;
	}

	public static double getShooterSpeed() {
		return shooterSpeed;
	}
	public static double getIter(){
		return iteration;
	}
	public static void resetGyro(){
		//System.out.println("Gyro has been reset");
		spigHeadingOffset=protectedSpigHeading;
	}
	public static double getProtectedHeading(){
		//"protected" just means you cannot set its value
		return protectedSpigHeading;
	}
	public static void resetPosition(){
		leftEncOffset=protectedLeftEnc;
		rightEncOffset=protectedRightEnc;
	}
	public static double getFullLeftPos(){
		return protectedLeftEnc;
	}
	public static double getFullRightPos(){
		return protectedRightEnc;
	}
	/**
	 * @return Acceleration on the left side in inches / sec^2
	 */
	public static double getLeftEncAccel(){
		return leftEncAccel; 
	}
	/**
	 * @return Acceleration on the right side in inches / sec^2
	 */
	public static double getRightEncAccel(){
		return rightEncAccel;
	}
	
	public static double getVoltL(){
		return VoltL;
	}
	public static double getVoltR(){
		return VoltR;
	}
	public static double getCurrL(){
		return CurrL;
	}
	public static double getCurrR(){
		return CurrR;
	}
	public static double getOhmL(){
		return OhmL;
	}
	public static double getOhmR(){
		return OhmR;
	}
	public static double getForceL(){
		return ForceL;
	}
	public static double getForceR(){
		return ForceR;
	}
	public static double getWattL(){
		return WattL;
	}
	public static double getWattR(){
		return WattR;
	}
	public static double getWattX(){
		return WattX;
	}
	public static double getWattY(){
		return WattY;
	}
	public static double getForceTotal(){
		return ForceTotal;
	}
	public static double getMassTotal(){
		return MassTotal;
	}
	public static double getWattTotal(){
		return WattTotal;
	}
	public static double getTorqueLeft(){
		return TorqueL;
	}
	public static double getTorqueRight(){
		return TorqueR;
	}
}
