package org.usfirst.frc.team4910.subsystems;

import org.usfirst.frc.team4910.iterations.Iterate;
import org.usfirst.frc.team4910.robot.OI;
import org.usfirst.frc.team4910.robot.RobotMap;
import org.usfirst.frc.team4910.robot.RobotState;
import org.usfirst.frc.team4910.util.SynchronousPID;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * 
 * @author Jason Cobb
 * Handles both the shooter and the shooting guider. Guider wheel is meant to feed balls into the shooter for some fixed amount of time
 * and at some fixed velocity;
 * 
 */
public class Shooter {

	private static Shooter instance;
	private enum ShooterState{
		Idle, Loading, Shooting;
	}
	private ShooterState currentState=ShooterState.Idle;
	private double loadingStart=0;
	private static double modifier = .5; //We don't know, we're testing
//	private double shootingStart=0;
//	private double shootKp=0.0;
	//private double setpoint=SmartDashboard.getNumber("ShootSetpoint", 0.0);
	private final Iterate iter = new Iterate(){
		
		@Override
		public void init() {
			RobotMap.shootControl.set(0);
			RobotMap.shootGuide.set(0);
			currentState=ShooterState.Idle;
			loadingStart=0;
			//shootingStart=0;
			//RobotMap.shootPID.setPIDF(0.003, 1.3E-5, 0.0, 8.0E-4);
			RobotMap.shootPID.setPIDF(RobotMap.shooterKp, RobotMap.shooterKi, RobotMap.shooterKd, RobotMap.shooterKf);
		}

		@Override
		public void run() {
			//TODO: fix all this code
			synchronized(Shooter.this){
				Timer.delay(.02);
		    	SmartDashboard.putNumber("ShooterSpeed", RobotState.getShooterSpeed());
		    	SmartDashboard.putNumber("ShootErrorSum", RobotMap.shootPID.getErrorSum());
		    	SmartDashboard.putNumber("ShootSetpoint", RobotMap.shootPID.getSetpoint());
		    	SmartDashboard.putNumber("ShootError", RobotMap.shootPID.getError());
				ShooterState newState;
				//SmartDashboard.putNumber("ShootSetpoint", setpoint);
				//shootKp = SmartDashboard.getNumber("ShootKp", 0.0);
				if(OI.thirdStick.getRawButton(OI.ShooterToggle) && currentState!=ShooterState.Idle){
					while(OI.thirdStick.getRawButton(OI.ShooterToggle)){
						RobotMap.shootControl.set(0);
						RobotMap.shootGuide.set(0);
						RobotMap.shootPID.reset();
						newState=ShooterState.Idle;
						currentState=ShooterState.Idle;
					}
				}
				
				switch(currentState){
				case Idle:
					newState=ShooterState.Idle;
					RobotMap.shootControl.set(0);
					RobotMap.shootGuide.set(0);
					
					if(OI.thirdStick.getRawButton(OI.ShooterToggle) && Math.abs(RobotState.getShooterSpeed())<.2*modifier){ //.2*1200
						RobotMap.shootPID.reset();
						while(OI.thirdStick.getRawButton(OI.ShooterToggle));
						newState = ShooterState.Loading;
						loadingStart=Timer.getFPGATimestamp();
					}
					break;
				case Loading:
					newState = ShooterState.Loading;
					//Get shooter ready
					RobotMap.shootPID.setIZoneRange(0.0, 50.0);
	        		RobotMap.shootPID.resetIntegrator();
					if((Timer.getFPGATimestamp()-loadingStart)<RobotMap.shooterSpinupTime && !OI.thirdStick.getRawButton(OI.ShooterToggle)){
						//RobotMap.shootControl.set((-.60));
						//RobotMap.shootPID.setSetpoint(-0.05*modifier); //.5*1200
						//SmartDashboard.putNumber("ShootSetpoint", -0.05*modifier);
						/**Graham was here =]
		        		**/
	        			//RobotMap.shootControl.set(RobotMap.shootPID.calculate(RobotState.getShooterSpeed()));
						modifier = SmartDashboard.getNumber("ShooterSpeedAdjust", .5);
	        			RobotMap.shootControl.set(-modifier);
						if(OI.thirdStick.getRawButton(OI.ShooterToggle)){
							RobotMap.shootControl.set(0);
							RobotMap.shootGuide.set(0);
							RobotMap.shootPID.reset();
							newState=ShooterState.Idle;
							currentState=ShooterState.Idle;
							break;
									
						}
					}else{
						newState = ShooterState.Shooting;
					}
					if(newState==ShooterState.Shooting){
						//When shooter is ready, load
						RobotMap.shootGuide.set(.75);
						double t = Timer.getFPGATimestamp();
					
						while(Timer.getFPGATimestamp()-t<.5 && !OI.thirdStick.getRawButton(OI.ShooterToggle));
						while(OI.thirdStick.getRawButton(OI.ShooterToggle)){
							RobotMap.shootControl.set(0);
							RobotMap.shootGuide.set(0);
							RobotMap.shootPID.reset();
							newState=ShooterState.Idle;
							currentState=ShooterState.Idle;
						}
					}
					break;
				case Shooting:
					//RobotMap.shootGuide.set(0.0);
					//RobotMap.shootPID.calculate(OI.thirdStick.getY());
					//RobotMap.shootControl.set((OI.thirdStick.getY())+(shootKp*(2600.0*OI.thirdStick.getY()-600.0*(RobotMap.shootControl.getEncVelocity()/80.0))));
					//RobotMap.shootControl.set(OI.thirdStick.getY());
					newState=ShooterState.Shooting;
					RobotMap.shootGuide.set(.75);
					//RobotMap.shootControl.set((-.60));
					/**Graham was also here =]
					**/
					//RobotMap.shootControl.set(RobotMap.shootPID.calculate(RobotState.getShooterSpeed()));
					
					RobotMap.shootControl.set(-modifier);
					if(OI.thirdStick.getRawButton(OI.ShooterToggle)){
						RobotMap.shootPID.reset();
						RobotMap.shootControl.set(0);
						RobotMap.shootGuide.set(0);
						newState=ShooterState.Idle;
						currentState=ShooterState.Idle;
					}
					break;
				default: 
					newState=currentState;
					break;
				}
				if(newState!=currentState){
					System.out.println("Shooter state changed to: "+newState);
					currentState=newState;
					if(newState==ShooterState.Shooting){
						//shootingStart=Timer.getFPGATimestamp();
					}
				}
			}
		}

		@Override
		public void end() {
			RobotMap.shootControl.set(0);
			RobotMap.shootGuide.set(0);
			RobotMap.shootPID.reset();
			currentState=ShooterState.Idle;
		}
		
	};
	
	public Iterate getLoop(){
		return iter;
	}
	private Shooter(){
		
	}
	public static Shooter getInstance(){
		return instance==null ? instance=new Shooter() : instance;
	}
	
	public static double getSpeed(){
		return modifier;
	}
//	private void load(){
//		//Get shooter ready
//		while((Timer.getFPGATimestamp()-loadingStart)<RobotMap.shooterSpinupTime && !OI.thirdStick.getRawButton(OI.ShooterToggle))
//			RobotMap.shootControl.set((-.60));
//		
//		//When shooter is ready, load
//		RobotMap.shootGuide.set(.5);
//		double t = Timer.getFPGATimestamp();
//		while(Timer.getFPGATimestamp()-t<.5 && !OI.thirdStick.getRawButton(OI.ShooterToggle));
//		while(OI.thirdStick.getRawButton(OI.ShooterToggle));
//			//RobotMap.shootPID.calculate(OI.thirdStick.getY()); //TODO: Apply vision tracking
//			//RobotMap.shootControl.set(shootKp*(400.0*OI.thirdStick.getY()-RobotMap.shootControl.getEncVelocity()));
//	}
	
}
