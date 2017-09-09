package org.usfirst.frc.team4910.util;

import java.util.ArrayList;
import java.util.HashMap;

import org.usfirst.frc.team4910.robot.OI;
import org.usfirst.frc.team4910.robot.Robot;
import org.usfirst.frc.team4910.robot.RobotMap;
import org.usfirst.frc.team4910.robot.RobotState;
import org.usfirst.frc.team4910.subsystems.DriveTrain;
import org.usfirst.frc.team4910.subsystems.DriveTrain.DriveControlState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 
 * @author Jason Cobb
 * Purpose is to register and sequence setpoints
 */
public class Path {
	public enum PathType{
		Heading, Position;
	}
	//private HashMap<Integer, HashMap<PathType, Double>> pathmap = new HashMap<Integer, HashMap<PathType, Double>>();
	private ArrayList<PathMap> pmap;
	private double PosThresh=7.25;
	public Robot robot;
	private class PathMap{
		private PathType pathType;
		private double setpoint;
		public PathMap(PathType pt, double set){
			pathType=pt;
			setpoint=set;
		}
		public PathType getPathType(){
			return pathType;
		}
		public double getSetpoint(){
			return setpoint;
		}
	}
	public Path(){
		pmap = new ArrayList<PathMap>();
		robot = new Robot();
	}
	public void register(PathType pt, double set){
		//pathmap.put(count, path);
		pmap.add(new PathMap(pt, set));
	}
	public void Iterate(){
		double start = Timer.getFPGATimestamp();
		Robot.drive.resetAll();
		for(PathMap current : pmap){
			if(RobotMap.testerCodeEnabled)
				robot.createNewCSV(); //can't make it static because I make a non static call inside there
			double currStart=Timer.getFPGATimestamp();
			switch(current.getPathType()){
			case Heading:
				//if(!RobotMap.isCompBot){
				if(true){ //this was for testing originally, don't judge
					//					double p=0.0,i=0.0,d=0.0;/*IZoneMin=0.0,IZoneMax=0.0;*/
					//					p=SmartDashboard.getNumber("kGP", RobotMap.driveGyroPID.getP());
					//					i=SmartDashboard.getNumber("kGI", RobotMap.driveGyroPID.getI());
					//					d=SmartDashboard.getNumber("kGD", RobotMap.driveGyroPID.getD());
					//					//IZoneMin=SmartDashboard.getNumber("kGIZoneMin", RobotMap.driveGyroPID.getIZoneMin());
					//					//IZoneMax=SmartDashboard.getNumber("kGIZoneMax", RobotMap.driveGyroPID.getIZoneMax());
					//					RobotMap.driveGyroPID.setPID(p, i, d);
					RobotMap.driveGyroPID.resetIntegrator();
					RobotState.resetPosition();
					Robot.drive.setSetpoint(0);
					System.out.println("Turning to "+current.getSetpoint()+" degrees");
					RobotMap.driveGyroPID.resetIntegrator();
					Robot.drive.setControlState(DriveControlState.velocity);
					Timer.delay(.07);
					//Robot.drive.updatePID();
					Robot.drive.setHeadingSetpoint(current.getSetpoint());
					Timer.delay(.07); //test if needed
					RobotMap.driveGyroPID.setMinimumTimeToRun(1.5*Math.abs(current.getSetpoint()/60.0)); //(degrees) / (max degrees / second) (experimentally gained)
					RobotMap.driveGyroPID.setTolerance(current.getSetpoint()/10.0);
					RobotMap.driveGyroPID.setOutputRange(-1.0, 1.0);
					RobotMap.driveGyroPID.setIZoneRange(current.getSetpoint()/10.0, current.getSetpoint()/4.0);
					//					RobotMap.driveVelocityLeftPID.setOutputRange(-0.5, 0.5);
					//					RobotMap.driveVelocityRightPID.setOutputRange(-0.5, 0.5);

					Timer.delay(.07); //It seems necessary at this point
					while(!RobotMap.driveGyroPID.onTarget()){
						if(!robot.isAutonomous() && RobotMap.isCompBot && !RobotMap.testerCodeEnabled){
							Robot.drive.disableHeadingMode();
							Robot.drive.resetAll();
							System.out.println("Potential error: Path exited early!");
							reset();
							Robot.drive.setControlState(DriveControlState.regular);
							return;
						}
						if(Timer.getFPGATimestamp()-currStart>1.5*Math.abs(current.getSetpoint()/60.0)
								|| (Math.abs(RobotMap.right1.get())<.12 && Math.abs(RobotMap.driveGyroPID.getError())<current.getSetpoint()/10.0 && Timer.getFPGATimestamp()-currStart>0.7*Math.abs(current.getSetpoint()/60.0))){
							//if() statement breaks if it's taken too long or it finishes early
							Timer.delay(.1);
							//System.out.println("End error: "+RobotMap.driveGyroPID.getError()+" Degrees");
							break;
						}
						//System.out.println("Current error: "+RobotMap.driveGyroPID.getError()+" Degrees");
						//Robot.drive.updatePID();
						if(RobotMap.testerCodeEnabled)
							robot.writeAllToCSV();
					}
					//Timer.delay(.1);
					Robot.drive.disableHeadingMode();
					Robot.drive.resetAll();
					System.out.println("Turning took " + (Timer.getFPGATimestamp()-currStart)+" Seconds");
					break;
				}else{
					compbotHeadingRoutine(current.getSetpoint());
					break;
				}
				
			case Position:
				//if(!RobotMap.isCompBot){
				if(true){
					System.out.println("Driving to "+current.getSetpoint()+" inches");


					//				double p=0.0,i=0.0,d=0.0;/*IZoneMin=0.0,IZoneMax=0.0;*/
					//				p=SmartDashboard.getNumber("kP", RobotMap.drivePositionLeftPID.getP());
					//				i=SmartDashboard.getNumber("kI", RobotMap.drivePositionLeftPID.getI());
					//				d=SmartDashboard.getNumber("kD", RobotMap.drivePositionLeftPID.getD());
					//				RobotMap.drivePositionLeftPID.setPID(p, i, d);
					//				RobotMap.drivePositionRightPID.setPID(p, i, d);
					double timethresh= RobotMap.isCompBot ? 5.25 : 6.0;
					Robot.drive.setControlState(DriveControlState.position);
					Timer.delay(.07);
					Robot.drive.setSetpoints(current.getSetpoint(), current.getSetpoint());
					RobotMap.drivePositionLeftPID.setMinimumTimeToRun(timethresh*Math.abs(current.getSetpoint()/RobotMap.leftMaxIPS));
					//(inches) / (max inches / second)
					RobotMap.drivePositionRightPID.setMinimumTimeToRun(timethresh*Math.abs(current.getSetpoint()/RobotMap.rightMaxIPS));
					RobotMap.drivePositionLeftPID.setTolerance(1.5);
					RobotMap.drivePositionRightPID.setTolerance(1.5);
					RobotMap.drivePositionLeftPID.setOutputRange(-1.0, 1.0);
					RobotMap.drivePositionRightPID.setOutputRange(-1.0, 1.0);
					RobotMap.drivePositionLeftPID.setIZoneRange(0.0, 10.0);
					RobotMap.drivePositionRightPID.setIZoneRange(0.0, 10.0);
					Timer.delay(.07);
					double startXPos=RobotState.getPosX();
					double startYPos=RobotState.getPosY();
					while(!RobotMap.drivePositionLeftPID.onTarget() || !RobotMap.drivePositionRightPID.onTarget()){
						if(!robot.isAutonomous() && RobotMap.isCompBot && !RobotMap.testerCodeEnabled){
							Robot.drive.disableHeadingMode();
							Robot.drive.resetAll();
							System.out.println("Potential error: Path exited early!"); //i.e. we're in teleop now
							reset();
							Robot.drive.setControlState(DriveControlState.regular);
							return;
						}
						if(Timer.getFPGATimestamp()-currStart>PosThresh*Math.abs(current.getSetpoint())/(RobotMap.leftMaxIPS) || OI.leftStick.getRawButton(OI.DisablePIDTester)){
							//loop time > 200% min time
							//Boolean algebra gets tricky sometimes
							//System.out.println("End error: "+RobotMap.drivePositionLeftPID.getError());
							break;
						}
						//System.out.println("Current Error: "+(RobotMap.drivePositionLeftPID.getError()+RobotMap.drivePositionRightPID.getError())/2.0);
						//Robot.drive.updatePID();
						if(RobotMap.testerCodeEnabled)
							robot.writeAllToCSV();
					}
					Robot.drive.setControlState(DriveControlState.regular);
					Robot.drive.resetAll();
					System.out.println("Total error: "+Math.hypot(RobotState.getPosX()-startXPos, RobotState.getPosY()-startYPos));
					System.out.println("Driving took " + (Timer.getFPGATimestamp()-currStart)+" Seconds");
					break;
				}else{
					compbotPositionRoutine(current.getSetpoint());
					break;
				}
			}
			Timer.delay(.07);
		}
		System.out.println("Path overall time was "+(Timer.getFPGATimestamp()-start) + " Seconds");
		reset();
		Robot.drive.setControlState(DriveControlState.regular);
		Robot.drive.resetAll();
	}
	/**
	 * This clears all path elements. This shouldn't need to be called externally, but it's there just in case.
	 */
	public synchronized void reset(){
		pmap.clear();
	}
	public synchronized void setPositionTimeThresh(double t){
		PosThresh=t;
	}
	
	/**
	 * We have two problems. One encoder might be bad and we don't know which, and they both respond to changes in voltage differently.
	 * They do, however, respond quickly to instant stops, so we're going to be using dead reckoning for compbot.
	 * 
	 */
	private synchronized void compbotPositionRoutine(double set){
		if(!RobotMap.isCompBot)
			return;
		Robot.drive.setControlState(DriveControlState.regular);
		System.out.println("Driving to position: "+set+" Inches");
		double initialRight=DriveTrain.countsToInches(RobotMap.right1.getEncPosition());
		double initialLeft=initialRight;
		double initialAngle=RobotState.getProtectedHeading();
		Timer.delay(.07);
		double time=Timer.getFPGATimestamp();
		double err=set;
		double out=0.0;
		double absError=Math.abs(err), absOut=Math.abs(out), absSet=Math.abs(set); //Micro-optimization, but there's no harm
		while(Timer.getFPGATimestamp()-time < (absSet>24.0 ? absSet*0.05 : 0.5) && robot.isAutonomous()){
			System.out.println("O"+out+"\nL"+DriveTrain.countsToInches(RobotMap.right1.getEncPosition())+"\nR"+DriveTrain.countsToInches(RobotMap.right1.getEncPosition()));
			err=set-((DriveTrain.countsToInches(RobotMap.right1.getEncPosition())+DriveTrain.countsToInches(RobotMap.right1.getEncPosition()))/2.0);
			absError=Math.abs(err); absSet=Math.abs(set);
			if((absSet>24.0 ? absError<0.05*absSet : absError<1.0)){
				Timer.delay(.05);
			}
			//err=set-((RobotState.getLeftPos()+RobotState.getRightPos())/2.0);
			out=Math.abs(err)>.20*set ? Math.signum(err) : Math.signum(err)*.15; //full output for first 80%, 20% for rest
			out = Math.abs((((-RobotMap.left1.getOutputVoltage()/RobotMap.left1.getBusVoltage())+(RobotMap.right1.getOutputVoltage()/RobotMap.right1.getBusVoltage()))/2.0)-out)>1.0 ? 0.3*out : out; //if dV>1, go to a low output instead of switching polarity and destroying the gears
			absOut=Math.abs(out);
			drive(out, out);
			if((absSet>24.0 ? absError<0.05*absSet: absError<1.0) && absOut<.1){
				break;
			}
		}
		if(!robot.isAutonomous()){
			System.out.println("Potential error: Path exited early!");
		}
		drive(0,0);
		Timer.delay(.07);
		System.out.println("Ending in time: "+(Timer.getFPGATimestamp()-time)
				+"\nEnding error: "+Math.max(Math.abs(initialLeft-RobotState.getFullLeftPos()), Math.abs(initialRight-RobotState.getFullRightPos()))
				+"\nAngular error: "+(RobotState.getProtectedHeading()-initialAngle));
		
	}
	/**
	 * See comments for position routine. This may be unneeded but it's here just in case it isn't.
	 * 
	 */
	private synchronized void compbotHeadingRoutine(double set){
		
		if(!RobotMap.isCompBot)
			return;
		double absSet=Math.abs(set);
		if(absSet<0.5){
			System.out.println("Angle setpoint was less than .5, returning");
			return;
		}
		Robot.drive.setControlState(DriveControlState.regular);
		System.out.println("Driving to angle: "+set+" Degrees");
		double initialLeft=RobotState.getFullLeftPos();
		double initialRight=RobotState.getFullRightPos();
		double initialAngle=RobotState.getProtectedHeading();
		Timer.delay(.07);
		double time=Timer.getFPGATimestamp();
		double err=set;
		double out=0.0;
		double absError=0.0, absOut=0.0; //Micro-optimization, but there's no harm
		//0.218589042452+0.0254944826258*set is just linear regression for angle and time, but we add .3 to account for error
		while(Timer.getFPGATimestamp()-time < (absSet>24.0 ? 0.518589042452+0.0254944826258*absSet : 0.86) && robot.isAutonomous()){
			absSet=Math.abs(set);
			if((absSet>2.0 ? absError<0.04*absSet : absError<2.0)){
				Timer.delay(.05);
			}
			err=set-RobotState.getSpigHeading();
			absError=Math.abs(err);
			out=Math.abs(err)>.20*set ? Math.signum(err) : Math.signum(err)*.15; //full output for first 80%, 20% for rest
			out = Math.abs((((-RobotMap.left1.getOutputVoltage()/RobotMap.left1.getBusVoltage())+(RobotMap.right1.getOutputVoltage()/RobotMap.right1.getBusVoltage()))/2.0)-out)>1.0 ? 0.1*out : out; //if dV>1, go to a low output instead of switching polarity and destroying the gears
			
			absOut=Math.abs(out);
			drive(-out,+out); //I hope this isn't backwards.
												//Incase it is, reverse the sign on the voltage and try,
												//doesn't work, try reversing output in the setSetpoints()
												//doesn't work, try reversing both
			if((absSet>2.0 ? absError<0.04*absSet: absError<2.0) && absOut<.1){
				break;
			}
			
		}
		if(!robot.isAutonomous()){
			System.out.println("Potential error: Path exited early!");
		}
		drive(0,0);
		Timer.delay(.07);
		System.out.println("Ending in time: "+(Timer.getFPGATimestamp()-time)
				+"\nPositional ending error: "+Math.max(Math.abs(initialLeft-RobotState.getFullLeftPos()), Math.abs(initialRight-RobotState.getFullRightPos()))
				+"\nAngular ending error: "+(RobotState.getProtectedHeading()-initialAngle));
	}
	
	
	/**
	 * Too busy to write comments
	 */
	public synchronized void drive(double left, double right){
        	RobotMap.left1.set(-left);
        	RobotMap.right1.set(right);
	}
	
	
}
