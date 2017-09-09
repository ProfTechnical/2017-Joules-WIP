package org.usfirst.frc.team4910.subsystems;

import org.usfirst.frc.team4910.iterations.Iterate;
import org.usfirst.frc.team4910.robot.OI;
import org.usfirst.frc.team4910.robot.RobotMap;

/**
 * 
 * @author Jason Cobb
 * Handles the ball intake mechanism. Motors go at a speed proportional to the driving motors, and activate and deactivate 
 * at the choice of the operator.
 */
public class Elevator {
	private static Elevator instance;
	public enum ElevatorState{
		Disabled, Running;
	}
	private ElevatorState currentState=ElevatorState.Running;
	private final Iterate iter = new Iterate(){
		
		@Override
		public void init() {
			RobotMap.elevControl.set(0);
			currentState=ElevatorState.Disabled;
		}

		@Override
		public void run() {
			synchronized(Elevator.this){
				switch(currentState){
				case Disabled:
					currentState=ElevatorState.Disabled;
					RobotMap.elevControl.set(0);
					if(OI.thirdStick.getRawButton(OI.ElevatorToggle)){
						while(OI.thirdStick.getRawButton(OI.ElevatorToggle)){
							RunElevator();
						}
						currentState=ElevatorState.Running;
					}
					break;
				case Running:
					currentState=ElevatorState.Running;
					RunElevator();
					if(OI.thirdStick.getRawButton(OI.ElevatorToggle)){
						while(OI.thirdStick.getRawButton(OI.ElevatorToggle)){
							RobotMap.elevControl.set(0);
						}
						currentState=ElevatorState.Disabled;
					}
					
					break;
				default:
					currentState=ElevatorState.Disabled;
					break;
				}
			}
		}

		@Override
		public void end() {
			RobotMap.elevControl.set(0);
			currentState=ElevatorState.Disabled;
		}
		
	};
	
	public Iterate getLoop(){
		return iter;
	}
	private Elevator(){
		
	}
	public ElevatorState getElevatorState(){
		return currentState;
	}
	public static Elevator getInstance(){
		return instance==null ? instance=new Elevator() : instance;
	}
	private synchronized void RunElevator(){
		// o = (b+f) + x
		// o is output, b is the maximum possible motor average, f is the minimum elevator speed
		// x is the average of the two drive motor outputs
		RobotMap.elevControl.set(.5+Math.abs((RobotMap.left1.get()+RobotMap.right1.get())/2.0));
		
	}
}
