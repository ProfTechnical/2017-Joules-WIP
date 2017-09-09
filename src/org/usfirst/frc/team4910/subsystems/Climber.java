package org.usfirst.frc.team4910.subsystems;

import org.usfirst.frc.team4910.iterations.Iterate;
import org.usfirst.frc.team4910.robot.OI;
import org.usfirst.frc.team4910.robot.RobotMap;

public class Climber {
	private static Climber instance;
	public static Climber getInstance(){
		return instance==null ? instance=new Climber() : instance;
	}
	private final Iterate iter = new Iterate(){

		@Override
		public void init() {
			RobotMap.climbControl.set(0);
		}

		@Override
		public void run() {
			//If this turns out to be backwards, switch the button numbers
			if(OI.thirdStick.getRawButton(OI.climberForwardToggle)){
				RobotMap.climbControl.set(1-RobotMap.climbControl.get()); //if 0, 1, if 1, 0. 
				while(OI.thirdStick.getRawButton(OI.climberForwardToggle));
			}
			if(OI.thirdStick.getRawButton(OI.climberBackwardToggle) && OI.thirdStick.getThrottle() < 0.3){
				RobotMap.climbControl.set(-1-RobotMap.climbControl.get()); //if 0, -1, if -1, 0
				while(OI.thirdStick.getRawButton(OI.climberBackwardToggle));
			}
		}

		@Override
		public void end() {
			RobotMap.climbControl.set(0);
		}
		
	};
	public Iterate getLoop(){
		return iter;
	}
	private Climber(){
		
	}
	
}
