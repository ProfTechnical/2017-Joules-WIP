package org.usfirst.frc.team4910.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Joystick;

/**
 * 
 * @author Jason Cobb
 * 
 * Handles operator input
 */
public class OI {
    public static OI instance = null;
    
    
    public static Joystick rightStick;
    public static Joystick leftStick;
    public static Joystick thirdStick;
    
    //Left stick buttons are generally assigned for PID/path tuning, and are mostly temporary
    //Right stick buttons are used mainly for subsystems Matthew has to worry about
    //Third stick buttons are things I don't want Matthew thinking about
    
    //Left Stick buttons
    public static final int PIDTuningSnapshot=1; //uses joystick and sets setpoint in one iteration
    public static final int ReverseDrive=2;
    public static final int EnablePIDTester=3;
    public static final int DisablePIDTester=4;
    public static final int TestMaxAccel=5;
    public static final int shooterPIDTest=6;
    public static final int AutoPathTest=9;
    public static final int StartPegTrackingTest=10;
    public static final int StopPegTrackingTest=11;
    public static final int HeadingPIDTest=12;
    
    
    //Right Stick buttons
    public static final int GearShiftToggle=2;
    public static final int UltrasonicTest=3;
    public static final int Gates=4;
    public static final int PIDDistTest=5;
    public static final int PIDAngleTest=6;
    public static final int AutoTest=7;
    public static final int ResetGyro=8;
    public static final int CompressorToggle=11;

    
    //Third Stick buttons
    
    public static final int ShooterToggle=1;
    public static final int ElevatorToggle=2;
    public static final int climberForwardToggle=3;
    public static final int climberBackwardToggle=4;
    public static final int cameraDebugTest=7;
    public static final int forwardAutoTest=8;
    
    //public static HashMap<Joystick, Integer> buttonMap = new HashMap<Joystick, Integer>(); //Possibly implement later
    //TODO: either implement a button hashmap or implement something else to keep track of buttons and their respective joysticks
    
    public OI(){
        rightStick = new Joystick(2);
        leftStick = new Joystick(1); 
        thirdStick = new Joystick(3);
    	
        
        
        
    }
    
    
    public static OI getInstance(){
    	return instance == null ? instance = new OI() : instance;
    }
    
}
