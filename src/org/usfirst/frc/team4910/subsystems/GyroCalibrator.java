package org.usfirst.frc.team4910.subsystems;

import org.usfirst.frc.team4910.iterations.Iterate;
import org.usfirst.frc.team4910.robot.RobotMap;
import org.usfirst.frc.team4910.util.GyroHelper;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 
 * @author Jason Cobb
 * Small class to help calibrate Robot when disabled
 */
public class GyroCalibrator implements Iterate {
	GyroHelper spig = RobotMap.spig;
	double mCalibrationStartTime = 0;
	@Override
	public void init() {
		
	}

	@Override
	public void run() {
        double now = Timer.getFPGATimestamp();
        // Keep re-calibrating the gyro every 5 seconds
        if (now - mCalibrationStartTime > GyroHelper.kCalibrationSampleTime) {
            //SmartDashboard.putBoolean("NAVX Calibrating", RobotMap.navxGyro.isCalibrating());
            spig.endCalibrate();
            //System.out.println("Gyro calibrated, new zero is " + spig.getCenter());
//            SmartDashboard.putNumber("Heading", spig.getAngle());
//            SmartDashboard.putNumber("SPI gyro center", spig.getCenter());
            mCalibrationStartTime = now;
            spig.startCalibrate();
        }
    }

    @Override
    public void end() {
        spig.cancelCalibrate();
    }

}
