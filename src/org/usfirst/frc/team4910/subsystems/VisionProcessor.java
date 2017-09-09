package org.usfirst.frc.team4910.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.usfirst.frc.team4910.iterations.Iterate;
import org.usfirst.frc.team4910.robot.OI;
import org.usfirst.frc.team4910.robot.Robot;
import org.usfirst.frc.team4910.robot.RobotMap;
import org.usfirst.frc.team4910.robot.RobotState;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.VideoProperty;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
/**
 * 
 * @author Jason Cobb
 * 
 * Purpose is to capture an image when triggered and then calculate whatever distances necessary.
 * Because of the delay, it will use timestamps to determine how the data should be applied
 */
@SuppressWarnings("unused")
public class VisionProcessor {
	private static VisionProcessor instance;
	private static final double verticalFOVPeg = 67.0;
	private static final double horizontalFOVPeg = 49.0;
	private static final double resolutionYPeg=320.0; //Our camera is turned on its side, or at least the peg one is
	private static final double resolutionXPeg=240.0;
	private static final double degPerPixelXPeg = horizontalFOVPeg/resolutionXPeg; //240x320
	private static final double degPerPixelYPeg = verticalFOVPeg/resolutionYPeg;
	private static final double centerImgXPeg = 119.5;
	private static final double centerImgYPeg = 159.5;
	
	private static final double verticalFOVBoiler = 49.0;
	private static final double horizontalFOVBoiler = 67.0;
	private static final double resolutionYBoiler=240.0;
	private static final double resolutionXBoiler=320.0;
	private static final double degPerPixelXBoiler = horizontalFOVBoiler/resolutionXBoiler; //320x240
	private static final double degPerPixelYBoiler = verticalFOVBoiler/resolutionYBoiler;
	private static final double centerImgXBoiler = 159.5;
	private static final double centerImgYBoiler = 119.5;
	
	private static final double pegTapeWidth=2.0; //In inches.
	private static final double pegTapeHeight=5.0;
	private static final double pegTapeDistance=6.25; //Distance, in inches, from the tapes
	private static final double tapeArea = 10.0; //Just area of one
	private static final double aspectRatio = pegTapeWidth/pegTapeHeight;
	private static final double heightToPegCenter = 7.0; //In inches
	private static final double minPegContourArea=5.0;
	

	private static final double optimalShootDistance=102.18;
	private static final double optimalShootAngle=86.33; //angle from horizontal, I think. Subject to change.
	private static final double heightToGoalCenter=0.0; //distance in inches to the center of the tapes
	private static final double goalCamFromCenterX=0.0; //from a top view, the base of the triangle from camera position to robot center
	private static final double goalCamFromCenterY=0.0; //height of that same triangle
	/**
	 * 
	 * @param cD calculated distance in inches
	 * @param t Angle in degrees. Use angle calculated from yawAngleToTargetX()
	 * @return transitional distance
	 */
	private static double goalTransitionalDistance(double cD, double t){
		double X=cD*Math.sin(Math.toRadians(t));
		double Y=cD*Math.cos(Math.toRadians(t));
		double ans=Math.hypot(X+goalCamFromCenterX, Y+goalCamFromCenterY);
		return ans;
	}
	/**
	 * 
	 * @param cD calculated distance in inches
	 * @param t Angle in degrees. Use angle calculated from yawAngleToTargetX()
	 * @return transitional angle in degrees
	 */
	private static double goalTransitionalAngle(double cD, double t){
		double X=cD*Math.sin(Math.toRadians(t));
		double Y=cD*Math.cos(Math.toRadians(t));
		return Math.toDegrees(Math.atan((X+goalCamFromCenterX)/(Y+goalCamFromCenterY)));
	}
	
	private static HashMap<String, VideoProperty> propertyMap = new HashMap<String, VideoProperty>();
	private static double yawAngleToTargetApproxX(double error){return (error)*degPerPixelXPeg;}
	private static double yawAngleToTargetApproxY(double error){return (error)*degPerPixelYPeg;}
	private static final double focalLengthX = resolutionXPeg/(2.0*Math.tan(Math.toRadians((horizontalFOVPeg/2.0))));
	private static final double focalLengthY = resolutionYPeg/(2.0*Math.tan(Math.toRadians((verticalFOVPeg/2.0))));
	private static double yawAngleToTarget(double error){return Math.toDegrees(Math.atan((error)/focalLengthX));}
	private static double pitchAngleToTarget(double error){return Math.toDegrees(Math.atan((error)/focalLengthY));}
	private static double DistanceToTarget(double error){return heightToPegCenter*focalLengthY/error;}
	//tan(theta)=height/distance, tan(theta)=err/focalY, distance/height = focalY/err 
	private static double DistanceToTargetApprox(double error){return heightToPegCenter/Math.tan(Math.toRadians(error*degPerPixelYPeg));}
	//distance=height/tan(theta)
	private static VideoCapture videoCapture;
	private static Mat BGR, HSV, blur, threshold, clusters, hierarchy, pegRec, boilerRec;
	private static boolean insideTarget = false;
	private static SerialPort cam;
	private static CvSink cvsPeg, cvsBoiler;
	private static double captureTime;
	private static AxisCamera pegcam, boilercam; //shouldn't be used for much
	private static CvSource pegOutput, boilerOutput;
	private static int iteration=0;
	private static boolean hasEnabled=false;
	private static double currentAngle=0.0;
	private static double currentDistance=0.0;
	private static double calculatedAngle=0.0;
	private static double calculatedDistance=0.0;
	private static double _angle=0.0, _distance=0.0;
	private static Thread visionThread;
	private enum VisionStates {
			Disabled, PegTracking, BoilerTracking;
	}
	private VisionStates visionState=VisionStates.Disabled;
	private static final Scalar 
	//Color values
	BLUE = new Scalar(255, 0, 0),
	GREEN = new Scalar(0, 255, 0),
	RED = new Scalar(0, 0, 255),
	WHITE = new Scalar(255, 255, 255),
	BLACK = new Scalar(0, 0, 0),

	//HSV Threshold
	LOWER_BOUNDS = RobotMap.isCompBot ? new Scalar(65,119,78) : new Scalar(60,83,78),
	UPPER_BOUNDS = RobotMap.isCompBot ? new Scalar(97,255,146) : new Scalar(99,253,203);
	private static Scalar color = BLACK;
	private static double itercount=0.0;
	private static double pegDistSum=0.0;
	private static double pegAngleSum=0.0;
	private final Iterate iter = new Iterate(){
		
		@Override
		public void init() {
			synchronized(VisionProcessor.this){
				visionState=VisionStates.Disabled;
				hasEnabled=false;
				itercount=pegDistSum=pegAngleSum=0.0;
				
			}
		}

		@Override
		public void run() {
			synchronized(VisionProcessor.this){
				switch(visionState){
				case Disabled:
					if(OI.leftStick.getRawButton(OI.StartPegTrackingTest) && RobotMap.testerCodeEnabled){
						//no need to put a while loop, since next iteration it'll go to PegTracking
						startPegTracking();
						//visionThread.start();
					}
//					if(cvsPeg.grabFrame(pegRec) == 0){
//						pegOutput.notifyError(cvsPeg.getError());
//					}
//					if(cvsBoiler.grabFrame(boilerRec) == 0){
//						boilerOutput.notifyError(cvsBoiler.getError());
//					}
//					Imgproc.rectangle(pegRec, new Point(100, 140), new Point(140,180), WHITE);
//					Imgproc.circle(pegRec, new Point(centerImgXPeg, centerImgYPeg), 3, color);
//					pegOutput.putFrame(pegRec);
//					Imgproc.rectangle(boilerRec, new Point(140, 100), new Point(180,140), WHITE);
//					Imgproc.circle(boilerRec, new Point(centerImgXPeg, centerImgYPeg), 3, color);
//					boilerOutput.putFrame(boilerRec);
					break;
				case PegTracking:
					if(OI.leftStick.getRawButton(OI.StopPegTrackingTest) && RobotMap.testerCodeEnabled){
						stopPegTracking();
						break;
					}
					processPeg();
					pegAngleSum+=getCalculatedPegAngle();
					pegDistSum+=getCalculatedPegDistance();
					itercount++;
					break;
				case BoilerTracking:
					break;
				default:
					break;
				}
			}
		}

		@Override
		public void end() {
			synchronized(VisionProcessor.this){
				visionState=VisionStates.Disabled;
				itercount=pegDistSum=pegAngleSum=0.0;
			}
		}
		
	};
	
	public Iterate getLoop(){
		return iter;
	}
	private VisionProcessor(){
		System.load("/usr/local/frc/lib/libopencv_java310.so");
		BGR = new Mat();
		HSV = new Mat();
		blur = new Mat();
		threshold = new Mat();
		clusters = new Mat();
		hierarchy = new Mat();
		pegRec = new Mat();
		boilerRec = new Mat();
		pegcam = CameraServer.getInstance().addAxisCamera("pegcam", RobotMap.PegIP);
		//boilercam = CameraServer.getInstance().addAxisCamera("boilercam", RobotMap.ShooterIP);
		pegcam.setResolution(240, 320);
		//boilercam.setResolution(320, 240);
		cvsPeg = CameraServer.getInstance().getVideo(pegcam);
		//cvsBoiler = CameraServer.getInstance().getVideo(boilercam);
		pegOutput = CameraServer.getInstance().putVideo("PegOut", 240, 320);
		//boilerOutput = CameraServer.getInstance().putVideo("BoilerOut", 320, 240);
		
		
		//TODO: test to see if this still throws an error
//		visionThread = new Thread(() -> {
//			while (!Thread.interrupted() && visionState==VisionStates.PegTracking) {
//				if (cvsPeg.grabFrame(BGR) == 0) {
//					pegOutput.notifyError(cvsPeg.getError());
//					continue;
//				}
//				if(OI.leftStick.getRawButton(OI.StopPegTrackingTest) && RobotMap.testerCodeEnabled){
//					stopPegTracking();
//					break;
//				}
//				processPeg();
//				pegAngleSum+=getCalculatedPegAngle();
//				pegDistSum+=getCalculatedPegDistance();
//				itercount++;
//				// Give the output stream a new image to display
//				pegOutput.putFrame(BGR);
//			}
//		});
//		visionThread.setDaemon(true);
//		visionThread.start();
		
		
	}
	public static VisionProcessor getInstance(){
		return instance==null ? instance=new VisionProcessor() : instance;
	}
	public synchronized void startPegTracking(){
		synchronized(VisionProcessor.this){
			visionState=VisionStates.PegTracking;
			cvsPeg.setEnabled(true);
			Timer.delay(.05); //Delay to make sure it doesn't trip over itself
			hasEnabled=true;
			cvsPeg.grabFrame(BGR); //Get an initial frame so exposure doesn't become a problem
			insideTarget = false;
			pegAngleSum=pegDistSum=itercount=0;
		}
		
	
	}
	public synchronized void stopPegTracking(){
		synchronized(VisionProcessor.this){
			visionState=VisionStates.Disabled;
			itercount=pegDistSum=pegAngleSum=0.0;
			cvsPeg.setEnabled(false);
		}
	}
	private synchronized void processPeg(){
		synchronized(VisionProcessor.this){
			cvsPeg.grabFrame(BGR);
			if(!RobotMap.isCompBot){ //Newer cameras decided to get rid of rotating the camera 90/270 degrees for some stupid reason
				Core.transpose(BGR, BGR);
				Core.flip(BGR, BGR, 0);
				//This rotates by 270 deg
			}
			captureTime=Timer.getFPGATimestamp();
			currentAngle=RobotState.getProtectedHeading();
			currentDistance=(RobotMap.left1.getEncPosition()+RobotMap.right1.getEncPosition())/2.0;
			iteration++;
			ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
			if(!BGR.empty() && !insideTarget){
				//System.out.println("Image not empty");
				Imgproc.cvtColor(BGR, HSV, Imgproc.COLOR_BGR2HSV); //turns image to HSV format
				Core.inRange(HSV, LOWER_BOUNDS, UPPER_BOUNDS, threshold); //applies HSV filter
				Imgproc.findContours(threshold, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE); //Finds contours
				//sort contours based on our current auto mode. Since the for loop just cycles through it in order, it should find the
				//needed pegs in order, and then stop
				if(((String)Robot.autoChoose.getSelected()).contains("Left")){ 
					//sorts from left to right
					contours.sort((MatOfPoint a, MatOfPoint b) -> Imgproc.boundingRect(a).x - Imgproc.boundingRect(b).x);
					
				}else if(((String)Robot.autoChoose.getSelected()).contains("Right")){
					//sorts from right to left
					contours.sort((MatOfPoint a, MatOfPoint b) -> (int)(Imgproc.boundingRect(b).br().x - Imgproc.boundingRect(a).br().x));
					
				} //ignore the else condition
				
				double contA; //contour area


				if(hierarchy.size().height > 0 && hierarchy.size().width > 0 && !insideTarget){
					System.out.println("Contours found");

					Rect rec1=null, rec2=null; //one for each peg
					Rect totalRect=null;
					boolean hasFoundAPeg=false;
					//cycles through contours

					for(int idx = 0; idx >= 0 && !insideTarget; idx = (int) hierarchy.get(0, idx)[0]){
						contA = Imgproc.contourArea(contours.get(idx));
						System.out.println(contA);
						//Imgproc.moments(contours.get(idx)).
						if(contA > minPegContourArea && !insideTarget){
							//System.out.println("Large contours found");
							MatOfPoint approxf1 = new MatOfPoint();
							MatOfPoint2f mMOP2f1 = new MatOfPoint2f(); // Converted contours
							MatOfPoint2f mMOP2f2 = new MatOfPoint2f(); // approxPolyDP stored

							contours.get(idx).convertTo(mMOP2f1, CvType.CV_32FC2);
							Imgproc.approxPolyDP(mMOP2f1, mMOP2f2, 7, true);
							mMOP2f2.convertTo(contours.get(idx), CvType.CV_32S);

							mMOP2f2.convertTo(approxf1, CvType.CV_32S);
							//Imgproc.drawContours(BGR, contours, idx, WHITE);
							if(!hasFoundAPeg){
								rec1 = Imgproc.boundingRect(approxf1);
								Point recMiddle = new Point(rec1.x+.5*rec1.width,rec1.y+.5*rec1.height);
								//Imgproc.circle(BGR, recMiddle, 2, BLUE);
								hasFoundAPeg=true;
							}else{
								rec2 = Imgproc.boundingRect(approxf1);
								Point recMiddle = new Point(rec2.x+.5*rec2.width,rec2.y+.5*rec2.height);
								//Imgproc.circle(BGR, recMiddle, 2, BLUE);
								if(Math.min(rec1.tl().x, rec2.tl().x)==rec1.tl().x){
									totalRect = new Rect(rec1.tl(), rec2.br());
								}else{
									totalRect = new Rect(rec2.tl(), rec1.br());
								}
								//Imgproc.rectangle(BGR, totalRect.br(), totalRect.tl(), GREEN);
								recMiddle = new Point(totalRect.x+.5*totalRect.width,totalRect.y+.5*totalRect.height);
								//Imgproc.circle(BGR, recMiddle, 2, RED);
								break; //no need to go further
							}

						}//end "check size" conditional inside for loop


					}//ends "iterate over contours" loop
					//Imgcodecs.imwrite("/home/lvuser/output"+iteration+".png", BGR);
					try{
						double deltaAngle = RobotState.getProtectedHeading()-currentAngle;
						double deltaDistance = ((RobotMap.left1.getEncPosition()+RobotMap.right1.getEncPosition())/2.0) - currentDistance;
						if(rec2==null && !((String)Robot.autoChoose.getSelected()).contains("middle")){
							calculatedAngle = yawAngleToTarget((rec1.x+.5*rec1.width)-centerImgXPeg) + deltaAngle;
							calculatedDistance = 243.62+157.027*Math.atan(0.0592341*(rec1.y+.5*rec1.height)-12.6384)+deltaDistance;
							calculatedDistance = Math.min(73.0, calculatedDistance);
						}else{
							calculatedAngle = yawAngleToTarget((totalRect.x+.5*totalRect.width)-centerImgXPeg) + deltaAngle;
							calculatedDistance = 243.62+157.027*Math.atan(0.0592341*(totalRect.y+.5*totalRect.height)-12.6384)+deltaDistance;
						}
						//double calculatedAngleY = pitchAngleToTarget((totalRect.y+.5*totalRect.height)-centerImgYPeg);
						//calculatedDistance = DistanceToTarget((totalRect.y+.5*totalRect.height)-centerImgYPeg)+deltaDistance;
						//calculatedDistance = 243.62+157.027*Math.atan(0.0592341*(rec1.y+.5*rec1.height)-12.6384)+deltaDistance;
						//I messed up somewhere in my calculations, but this fits the curve.
						//Data calculated using notepad++ and an (unfortunate) kid looking for something to do
						//I used notepad++ to format the data such that I could plug it into Wolfram Alphas new "development" cloud
						//which is just Mathematica-lite. I used FindFit[] to get some good points. For whatever reason, this failed
						//and gave me some nonsense graph that looked off by a bit. I then plugged the points into desmos.com and plotted
						//the graph WA gave me, and added 20.0 to the curve (it looked close and I just guessed) and it fit perfectly. 
						//System.out.println("Center X coordinate and angle: "+(totalRect.x+.5*totalRect.width)+", "+RobotState.getSpigHeading());
						//System.out.println("Estimated angle and actual: "+calculatedAngle+", "+RobotState.getSpigHeading());
						//System.out.println("Center Y coordinate and distance: "+(totalRect.y+.5*totalRect.height)+", "+RobotState.getLeftPos());
						calculatedAngle = -0.362377+0.736886*calculatedAngle+0.00256394*calculatedAngle*calculatedAngle+0.000025452*calculatedAngle*calculatedAngle*calculatedAngle;
						//another curve regression to account for lens
						if(rec2==null){
							System.out.println("rec2 is null");
							System.out.println("Angle to target X: "+calculatedAngle+" X coord: "+((rec1.x+.5*rec1.width)-centerImgXPeg));
						
							//System.out.println("Angle to target Y: "+calculatedAngleY);
						
							System.out.println("Ground distance to target: "+calculatedDistance+" Y coord: "+((rec1.y+.5*rec1.height)-centerImgYPeg));
						}else{
							System.out.println("Angle to target X: "+calculatedAngle+" X coord: "+((totalRect.x+.5*totalRect.width)-centerImgXPeg));
							
							//System.out.println("Angle to target Y: "+calculatedAngleY);
						
							System.out.println("Ground distance to target: "+calculatedDistance+" Y coord: "+((totalRect.y+.5*totalRect.height)-centerImgYPeg));
						}
						
						//System.out.println("Distance Y: "+calculatedDistance*Math.cos(Math.toRadians(calculatedAngle)));
						_angle=calculatedAngle;
						_distance=calculatedDistance;
						
					}catch(Exception e){
						System.out.println("No target found");
						calculatedAngle=0;
						calculatedDistance=0;
						_angle+=calculatedAngle;
						_distance+=calculatedDistance;
					}
				}//ends "find contours" conditionals

			}//ends "is image even there" conditional
		}//end sync block
	}//end of processing
	/**
	 * 
	 * @return angle to peg
	 */
	public synchronized double getCalculatedPegAngle(){
		return _angle;
	}
	public synchronized double getCalculatedPegDistance(){
		return _distance;
	}
	public synchronized double getAveragePegAngle(){
		return pegAngleSum/itercount;
	}
	public synchronized double getAveragePegDistance(){
		return pegDistSum/itercount;
	}
	public synchronized double getCurrentIteration(){
		return itercount;
	}
}
