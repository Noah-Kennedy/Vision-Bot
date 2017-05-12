package org.usfirst.frc.team4786.robot.subsystems;

import static org.opencv.core.Core.FILLED;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_NONE;
import static org.opencv.imgproc.Imgproc.RETR_LIST;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.drawContours;
import static org.opencv.imgproc.Imgproc.findContours;

import java.util.ArrayList;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team4786.robot.RobotMap;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**The vision class is used to process and handle the majority of our computer vision
 *
 */
public class Vision extends Subsystem {

	//make all of the instance fields private
	//it will screw up your multithreading if something other than the vision thread tries to directly access something from here
	//don't allow for objects to be publicly accessed
	private Mat processed;
	private Mat frame;
	private UsbCamera camera;
	private CvSink sink;
	private CvSource stream;
	private double centerX;
	private double middleY;
	private Point middlePoint;
	private int numTargets;
	private double middleX;
	private ArrayList<Double> distances;
	private ArrayList<Double> horizontalAngles;
	private ArrayList<Double> verticalAngles;

	/**
	 * The constructor for the Vision subsystem
	 * @param streamName is the name of the stream on the dashboard
	 * @param cam is the device port of the camera
	 */
	public Vision(String streamName, int cam) {
		
		//instantiate mats, one to keep as the original image to have markers and contours drawn onto it
		//and another for processing
		processed = new Mat();
		frame = new Mat();
		
		distances = new ArrayList<Double>();
		horizontalAngles = new ArrayList<Double>();
		verticalAngles = new ArrayList<Double>();
		
		//instantiate usb camera to parameterized port
		camera = CameraServer.getInstance().startAutomaticCapture(cam);
		
		//set camera resolution and FPS (frames per second)
		camera.setResolution(RobotMap.width, RobotMap.height);
		camera.setFPS(RobotMap.cameraFPS);
		
		//set exposure
		//make this number as big as you can get it while still being able to see stuff
		//if this camera will never be used by drivers for driving, then this should be as high as can be while
		//the rectangles are still visible
		camera.setExposureManual(RobotMap.cameraExposure);
		
		//instantiate the CvSink, which is where the freshly captured camera stream will be placed
		sink = CameraServer.getInstance().getVideo();
		
		//instantiate the CvSource, which is where we will put the processed images for the drivers to see
		stream = CameraServer.getInstance().putVideo(streamName, RobotMap.width, RobotMap.height);
		
		//set up some important constants, like the center pixel of the image
		middleX = (RobotMap.width + 1) / 2;
		middleY = (RobotMap.height + 1) / 2;
		middlePoint = new Point(middleX,middleY);
	}

	//never to be called by the programmer
	//only to be interpreted by wpilib to determine which command to run s default
	public void initDefaultCommand() {

	}

	/**
	 * Grabs the frame from our cvsink
	 */
	public void grabFrame() {
		sink.grabFrame(frame);
	}

	/**
	 * Processes our current frame by applying filters, blurs and contour finding.
	 * The processed data will be overlaid on top of the original captured frame
	 * Synchronized means that any methods wishing to access this object while a different thread is running this method need to wait their turn
	 */
	public void process() {
		
		/*
		 * We are going to keep the frame mat the same until the end when we add the markers and filtered contours.
		 * We will run all of our processing algorithms on the processed mat.
		 * The processed mat will usually serve as both a destination and source mat.
		 * We will first apply a blur to smooth the image out and remove false positives.
		 * Next we will convert it from BGR to HSV.
		 * We will work in HSV because it filters better in adverse light conditions and at longer distances.
		 * We will try and avoid having large minimum area filters because they impose substantial limits on our range
		 * Next we will filter out all of the pixels not in our HSV range.
		 * Next we will find the contours in our filtered image.
		 * Next we will apply some additional filtering techniques to remove false positives.
		 * Finally we will collect basic data on the image and draw the filtered contours and markers onto our output frame.
		 */
		
		//stop this madness if the frame is empty
		if(frame.empty()) return;
		
		//blurs the image to remove false positives
		Imgproc.GaussianBlur(frame, processed, new Size(17, 17), 3);

		//stop this madness if the processed frame is empty
		//TODO figure out if this is necessary
		//if(processed.empty()) return;
		
		//we are going to use HSV, not BGR for better filtration
		//convert BGR to HSV
		Imgproc.cvtColor(processed, processed, Imgproc.COLOR_BGR2HSV,0);
		
		//create scalars to hold high and low thresholds if using BGR
		//shouldn't be using these values because HSV is better
		/*Scalar lowRange = new Scalar(RobotMap.lowBlueValue, RobotMap.lowGreenValue, RobotMap.lowRedValue);
		Scalar highRange = new Scalar(RobotMap.highBlueValue, RobotMap.highGreenValue, RobotMap.highRedValue);*/
		
		//create scalars if using HSV
		//should be using these because HSV is best
		Scalar lowRange = new Scalar(RobotMap.lowHue, RobotMap.lowSat, RobotMap.lowVal);
		Scalar highRange = new Scalar(RobotMap.highHue, RobotMap.highSat, RobotMap.highVal);
		
		//removes everything not in our filter range
		Core.inRange(processed, lowRange, highRange, processed);
		
		//create an arraylist to hold the unfiltered contours
		ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();

		//find the contours in our image
		findContours(processed, contours, processed, RETR_LIST, CHAIN_APPROX_NONE);

		//list of filtered contours
		ArrayList<MatOfPoint> filteredContours = new ArrayList<MatOfPoint>();
		
		//list of filtered contours as rect objects
		ArrayList<Rect> rects = new ArrayList<Rect>();

		//put our contours into rectangle objects if they pass our conditions
		for (MatOfPoint contour : contours) {
			//bounding rect objects are rectangles whose bounderies encompass all of the contour
			Rect boundingRect = boundingRect(contour);
			//check to see if we are a tallish rectangle with a largish area
			if (boundingRect.height > boundingRect.width &&
					Imgproc.contourArea(contour) / boundingRect.area() > RobotMap.contourToRectPercentage)	{
				
				filteredContours.add(contour);
				rects.add(boundingRect);
				
				//distance finding
				//TODO improve and clean up, possibly rewrite
				double distanceToTarget = ((RobotMap.heightOfTargetInFeet*frame.rows())/
						(boundingRect.height*(.5*RobotMap.cameraFOVHeightInFeet)/RobotMap.distanceAtCalibration))-RobotMap.distanceOfCamFromFrontOfBot;
				//Distance calculations, may need to be tuned
				distanceToTarget += .088;
				distanceToTarget /= 1.886;
				
				distances.add(distanceToTarget);
				
				/*angle finding, uses linear approximation method
				because it's close enough with our camera and
				add them to a list of length numTargets*/
				
				//horizontal
				double angle = findHorizontalAngleToPoint(center(boundingRect));
				horizontalAngles.add(angle);
				
				//vertical
				angle = findVerticalAngleToPoint(center(boundingRect));
				verticalAngles.add(angle);
				
			}
		}

		//draw our contours
		drawContours(frame, filteredContours, -1, new Scalar(0, 0xFF, 0), FILLED);
		//figure out how many targets there are
		numTargets = filteredContours.size();
		
		//draw marker at center of all rects
		if(rects.size() > 0)
			Imgproc.drawMarker(frame, center(rects), new Scalar(0xFF, 0, 0xFF));
		
		//draw markers to show info on each rect
		for (Rect rect : rects) {
			Imgproc.drawMarker(frame, center(rect), new Scalar(0, 0, 0xFF));
			Imgproc.drawMarker(frame, rect.br(), new Scalar(0xFF, 0, 0));
			Imgproc.drawMarker(frame, rect.tl(), new Scalar(0xFF, 0, 0));
		}
		
		//if the number of targets > 0, find the point in the center of them all
		if(numTargets > 0)
			centerX = center(rects).x;
		
		//draw a point in the middle of the screen
		Imgproc.drawMarker(frame, middlePoint, new Scalar(0xFF,0xFF,0xFF));

		
	}
	
	/**
	 * Find the horizontal angle from the normal
	 * @param p the point of interest
	 * @return the angle from the normal to the point horizontally
	 */
	private double findHorizontalAngleToPoint(Point p){
		return (p.x - middleX) * RobotMap.degreesPerPixelWidth;
	}
	
	/**
	 * Find the vertical angle from the normal
	 * @param p the point of interest
	 * @return the angle from the normal to the point vertically
	 */
	private double findVerticalAngleToPoint(Point p){
		return (p.y - middleY) * RobotMap.degreesPerPixelHeight;
	}
	
	/**
	 * print the HSV values of the center pixel to the smart dashboard.
	 * Currently gives incorrect values.
	 */
	public void printHSV(){
		if(frame.empty()) return;
		double[] d = frame.get((int) middleX, (int) middleY);
		SmartDashboard.putNumber("Hue", d[0]);
		SmartDashboard.putNumber("Hue", d[1]);
		SmartDashboard.putNumber("Hue", d[2]);
	}
	
	public void printDistances(){
		System.out.println("There are " + numTargets + " targets");
		System.out.println("Horizontal Angles " + horizontalAngles);
		System.out.println("Vertical Angles " + verticalAngles);
		System.out.println("Distances " + distances);
	}
	

	/**
	 * Feeds the processed data to the cvsource used as an output stream.
	 */
	public void putFrame() {
		stream.putFrame(frame);
	}
	
	/**
	 * Returns the number of targets found onscreen
	 * @return the number of targets on the screen
	 */
	public int getNumTargets(){
		return numTargets;
	}

	/**
	 * Takes in two opencv Points and outputs the opencv point at the midpoint
	 * 
	 * @param p1
	 * @param p2
	 * @return the midpoint of the two points
	 */
	private static Point midpoint(Point p1, Point p2) {
		double x = p1.x + p2.x;
		x /= 2.0;
		double y = p1.y + p2.y;
		y /= 2.0;
		return new Point(x, y);
	}
	
	/**
	 * 
	 * @return the distance between the peg and the center of our image
	 */
	public double getOffset(){
		return centerX - middleX;
	}

	/**
	 * Takes in an opencv rect object and finds its center
	 * 
	 * @param r
	 * @return the center of the rect
	 */
	private static Point center(Rect r) {
		return midpoint(r.br(), r.tl());
	}
	
	/**
	 * Averages all of the top left and bottom right corners together to find the center point
	 * @param rects a list of rect objects
	 * @return the point at the center of the list
	 */
	private static Point center(ArrayList<Rect> rects){
		double x = 0;
		double y = 0;
		for(Rect r : rects){
			x += center(r).x;
			y += center(r).y;
		}
		
		x /= ((double) rects.size());
		y /= ((double) rects.size());
		return new Point(x,y);
	}


}
