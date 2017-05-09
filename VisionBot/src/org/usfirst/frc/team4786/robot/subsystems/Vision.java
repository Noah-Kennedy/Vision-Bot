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
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**The vision class is used to process and handle the majority of our computer vision
 *
 */
public class Vision extends Subsystem implements PIDSource {

	//make all of the instance fields private
	//it will screw up your multithreading if something other than the vision thread tries to directly access something from here
	//don't allow for objects to be publicly accessed
	private Mat processed;
	private Mat frame;
	private UsbCamera camera;
	private CvSink sink;
	private CvSource stream;
	private double centerX;
	private int middleY;
	private Point middlePoint;
	private int numTargets;
	private int middleX;

	/**
	 * The constructor for the Vision subsystem
	 * @param streamName is the name of the stream on the dashboard
	 * @param cam is the device port of the camera
	 */
	public Vision(String streamName, int cam) {
		processed = new Mat();
		frame = new Mat();
		camera = CameraServer.getInstance().startAutomaticCapture(cam);
		camera.setResolution(RobotMap.width, RobotMap.height);
		camera.setFPS(15);
		camera.setExposureManual(1);
		sink = CameraServer.getInstance().getVideo();
		stream = CameraServer.getInstance().putVideo(streamName, RobotMap.width, RobotMap.height);
		middleX = RobotMap.width / 2;
		middleY = RobotMap.height / 2;
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
		
		//stops the madness if the frame is empty
		if(frame.empty()) return;
		//blurs the image to remove false positives
		Imgproc.GaussianBlur(frame, processed, new Size(17, 17), 3);

		//stop this madness if the processed frame is empty
		if(processed.empty()) return;
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
			if (boundingRect.height > boundingRect.width)	{
				filteredContours.add(contour);
				rects.add(boundingRect);
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
		if(numTargets > 0)
			centerX = center(rects).x;
		
	}
	
	/**
	 * print the HSV values of the center pixel to the smart dashboard.
	 * Currently gives incorrect values.
	 */
	public void printHSV(){
		if(frame.empty()) return;
		double[] d = frame.get(middleX, middleY);
		SmartDashboard.putNumber("Hue", d[0]);
		SmartDashboard.putNumber("Hue", d[1]);
		SmartDashboard.putNumber("Hue", d[2]);
		Imgproc.drawMarker(frame, middlePoint, new Scalar(0xFF,0xFF,0xFF));
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

	/**
	 * Currently source types other than kDisplacement are unsupported, this method does nothing
	 */
	public void setPIDSourceType(PIDSourceType pidSource) {
		
	}

	/**
	 * @return kDisplacement
	 */
	public PIDSourceType getPIDSourceType() {
		return PIDSourceType.kDisplacement;
	}

	/**
	 * Returns value used by PID
	 */
	@Override
	public double pidGet() {
		return getOffset();
	}
}
