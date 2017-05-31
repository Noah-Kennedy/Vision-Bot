package org.usfirst.frc.team4786.robot.subsystems;

import static org.opencv.core.Core.FILLED;
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
import org.usfirst.frc.team4786.robot.VisionLib;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The vision class is used to process and handle the majority of our computer
 * vision
 *
 */
public class Vision extends Subsystem {

	// make all of the instance fields private
	// it will screw up your multithreading if something other than the vision
	// thread tries to directly access something from here
	// don't allow for objects to be publicly accessed
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
	private ArrayList<Double> aspectRatios;
	private ArrayList<Double> solidities;
	private NetworkTable visionTable;
	private NetworkTable testTable;
	private long timestamp;


	/**
	 * The constructor for the Vision subsystem
	 * 
	 * @param streamName
	 *            is the name of the stream on the dashboard
	 * @param cam
	 *            is the device port of the camera
	 */
	public Vision(String streamName, int cam) {

		// instantiate mats, one to keep as the original image to have markers
		// and contours drawn onto it
		// and another for processing
		visionTable = NetworkTable.getTable("visionTable");
		testTable = NetworkTable.getTable("testTable");
		processed = new Mat();
		frame = new Mat();



		// instantiate usb camera to parameterized port
		camera = CameraServer.getInstance().startAutomaticCapture(cam);

		// set camera resolution and FPS (frames per second)
		camera.setResolution(RobotMap.width, RobotMap.height);
		camera.setFPS(RobotMap.cameraFPS);

		// set exposure
		// make this number as big as you can get it while still being able to
		// see stuff
		// if this camera will never be used by drivers for driving, then this
		// should be as high as can be while
		// the rectangles are still visible
		camera.setExposureManual(RobotMap.cameraExposure);

		// instantiate the CvSink, which is where the freshly captured camera
		// stream will be placed
		sink = CameraServer.getInstance().getVideo();

		// instantiate the CvSource, which is where we will put the processed
		// images for the drivers to see
		stream = CameraServer.getInstance().putVideo(streamName, RobotMap.width, RobotMap.height);

		// set up some important constants, like the center pixel of the image
		middleX = (RobotMap.width + 1) / 2;
		middleY = (RobotMap.height + 1) / 2;
		middlePoint = new Point(middleX, middleY);
	}

	// never to be called by the programmer
	// only to be interpreted by wpilib to determine which command to run s
	// default
	public void initDefaultCommand() {

	}

	/**
	 * Grabs the frame from our cvsink and timestamp
	 */
	public void grabFrame() {
		sink.grabFrame(frame);
		//get the timestamp
		//useful for calculations later on
		timestamp = sink.getSource().getLastFrameTime();
	}

	/**
	 * Processes our current frame by applying filters, blurs and contour
	 * finding. The processed data will be overlaid on top of the original
	 * captured frame Synchronized means that any methods wishing to access this
	 * object while a different thread is running this method need to wait their
	 * turn
	 */
	public void process() {
		distances = new ArrayList<Double>();
		horizontalAngles = new ArrayList<Double>();
		verticalAngles = new ArrayList<Double>();
		aspectRatios = new ArrayList<Double>();
		solidities = new ArrayList<Double>();
		
		// stop this madness if the frame is empty
		if (frame.empty())
			return;

		// blurs the image to remove false positives
		Imgproc.GaussianBlur(frame, processed, new Size(9, 9), 3);

		// convert BGR to HSV
		Imgproc.cvtColor(processed, processed, Imgproc.COLOR_BGR2HSV, 0);

		// create scalars if using HSV
		Scalar lowRange = new Scalar(RobotMap.lowHue, RobotMap.lowSat, RobotMap.lowVal);
		Scalar highRange = new Scalar(RobotMap.highHue, RobotMap.highSat, RobotMap.highVal);

		// removes everything not in our filter range
		Core.inRange(processed, lowRange, highRange, processed);

		// morphologies, remove false positives and negatives

		// opening, removes false positives
		/*Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_OPEN,
				Imgproc.getStructuringElement(Imgproc.MORPH_CROSS, new Size(9, 9)));
		
		 // closing, removes false negatives
		Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE,
				Imgproc.getStructuringElement(Imgproc.MORPH_CROSS, new Size(17, 17)));*/
		

		// create an arraylist to hold the unfiltered contours
		ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();

		// find the contours in our image
		findContours(processed, contours, processed, RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

		// list of filtered contours and filtered rectangles
		ArrayList<MatOfPoint> filteredContours = new ArrayList<MatOfPoint>();
		ArrayList<Rect> rects = new ArrayList<Rect>();

		//Filter the contours
		for (MatOfPoint contour : contours) {
			
			// bounding rect objects are rectangles whose bounderies encompass the entirety of the contour
			Rect boundingRect = boundingRect(contour);
			
			//second stage filtration
			if (VisionLib.getPassesAspectRatioTest(boundingRect)
					&& VisionLib.getPassesContourToRectRatio(contour, boundingRect)) {

				// add the contours and bounding rects to our filtered lists
				filteredContours.add(contour);
				rects.add(boundingRect);

				// get the distances
				distances.add(this.getDistanceFromTarget(boundingRect));

				// horizontal
				horizontalAngles.add(findHorizontalAngleToPoint(VisionLib.center(boundingRect)));

				// vertical
				verticalAngles.add(findVerticalAngleToPoint(VisionLib.center(boundingRect)));

				// aspect ratios
				aspectRatios.add(VisionLib.getAspectRatio(boundingRect));

				// solidities
				solidities.add(VisionLib.getSolidity(contour, boundingRect));
			}
		}

		// draw our filtered contours
		drawContours(frame, filteredContours, -1, new Scalar(0, 0xFF, 0), FILLED);

		// figure out how many targets there are
		numTargets = filteredContours.size();

		// draw marker at center of all rects
		if (rects.size() > 0)
			Imgproc.drawMarker(frame, VisionLib.center(rects), new Scalar(0xFF, 0, 0xFF));

		// draw markers to show top left, center and bottom right of each rect
		for (Rect rect : rects) {
			Imgproc.drawMarker(frame, VisionLib.center(rect), new Scalar(0, 0, 0xFF));
			Imgproc.drawMarker(frame, rect.br(), new Scalar(0xFF, 0, 0));
			Imgproc.drawMarker(frame, rect.tl(), new Scalar(0xFF, 0, 0));
		}

		// if the number of targets > 0, find the point in the center of them all
		if (numTargets > 0)
			centerX = VisionLib.center(rects).x;

		// draw a point in the middle of the screen
		Imgproc.drawMarker(frame, middlePoint, new Scalar(0xFF, 0xFF, 0xFF));

	}

	/**
	 * Find the horizontal angle from the normal
	 * 
	 * @param p
	 *            the point of interest
	 * @return the angle from the normal to the point horizontally
	 */
	private double findHorizontalAngleToPoint(Point p) {
		return (p.x - middleX) * RobotMap.degreesPerPixelWidth;
	}

	/**
	 * Find the vertical angle from the normal
	 * 
	 * @param p
	 *            the point of interest
	 * @return the angle from the normal to the point vertically
	 */
	private double findVerticalAngleToPoint(Point p) {
		return (p.y - middleY) * RobotMap.degreesPerPixelHeight;
	}

	/**
	 * Noah's distance algorithm, using angles, not fudge factors and weirdness
	 * 
	 * @param boundingRect
	 * @return the distance of the target
	 */
	private double getDistanceFromTarget(Rect boundingRect) {
		return Math.abs((RobotMap.heightOfTargetInFeet
				+ RobotMap.bottomHeight
				- RobotMap.cameraHeight)
				/ (Math.tan(Math.toRadians(findVerticalAngleToPoint(boundingRect.tl()) - RobotMap.cameraAngle))));
	}

	/**
	 * Prints spatial info to the smart dashboard. This has the horizontal
	 * angles, vertical angles, distances, aspect ratios and solidities for the
	 * first two contours. Doesn't show targets if there are not enough.
	 */
	public void showSpacialInfo() {
		//test for driver station communication
		testTable.putString("Test", "Recieved");
		
		//here is our latency
		SmartDashboard.putNumber("Latency", System.currentTimeMillis() - timestamp);

		//if there are no targets, exit to avoid a null pointer exception
		if (numTargets < 1)
			return;
		SmartDashboard.putNumber("1st target horizontal angle", horizontalAngles.get(0));
		SmartDashboard.putNumber("1st target vertical angle", verticalAngles.get(0));
		SmartDashboard.putNumber("1st target distance", distances.get(0));
		SmartDashboard.putNumber("1st target aspect ratio", aspectRatios.get(0));
		SmartDashboard.putNumber("1st target solidity", solidities.get(0));

		//if there is one target, exit to avoid a null pointer
		if (numTargets < 2)
			return;
		SmartDashboard.putNumber("2nd target horizontal angle", horizontalAngles.get(1));
		SmartDashboard.putNumber("2nd target vertical angle", verticalAngles.get(1));
		SmartDashboard.putNumber("2nd target distance", distances.get(1));
		SmartDashboard.putNumber("2nd target aspect ratio", aspectRatios.get(1));
		SmartDashboard.putNumber("2nd target solidity", solidities.get(1));

	}
	
	/**
	 * Send the spatial info over network tables, not SmartDashboard
	 * For debugging purposes
	 */
	public void sendOverNetworkTables(){
		/*for(String key : visionTable.getKeys()){
			visionTable.delete(key);
		}*/
		if (numTargets < 1)
			return;
		visionTable.putNumber("1st target horizontal angle", horizontalAngles.get(0));
		visionTable.putNumber("1st target vertical angle", verticalAngles.get(0));
		visionTable.putNumber("1st target distance", distances.get(0));
		visionTable.putNumber("1st target aspect ratio", aspectRatios.get(0));
		visionTable.putNumber("1st target solidity", solidities.get(0));

		if (numTargets < 2)
			return;
		visionTable.putNumber("2nd target horizontal angle", horizontalAngles.get(1));
		visionTable.putNumber("2nd target vertical angle", verticalAngles.get(1));
		visionTable.putNumber("2nd target distance", distances.get(1));
		visionTable.putNumber("2nd target aspect ratio", aspectRatios.get(1));
		visionTable.putNumber("2nd target solidity", solidities.get(1));

	}

	/**
	 * Feeds the processed data to the cvsource used as an output stream.
	 */
	public void putFrame() {
		stream.putFrame(frame);
	}

	/**
	 * Returns the number of targets found onscreen
	 * 
	 * @return the number of targets on the screen
	 */
	public int getNumTargets() {
		return numTargets;
	}



	/**
	 * 
	 * @return the distance between the peg and the center of our image
	 */
	public double getOffset() {
		return centerX - middleX;
	}



	/**
	 * Gets the distance for a given filtered rectangle
	 * 
	 * @param index
	 * @return the distance at the index
	 */
	public double getDistance(int index) {
		return distances.get(index);
	}

	/**
	 * Gets the vertical angle for a given filtered rectangle
	 * 
	 * @param index
	 * @return the vertical angle at the index
	 */
	public double getVerticalAngles(int index) {
		return verticalAngles.get(index);
	}

	/**
	 * Gets the horizontal angle for a given filtered rectangle
	 * 
	 * @param index
	 * @return the horizontal angle at the index
	 */
	public double getHorizontalAngles(int index) {
		return horizontalAngles.get(index);
	}

}
