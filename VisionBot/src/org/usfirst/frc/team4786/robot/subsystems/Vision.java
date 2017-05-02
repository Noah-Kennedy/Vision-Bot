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
import org.usfirst.frc.team4786.robot.commands.VisionRunnable;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;

/**The vision class is used to process and handle the majority of our computer vision
 *
 */
public class Vision extends Subsystem implements PIDSource {

	//make all of the instance fields private
	//it will screw up your multithreading if something other than the vision thread tries to directly access something from here
	private Mat processed;
	private Mat frame;
	private UsbCamera camera;
	private CvSink sink;
	private CvSource stream;
	private double center;
	private int numTargets;
	private int middle;

	public Vision(String streamName, int cam) {
		processed = new Mat();
		frame = new Mat();
		camera = CameraServer.getInstance().startAutomaticCapture(cam);
		camera.setResolution(RobotMap.width, RobotMap.height);
		camera.setFPS(15);
		sink = CameraServer.getInstance().getVideo();
		stream = CameraServer.getInstance().putVideo(streamName, RobotMap.width, RobotMap.height);
		middle = RobotMap.width / 2;
	}

	//never to be called by the programmer
	//only to be interpreted by wpilib to determine which command to run s default
	public void initDefaultCommand() {
		//TODO figure out if better multithreaded and comment out default command if necessary
		setDefaultCommand(new VisionRunnable());
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
	 */
	public void process() {
		//blurs the image to remove false positives
		Imgproc.GaussianBlur(frame, processed, new Size(17, 17), 2);

		//we are going to use HSV, not BGR for better filtration
		Imgproc.cvtColor(processed, processed, Imgproc.COLOR_BGR2HSV);
		
		//create scalars to hold high and low thresholds if using BGR
		/*Scalar lowRange = new Scalar(RobotMap.lowBlueValue, RobotMap.lowGreenValue, RobotMap.lowRedValue);
		Scalar highRange = new Scalar(RobotMap.highBlueValue, RobotMap.highGreenValue, RobotMap.highRedValue);*/
		
		//create scalars if using HSV
		Scalar lowRange = new Scalar(RobotMap.lowHue, RobotMap.lowSat, RobotMap.lowVal);
		Scalar highRange = new Scalar(RobotMap.highHue, RobotMap.highSat, RobotMap.highVal);
		
		//removes everything not in our filter range
		Core.inRange(processed, lowRange, highRange, processed);

		//Mat hierarchy = new Mat();
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
			if (boundingRect.height > boundingRect.width && boundingRect.area() > RobotMap.minimumArea)	{
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
		/*if(twoTargets)
			center = midpoint(center(rects.get(0)), center(rects.get(1))).x;*/
		center = center(rects).x;

		// cameraStream.putFrame(mat);
	}

	/**
	 * Feeds the processed data to the cvsource used as an output stream.
	 */
	public void putFrame() {
		stream.putFrame(frame);
	}
	
	public int getNumTargets(){
		int temp = numTargets;
		return temp;
	}

	/**
	 * Takes in two opencv Points and outputs the opencv point at the midpoint
	 * 
	 * @param p1
	 * @param p2
	 * @return the midpoint of the two points
	 */
	public static Point midpoint(Point p1, Point p2) {
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
		return center - middle;
	}

	/**
	 * Takes in an opencv rect object and finds its center
	 * 
	 * @param r
	 * @return the center of the rect
	 */
	public static Point center(Rect r) {
		return midpoint(r.br(), r.tl());
	}
	
	public static Point center(ArrayList<Rect> rects){
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

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return PIDSourceType.kDisplacement;
	}

	@Override
	public double pidGet() {
		return getOffset();
	}
}
