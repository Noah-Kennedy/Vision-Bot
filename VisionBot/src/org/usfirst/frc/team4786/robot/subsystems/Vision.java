package org.usfirst.frc.team4786.robot.subsystems;

import static org.opencv.core.Core.FILLED;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_NONE;
import static org.opencv.imgproc.Imgproc.RETR_LIST;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.drawContours;
import static org.opencv.imgproc.Imgproc.findContours;

import java.util.ArrayList;
import java.util.List;

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

/**
 *
 */
public class Vision extends Subsystem implements PIDSource {

	private Mat processed;
	private Mat frame;
	private UsbCamera camera;
	private CvSink sink;
	private CvSource stream;
	private double center;
	private boolean twoTargets;
	private int numberOfFilteredContours;
	private int middle;

	public Vision() {
		processed = new Mat();
		frame = new Mat();
		camera = CameraServer.getInstance().startAutomaticCapture(0);
		camera.setResolution(RobotMap.width, RobotMap.height);
		camera.setFPS(15);
		sink = CameraServer.getInstance().getVideo();
		stream = CameraServer.getInstance().putVideo("Stream", RobotMap.width, RobotMap.height);
		middle = RobotMap.width / 2;
	}

	//never to be called by the programmer
	//only to be interpreted by wpilib to determine which command to run s default
	public void initDefaultCommand() {
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

		//removes everything not in our filter range
		Scalar lowRange = new Scalar(RobotMap.lowBlueValue, RobotMap.lowGreenValue, RobotMap.lowRedValue);
		Scalar highRange = new Scalar(RobotMap.highBlueValue, RobotMap.highGreenValue, RobotMap.highRedValue);
		Core.inRange(processed, lowRange, highRange, processed);

		Mat hierarchy = new Mat();
		List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

		//find the contours in our image
		findContours(processed, contours, hierarchy, RETR_LIST, CHAIN_APPROX_NONE);

		//list of filtered contours
		ArrayList<MatOfPoint> filteredContours = new ArrayList<MatOfPoint>();
		
		//list of filtered contours as rect objects
		ArrayList<Rect> rects = new ArrayList<Rect>();

		//put our contours into rectangle objects if they pass our conditions
		for (MatOfPoint contour : contours) {
			//bounding rect objects are rectangles whose bounderies encompass all of the contour
			Rect boundingRect = boundingRect(contour);
			if (boundingRect.height > boundingRect.width && boundingRect.area() > RobotMap.minimumArea) {
				filteredContours.add(contour);
				rects.add(boundingRect);
			}
		}

		//draw our contours
		drawContours(frame, filteredContours, -1, new Scalar(0, 0xFF, 0), FILLED);
		//figure out how many targets there are
		numberOfFilteredContours = filteredContours.size();
		//figure out if we have 2 targets
		if(rects.size() == 2) twoTargets = true;
		else twoTargets = false;
		//if we have 2 targets, draw the marker where we think the peg is
		if(twoTargets)
			Imgproc.drawMarker(frame, midpoint(center(rects.get(0)), center(rects.get(1))), new Scalar(0xFF, 0, 0));
		
		//draw markers to show info on each rect
		for (int i = 0; i < rects.size(); i++) {
			Imgproc.drawMarker(frame, center(rects.get(i)), new Scalar(0xFF, 0, 0));
			Imgproc.drawMarker(frame, rects.get(i).br(), new Scalar(0xFF, 0, 0));
			Imgproc.drawMarker(frame, rects.get(i).tl(), new Scalar(0xFF, 0, 0));
		}
		if(twoTargets)
			center = midpoint(center(rects.get(0)), center(rects.get(1))).x;

		// cameraStream.putFrame(mat);
	}

	/**
	 * Feeds the processed data to the cvsource used as an output stream.
	 */
	public void putFrame() {
		stream.putFrame(processed);
	}
	
	public int getNumTargets(){
		int temp = numberOfFilteredContours;
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
