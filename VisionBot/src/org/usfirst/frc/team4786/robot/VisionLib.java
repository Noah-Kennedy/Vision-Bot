package org.usfirst.frc.team4786.robot;

import java.util.ArrayList;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

public class VisionLib {
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
	 * Takes in an opencv rect object and finds its center
	 * 
	 * @param r
	 * @return the center of the rect
	 */
	public static Point center(Rect r) {
		return midpoint(r.br(), r.tl());
	}

	/**
	 * Averages all of the top left and bottom right corners together to find
	 * the center point
	 * 
	 * @param rects
	 *            a list of rect objects
	 * @return the point at the center of the list
	 */
	public static Point center(ArrayList<Rect> rects) {
		double x = 0;
		double y = 0;
		for (Rect r : rects) {
			x += center(r).x;
			y += center(r).y;
		}

		x /= ((double) rects.size());
		y /= ((double) rects.size());
		return new Point(x, y);
	}

	// Conditions for contours and rects to pass to not be filtered out

	/**
	 * Tests whether or not we have the correct ratio of bounding rect size to
	 * contour size Basically makes sure that we have the correct amount of
	 * rectangularness
	 * 
	 * @param c
	 *            is the contour
	 * @param r
	 *            is the bounding rectangle
	 * @return whether we are within the ratio interval
	 */
	public static boolean getPassesContourToRectRatio(MatOfPoint c, Rect r) {
		// return true;
		return Imgproc.contourArea(c) / r.area() >= RobotMap.contourToRectLowerPercentage
				&& Imgproc.contourArea(c) / r.area() <= RobotMap.contourToRectUpperPercentage;
	}

	/**
	 * Finds the solidity of the target by dividing contour area by rectangle
	 * area
	 * 
	 * @param c
	 * @param r
	 * @return the solidity of the target
	 */
	public static double getSolidity(MatOfPoint c, Rect r) {
		return Imgproc.contourArea(c) / r.area();
	}

	/**
	 * Figures out whether or not we passed the aspect ratio test
	 * 
	 * @param r
	 * @return whether or not we passed the test
	 */
	public static boolean getPassesAspectRatioTest(Rect r) {
		// return true;
		return getAspectRatio(r) >= RobotMap.lowAspectRatio && getAspectRatio(r) <= RobotMap.highAspectRatio;
	}

	/**
	 * Finds the aspect ratio (width / height)
	 * 
	 * @param r
	 * @return the aspect ratio
	 */
	public static double getAspectRatio(Rect r) {
		return (double) ((double) r.width) / r.height;
	}
}
