package org.usfirst.frc.team20.robot;

import java.util.Comparator;
import com.ni.vision.NIVision;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ImageType;
import com.ni.vision.NIVision.Point;
import com.ni.vision.NIVision.Rect;
import com.ni.vision.NIVision.ShapeMode;
import edu.wpi.first.wpilibj.CameraServer;

public class VisionTargeting {
	public class ParticleReport implements Comparator<ParticleReport>, Comparable<ParticleReport> {
		double PercentAreaToImageArea;
		double Area;
		double BoundingRectLeft;
		double BoundingRectTop;
		double BoundingRectRight;
		double BoundingRectBottom;
		public int compareTo(ParticleReport r) {
			return (int) (r.Area - this.Area);
		}
		public int compare(ParticleReport r1, ParticleReport r2) {
			return (int) (r1.Area - r2.Area);
		}
	};
	double AREA_MINIMUM;
	double LONG_RATIO;
	double SHORT_RATIO;
	double SCORE_MIN;
	double VIEW_ANGLE;
	Image frame;
	Image binaryFrame;
	int imaqError;
	int session;
	Image filteredImage;
	Image particleBinaryFrame;
	NIVision.StructuringElement box;
	NIVision.ParticleFilterCriteria2 criteria[];
	NIVision.ParticleFilterOptions2 filterOptions;
	int numParticles;
	double verticalImage;
	double horizontalImage;
	double distance;
	NIVision.Range TARGET_HUE_RANGE;
	NIVision.Range TARGET_SAT_RANGE;
	NIVision.Range TARGET_VAL_RANGE;
	double leftRec;
	double rightRec;
	double topRec;
	double bottomRec;
	double centerRec[] = new double[2];{
	centerRec[0] = (leftRec + rightRec) / 2;
	centerRec[1] = (topRec + bottomRec) / 2;}
	double boundingRight;
	double boundingLeft;
	
	
	public VisionTargeting(){
		System.out.println("Starting Init");
		AREA_MINIMUM = 0.25;
		LONG_RATIO = 2.22;
		SHORT_RATIO = 1.4;
		SCORE_MIN = 75.0;
		VIEW_ANGLE = 49.4;
		frame = NIVision.imaqCreateImage(ImageType.IMAGE_RGB, 0);
		binaryFrame = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 0);
		particleBinaryFrame = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 0);
		session = NIVision.IMAQdxOpenCamera("cam0",
				NIVision.IMAQdxCameraControlMode.CameraControlModeController);
		box = new NIVision.StructuringElement(3, 3, 1);
		criteria = new NIVision.ParticleFilterCriteria2[1];
		criteria[0] = new NIVision.ParticleFilterCriteria2(NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA,
				AREA_MINIMUM, 100.0, 0, 0);
		filterOptions = new NIVision.ParticleFilterOptions2(0, 0, 1, 1);
		verticalImage = 180;
		horizontalImage = 240;
		distance = -1;
		NIVision.IMAQdxConfigureGrab(session);
		NIVision.IMAQdxStartAcquisition(session);
		TARGET_HUE_RANGE = new NIVision.Range(0, 130);
		TARGET_SAT_RANGE = new NIVision.Range(149, 255);
		TARGET_VAL_RANGE = new NIVision.Range(142, 200);
		filteredImage = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 0);
		System.out.println("Done");
	}
	private void getImage(){
		NIVision.IMAQdxGrab(session, frame, 1);
		NIVision.imaqColorThreshold(binaryFrame, frame, 255, NIVision.ColorMode.HSV,
				TARGET_HUE_RANGE, TARGET_SAT_RANGE, TARGET_VAL_RANGE);
		imaqError = NIVision.imaqParticleFilter4(particleBinaryFrame, binaryFrame,
				criteria, filterOptions, null);
		numParticles = NIVision.imaqCountParticles(particleBinaryFrame, 1);
	}
	private void drawRectangle(){
		for (int particleIndex = 0; particleIndex < numParticles; particleIndex++) {
			System.out.println("We've got particles!");
			ParticleReport par = new ParticleReport();
			par.Area = NIVision.imaqMeasureParticle(particleBinaryFrame, particleIndex, 0,
					NIVision.MeasurementType.MT_AREA);
			par.BoundingRectTop = NIVision.imaqMeasureParticle(particleBinaryFrame, particleIndex, 0,
					NIVision.MeasurementType.MT_BOUNDING_RECT_TOP);
			System.out.println("Top: " + par.BoundingRectTop);
			par.BoundingRectLeft = NIVision.imaqMeasureParticle(particleBinaryFrame, particleIndex, 0,
					NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT);
			System.out.println("Left: " + par.BoundingRectLeft);
			par.BoundingRectBottom = NIVision.imaqMeasureParticle(particleBinaryFrame, particleIndex, 0,
					NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM);
			par.BoundingRectRight = NIVision.imaqMeasureParticle(particleBinaryFrame, particleIndex, 0,
					NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT);
			System.out.println("Par Works: " + par.BoundingRectRight);
			Rect r = new NIVision.Rect((int) par.BoundingRectTop, (int) par.BoundingRectLeft,
					Math.abs((int) (par.BoundingRectTop - par.BoundingRectBottom)),
					Math.abs((int) (par.BoundingRectLeft - par.BoundingRectRight)));
			NIVision.imaqDrawShapeOnImage(binaryFrame, binaryFrame, r, DrawMode.DRAW_VALUE,
					ShapeMode.SHAPE_RECT, 150f);
			leftRec = par.BoundingRectLeft - horizontalImage;
			rightRec = par.BoundingRectRight - horizontalImage;
			topRec = -(par.BoundingRectTop - verticalImage);
			bottomRec = -(par.BoundingRectBottom - verticalImage);
			centerRec = new double[2];
			centerRec[0] = (leftRec + rightRec) / 2;
			centerRec[1] = (topRec + bottomRec) / 2;
			boundingRight = par.BoundingRectRight;
			boundingLeft = par.BoundingRectLeft;
		}
	}
	private void drawCenterCrosshairs(){
		Point startH = new NIVision.Point((int) (horizontalImage - 10), (int) (verticalImage));
		Point endH = new NIVision.Point((int) (horizontalImage + 10), (int) verticalImage);
		NIVision.imaqDrawLineOnImage(binaryFrame, binaryFrame, DrawMode.DRAW_VALUE, startH, endH, 200f);
		Point startV = new NIVision.Point((int) (horizontalImage), (int) (verticalImage - 10));
		Point endV = new NIVision.Point((int) (horizontalImage), (int) (verticalImage + 10));
		NIVision.imaqDrawLineOnImage(binaryFrame, binaryFrame, DrawMode.DRAW_VALUE, startV, endV, 200f);
	}
	private void drawCenterRecCrosshairs(){
		Point startHrec = new NIVision.Point((int) (centerRec[0] + horizontalImage - 10),
				(int) (-centerRec[1] + verticalImage + 10));
		Point endHrec = new NIVision.Point((int) (centerRec[0] + horizontalImage + 10),
				(int) (-centerRec[1] + verticalImage - 10));
		NIVision.imaqDrawLineOnImage(binaryFrame, binaryFrame, DrawMode.DRAW_VALUE, startHrec, endHrec, 150f);
		Point startVrec = new NIVision.Point((int) (centerRec[0] + horizontalImage - 10),
				(int) (-centerRec[1] + verticalImage - 10));
		Point endVrec = new NIVision.Point((int) (centerRec[0] + horizontalImage + 10),
				(int) (-centerRec[1] + verticalImage + 10));
		NIVision.imaqDrawLineOnImage(binaryFrame, binaryFrame, DrawMode.DRAW_VALUE, startVrec, endVrec, 150f);
	}
	public void processImage(){
		getImage();
		drawRectangle();
		drawCenterCrosshairs();
		drawCenterRecCrosshairs();
		CameraServer.getInstance().setImage(binaryFrame);
	}
	public double getDistance(){
		double Tft = 1.625;
		double horizontalPixel = horizontalImage * 2;
		double Tpixel = Math.abs(boundingLeft - boundingRight);
		double FOVpixel = (horizontalPixel * Tft) / (2 * Tpixel);
		double distance = FOVpixel / Math.tan((44.5 * (Math.PI / 180)) / 2);
		System.out.println("Distance: " + distance);
		return distance;
	}
	public double centerXCoordinate(){
		Point center = new NIVision.Point((int)(horizontalImage), (int)(verticalImage));
		return center.x;
	}
	public double centerYCoordinate(){
		Point center = new NIVision.Point((int)(horizontalImage), (int)(verticalImage));
		return center.y;
	}
	public double recCenterXCoordinate(){
		Point recCenter = new NIVision.Point((int) (centerRec[0]), (int) (centerRec[1]));
		return recCenter.x;
	}
	public double recCenterYCoordinate(){
		Point recCenter = new NIVision.Point((int) (centerRec[0]), (int) (centerRec[1]));
		return recCenter.y;
	}
}	
	
