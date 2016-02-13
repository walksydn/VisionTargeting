package org.usfirst.frc.team20.robot;

import java.util.Comparator;

import org.usfirst.frc.team20.robot.Robot.ParticleReport;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ImageType;
import com.ni.vision.NIVision.Point;

public class VisionTargeting {
	public class ParticleReport implements Comparator<ParticleReport>, Comparable<ParticleReport>{
		double PercentAreaToImageArea;
		double Area;
		double BoundingRectLeft;
		double BoundingRectTop;
		double BoundingRectRight;
		double BoundingRectBottom;
		
		public int compareTo(ParticleReport r)
		{
			return (int)(r.Area - this.Area);
		}
		
		public int compare(ParticleReport r1, ParticleReport r2)
		{
			return (int)(r1.Area - r2.Area);
		}
	};
	NIVision.Range TARGET_HUE_RANGE = new NIVision.Range(0, 130); 
	NIVision.Range TARGET_SAT_RANGE = new NIVision.Range(149, 255); 
	NIVision.Range TARGET_VAL_RANGE = new NIVision.Range(142, 200);
	double AREA_MINIMUM = 0.5; 
	double LONG_RATIO = 2.22; 
	double SHORT_RATIO = 1.4;
	double SCORE_MIN = 75.0;
	double VIEW_ANGLE = 49.4; 
	NIVision.ParticleFilterCriteria2 criteria[] = new NIVision.ParticleFilterCriteria2[1];
	NIVision.ParticleFilterOptions2 filterOptions = new NIVision.ParticleFilterOptions2(0,0,1,1);
	
	Image frame;
	Image binaryFrame;
	int imaqError;
	int session;
	Image filteredImage;
	Image particleBinaryFrame;

	NIVision.StructuringElement box;

	double verticalImage = 180;
    double horizontalImage = 240;
	public VisionTargeting(){
		frame = NIVision.imaqCreateImage(ImageType.IMAGE_RGB, 0);
		binaryFrame = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 0);
		filteredImage = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 0);
		particleBinaryFrame  = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 0);
		session = NIVision.IMAQdxOpenCamera("cam0",
				NIVision.IMAQdxCameraControlMode.CameraControlModeController);
		NIVision.IMAQdxConfigureGrab(session);
	
		box = new NIVision.StructuringElement(3, 3, 1);
		
		criteria[0] = new NIVision.ParticleFilterCriteria2(NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA,
				AREA_MINIMUM, 100.0, 0, 0);

	}
		public void getImage(){
			
		}
		public int centerOfImageX(){
			Point center = new NIVision.Point((int)(horizontalImage), (int)(verticalImage));
			return center.x;
		}
		public int centerOfImageY(){
			Point center = new NIVision.Point((int)(horizontalImage), (int)(verticalImage));
			return center.y;
		}
		public void centerOfRectangle(){
			
		}
		public void drawCrosshairs(){
			
		}
		public void getDistance(){
			
		}
}
