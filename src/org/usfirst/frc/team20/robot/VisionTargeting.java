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
	private class ParticleReport implements Comparator<ParticleReport>, Comparable<ParticleReport>{
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

	Image frame = NIVision.imaqCreateImage(ImageType.IMAGE_RGB, 0);
	Image binaryFrame;
	int imaqError;
	int session;
	Image filteredImage;
	Image particleBinaryFrame;
	int numParticles;
	private double verticalImage;
    private double horizontalImage;
	
	NIVision.StructuringElement box;
	
	private double leftRec;
	private double rightRec;
	private double topRec;
	private double bottomRec;
	private double[] centerRec;
	private double Tft;
    private double horizontalPixel;
    private double Tpixel;
    private double FOVpixel;
    private double distance;
    private double boundingLeft;
    private double boundingRight;
	public VisionTargeting(){
        AREA_MINIMUM = 0.5; 
    	frame = NIVision.imaqCreateImage(ImageType.IMAGE_RGB, 0);
		binaryFrame = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 0);
		filteredImage = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 0);
		particleBinaryFrame  = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 0);
		session = NIVision.IMAQdxOpenCamera("cam0",
				NIVision.IMAQdxCameraControlMode.CameraControlModeController);

	
		box = new NIVision.StructuringElement(3, 3, 1);
		
		criteria[0] = new NIVision.ParticleFilterCriteria2(NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA,
				AREA_MINIMUM, 100.0, 0, 0);
		numParticles = NIVision.imaqCountParticles(particleBinaryFrame, 1);
		verticalImage = 180;
        horizontalImage = 240;
		NIVision.IMAQdxConfigureGrab(session);
        CameraServer.getInstance().setImage(binaryFrame);
    }
	public void getImage(){
		NIVision.IMAQdxStartAcquisition(session);
		NIVision.IMAQdxGrab(session, frame, 1);
		
		NIVision.imaqColorThreshold(binaryFrame, frame, 255, NIVision.ColorMode.HSV,
				TARGET_HUE_RANGE, TARGET_SAT_RANGE,
				TARGET_VAL_RANGE);
		float areaMin = 0.25f;
		float areaMax = 1000.0f;
		criteria[0].lower = areaMin;
		criteria[0].upper = areaMax;
		imaqError = NIVision.imaqParticleFilter4(particleBinaryFrame, binaryFrame, criteria,
				filterOptions,null);
	}
	public void drawImageCenterCrosshair(){
		Point startH = new NIVision.Point((int)(horizontalImage - 10), (int)(verticalImage));
        Point endH = new NIVision.Point((int)(horizontalImage + 10), (int)verticalImage);
        NIVision.imaqDrawLineOnImage(binaryFrame, binaryFrame, DrawMode.DRAW_VALUE, startH, endH,
        		200f);
        Point startV = new NIVision.Point((int)(horizontalImage), (int)(verticalImage - 10));
        Point endV = new NIVision.Point((int)(horizontalImage), (int)(verticalImage + 10));
        NIVision.imaqDrawLineOnImage(binaryFrame, binaryFrame, DrawMode.DRAW_VALUE, startV, endV,
        		200f);
	}
	public int findImageCenterX(){
		Point center = new NIVision.Point((int)(horizontalImage), (int)(verticalImage));
		System.out.println("Image Center X" + center.x);
		return center.x;
	}
	public int findImageCenterY(){
		Point center = new NIVision.Point((int)(horizontalImage), (int)(verticalImage));
		System.out.println("Image Center Y" + center.y);
		return center.y;
	}
		
	public void drawRectangle(){
		if(numParticles > 0){
			System.out.println("Test num of particles " + numParticles);
			for(int particleIndex = 0; particleIndex < numParticles; particleIndex++){
				ParticleReport par = new ParticleReport();
//				par.PercentAreaToImageArea = NIVision.imaqMeasureParticle(particleBinaryFrame, particleIndex,
//						0, NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA); 
				NIVision.imaqMeasureParticle(particleBinaryFrame, particleIndex,
						0, NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA);
				par.Area = NIVision.imaqMeasureParticle(particleBinaryFrame, particleIndex, 0,
						NIVision.MeasurementType.MT_AREA);
				par.BoundingRectTop = NIVision.imaqMeasureParticle(particleBinaryFrame, particleIndex,
						0, NIVision.MeasurementType.MT_BOUNDING_RECT_TOP);
				System.out.println("par.BoundingRectTop = " + par.BoundingRectTop);
				par.BoundingRectLeft = NIVision.imaqMeasureParticle(particleBinaryFrame, particleIndex, 0,
						NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT);
				par.BoundingRectBottom = NIVision.imaqMeasureParticle(particleBinaryFrame, particleIndex, 0,
						NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM);
				par.BoundingRectRight = NIVision.imaqMeasureParticle(particleBinaryFrame, particleIndex, 0,
						NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT);
				Rect r = new NIVision.Rect((int)par.BoundingRectTop ,(int) par.BoundingRectLeft, 
						Math.abs((int)(par.BoundingRectTop - par.BoundingRectBottom)),
						Math.abs((int)(par.BoundingRectLeft - par.BoundingRectRight)) );
		        NIVision.imaqDrawShapeOnImage(binaryFrame, binaryFrame, r,
		        		DrawMode.DRAW_VALUE, ShapeMode.SHAPE_RECT, 125f);
		        boundingLeft = par.BoundingRectLeft;
			    boundingRight = par.BoundingRectRight;
		        leftRec = par.BoundingRectLeft - horizontalImage;
		        rightRec = par.BoundingRectLeft - horizontalImage;
		        topRec = -(par.BoundingRectTop - verticalImage);
			    bottomRec = -(par.BoundingRectBottom - verticalImage);
			    centerRec = new double[2];
			    centerRec[0] = (leftRec + rightRec) / 2;
			    centerRec[1] = (topRec + bottomRec) / 2;
			}
		}
	}
	public int findRecCenterX(){
		Point recCenter = new NIVision.Point((int)(centerRec[0]), (int)(centerRec[1]));
		System.out.println("Rec Center X: " + recCenter.x);
		return recCenter.x;
	}
	public int findRecCenterY(){
		Point recCenter = new NIVision.Point((int)(centerRec[0]), (int)(centerRec[1]));
		System.out.println("Rec Center Y: " + recCenter.y);
		return recCenter.y;
	}
	public void drawRecCenterCrosshair(){
		Point startHrec = new NIVision.Point((int)(centerRec[0] + horizontalImage - 10),
        		(int)(-centerRec[1] + verticalImage + 10));
        Point endHrec = new NIVision.Point((int)(centerRec[0] + horizontalImage + 10),
        		(int)(-centerRec[1] + verticalImage - 10));
        NIVision.imaqDrawLineOnImage(binaryFrame, binaryFrame, DrawMode.DRAW_VALUE,
        		startHrec, endHrec,
        		150f);
        Point startVrec = new NIVision.Point((int)(centerRec[0] + horizontalImage - 10),
        		(int)(-centerRec[1] + verticalImage - 10));
        Point endVrec = new NIVision.Point((int)(centerRec[0] + horizontalImage + 10),
        		(int)(-centerRec[1] + verticalImage + 10));
        NIVision.imaqDrawLineOnImage(binaryFrame, binaryFrame, DrawMode.DRAW_VALUE, startVrec,
        		endVrec, 150f);
	}
	public void getDistance(){
		Tft = 1.625;
        horizontalPixel = horizontalImage*2;
        Tpixel = Math.abs(boundingLeft - boundingRight);
        FOVpixel = (horizontalPixel*Tft)/(2*Tpixel);
        distance = FOVpixel/Math.tan((44.5*(Math.PI/180))/2);
        System.out.println("Distance: " + distance);
	}
}
