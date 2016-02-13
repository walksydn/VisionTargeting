
package org.usfirst.frc.team20.robot;

import java.util.Comparator;
import com.ni.vision.NIVision;
import com.ni.vision.NIVision.*;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ImageType;
import com.ni.vision.NIVision.ShapeMode;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	//A structure to hold measurements of a particle
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

	
	//78 100
	
	NIVision.Range TARGET_HUE_RANGE = new NIVision.Range(0, 130); 
	NIVision.Range TARGET_SAT_RANGE = new NIVision.Range(149, 255); 
//	NIVision.Range TARGET_VAL_RANGE = new NIVision.Range(142, 255); // Default
																	// value
	NIVision.Range TARGET_VAL_RANGE = new NIVision.Range(142, 200); // Default
	// value
	
	
	
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
	
	
	

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initiaRGETlization code.
	 */
	public void robotInit() {
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

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	public void autonomousInit() {

	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {

	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
			
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
		
		
	    int numParticles = NIVision.imaqCountParticles(particleBinaryFrame, 1);
//		double middleHorizonal=0.0000;
//		double middleHVertical=0.0000;
		
//		System.out.println("Filtered particles" + numParticles);
		
		double verticalImage = 180;
        double horizontalImage = 240;
		Point startH = new NIVision.Point((int)(horizontalImage - 10), (int)(verticalImage));
        Point endH = new NIVision.Point((int)(horizontalImage + 10), (int)verticalImage);
        NIVision.imaqDrawLineOnImage(binaryFrame, binaryFrame, DrawMode.DRAW_VALUE, startH, endH,
        		200f);
        Point startV = new NIVision.Point((int)(horizontalImage), (int)(verticalImage - 10));
        Point endV = new NIVision.Point((int)(horizontalImage), (int)(verticalImage + 10));
        NIVision.imaqDrawLineOnImage(binaryFrame, binaryFrame, DrawMode.DRAW_VALUE, startV, endV,
        		200f);
		if(numParticles > 0){
			System.out.println("Test num of particles " + numParticles);
			
			//Measure particles and sort by particle size
//			Vector<ParticleReport> particles = new Vector<ParticleReport>();
			
			for(int particleIndex = 0; particleIndex < numParticles; particleIndex++)
			{
				
				ParticleReport par = new ParticleReport();
//				par.PercentAreaToImageArea = NIVision.imaqMeasureParticle(particleBinaryFrame, particleIndex,
//						0, NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA);
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
//				middleHorizonal = (par.BoundingRectLeft - par.BoundingRectRight) / 2.0000;
//				middleHVertical = (par.BoundingRectTop - par.BoundingRectBottom) / 2.0000;
//				System.out.println("Middle horizontal =" + middleHorizonal);
//				System.out.println("Middle vertical =" + middleHVertical);
				Rect r = new NIVision.Rect((int)par.BoundingRectTop ,(int) par.BoundingRectLeft, 
						Math.abs((int)(par.BoundingRectTop - par.BoundingRectBottom)),
						Math.abs((int)(par.BoundingRectLeft - par.BoundingRectRight)) );
		        NIVision.imaqDrawShapeOnImage(binaryFrame, binaryFrame, r,
		        		DrawMode.DRAW_VALUE, ShapeMode.SHAPE_RECT, 125f);
		        
		        double leftRec = par.BoundingRectLeft - horizontalImage;
		        double rightRec = par.BoundingRectRight - horizontalImage;
		        double topRec = -(par.BoundingRectTop - verticalImage);
		        double bottomRec = -(par.BoundingRectBottom - verticalImage);
		        double centerRec[] = new double[2];
		        centerRec[0] = (leftRec + rightRec) / 2;
		        centerRec[1] = (topRec + bottomRec) / 2;
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
		        Point center = new NIVision.Point((int)(horizontalImage), (int)(verticalImage));
		        Point recCenter = new NIVision.Point((int)(centerRec[0]), (int)(centerRec[1]));
		        System.out.println("Center: " + center.x + " , " + center.y);
		        System.out.println("Rectangle Center: " + recCenter.x + " , " + recCenter.y);
		        //FOV calculations
		        double Tft = 1.625;
		        double horizontalPixel = horizontalImage*2;
		        double Tpixel = Math.abs(par.BoundingRectLeft - par.BoundingRectRight);
		        double FOVpixel = (horizontalPixel*Tft)/(2*Tpixel);
		        double distance = FOVpixel/Math.tan((44.5*(Math.PI/180))/2);
		        System.out.println("Distance: " + distance);

		       
		   //     NIVision.imaqDrawShapeOnImage(binaryFrame, binaryFrame, r, DrawMode.DRAW_VALUE, ShapeMode.SHAPE_OVAL, 125f);
		        
		       // Math.abs((int)(par.BoundingRectRight - par.BoundingRectLeft))
		        
			}
		}
		
		//NIVision.imaqContourFitCircle(image, 20, rejectOutliers)
		// 
		
	
		//NIVision.imaqConvexHull(filteredImage, binaryFrame, 8);
		
		CameraServer.getInstance().setImage(binaryFrame);
		
		//CameraServer.getInstance().setImage(frame);
		Timer.delay(0.005);

	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		VisionTargeting vision = new VisionTargeting();
		vision.getImage();
		vision.drawImageCenterCrosshair();
		vision.drawRectangle();
		vision.drawRecCenterCrosshair();
		vision.findRecCenterX();
		vision.findRecCenterY();
		vision.findImageCenterX();
		vision.findImageCenterY();
	}

}
