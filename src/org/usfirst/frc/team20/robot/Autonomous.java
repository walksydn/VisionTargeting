package org.usfirst.frc.team20.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;

public class Autonomous {
	AHRS navx;
	private boolean gotYaw;
	private double origYaw;
	DriveTrain drive;

	public Autonomous(){
		navx = new AHRS(SerialPort.Port.kMXP);
		drive = new DriveTrain(4);
		gotYaw = false;
	}
	public void turnToAngle(int angle, int tolerance){
		if (!gotYaw){
			origYaw = navx.getYaw();
			gotYaw = true;
		}
		if (navx.getYaw() > origYaw + angle + tolerance && navx.getYaw() < origYaw + 180) {
			drive.drive(0, -1); 			//TODO negate proper side (right)
			drive.drive(1, 1);
			drive.drive(2, -1);
			drive.drive(3, 1);
		}
		if (navx.getYaw() < origYaw + angle + tolerance && navx.getYaw() > origYaw - 180) {
			drive.drive(0, 1); 		//TODO negate proper side (left)
			drive.drive(1, -1);
			drive.drive(2, 1);
			drive.drive(3, -1);
		}
	}
	public void stop(){
		drive.drive(0, 0);
		drive.drive(1, 0);
		drive.drive(2, 0);
		drive.drive(3, 0);
	}
}
