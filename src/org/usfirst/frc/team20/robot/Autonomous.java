package org.usfirst.frc.team20.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;

public class Autonomous {
	AHRS navx;
	boolean gotYaw;
	private double origYaw;
	DriveTrain drive;

	public Autonomous(){
		navx = new AHRS(SerialPort.Port.kMXP);
		drive = new DriveTrain(4);
		gotYaw = false;
		drive.assignPort(0, 10);
		drive.assignPort(1, 9);
		drive.assignPort(2, 1);
		drive.assignPort(3, 2);
	}
	public void turnToYaw(double yaw, int tolerance, double speed){
		if (!gotYaw){
			origYaw = navx.getYaw();
			gotYaw = true;
		}
		if (navx.getYaw() > origYaw + yaw + tolerance && navx.getYaw() < origYaw + 180) {
			drive.drive(0, -speed); 			//TODO negate proper side (right)
			drive.drive(1, -speed);
			drive.drive(2, -speed);
			drive.drive(3, -speed);
		}else if (navx.getYaw() < origYaw + yaw + tolerance && navx.getYaw() > origYaw - 180) {
			drive.drive(0, speed); 		//TODO negate proper side (left)
			drive.drive(1, speed);
			drive.drive(2, speed);
			drive.drive(3, speed);
		}else{
			drive.drive(0, 0);
			drive.drive(1, 0);
			drive.drive(2, 0);
			drive.drive(3, 0);
		}
	}
	public void stop(){
		drive.drive(0, 0);
		drive.drive(1, 0);
		drive.drive(2, 0);
		drive.drive(3, 0);
	}
}
