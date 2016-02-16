package org.usfirst.frc.team20.robot;

import edu.wpi.first.wpilibj.CANTalon;

public class DriveTrain {
	CANTalon[]motors;
	
	public DriveTrain(int m){
		motors = new CANTalon[m];
	}
	public void assignPort(int i, int p){		//i = motor (numbered from left to right,
		motors[i] = new CANTalon(p);			//top to bottom), p = port number
	}
	public void drive(int m, double s){			//m = motor number (i)
		motors[m].set(s);						//s = speed of motor
	}
	public void arcadeDrive(double speed, double right, double left){
		for(int i=0; i<motors.length; i++){
			if(i%2==0){
				motors[i].set(-(speed-right+left));
			}else{
				motors[i].set(speed-left+right);
			}	
		}		
	}
	public void tankDrive(double left, double right){
		for(int i=0; i<motors.length; i++){
			if(i%2==0){
				motors[i].set(-left);
			}else{
				motors[i].set(right);
			}
		}
	}
	
	
}
