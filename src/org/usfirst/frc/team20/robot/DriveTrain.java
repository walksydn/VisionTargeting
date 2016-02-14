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
	public void arcadeDrive(double s, double r, double l){
		for(int i=0; i<motors.length; i++){
			if(i%2==0){
				motors[i].set(-(s-r+l));
			}else{
				motors[i].set(s-l+r);
			}	
		}		
	}
	public void tankDrive(double l, double r){
		for(int i=0; i<motors.length; i++){
			if(i%2==0){
				motors[i].set(-l);
			}else{
				motors[i].set(r);
			}
		}
	}
	
	
}
