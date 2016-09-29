/* 4580 */
package org.usfirst.frc.team4580.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*  Get raw axis values:
 * 			 (Logitech): 0=Left x-axis
 * 						 1=Left y-axis
 * 						 2=Right x-axis
 * 						 3=Right y-axis
 * 				(X-box): 0=Left x-axis
 * 						 1=Left y-axis
 * 						 2=Left Trigger
 * 						 3=Right Trigger
 * 						 4=Right x-axis
 * 						 5=Right y-axis
 *Get raw button values: 
 * 			 (Logitech): 1=X 
 * 						 2=A 
 * 						 3=B 
 * 						 4=Y
 * 			             5=leftBumper 
 * 						 6=rightBumper
 * 			             7=leftTrigger 
 * 						 8=rightTrigger
 *                       9=back 
 *                       10=start 
 *                       11=leftStick 
 *                       12=rightStick
 *              (X-box): 1=A
 *              		 2=B
 *              		 3=X
 *              		 4=Y
 *                       5=leftBumper
 *                       6=rightBumper
 *                       7=back
 *                       8=start
 *                       9=leftStick
 *                       10=rightStick
 *        Can addresses:        
 * 						 frontLeft = 2
 *                       rearLeft = 3
 *                       frontRight = 4
 *                       rearRight = 5
 *                       rotateLeft = 6
 *                       rotateRight= 7
 *                       leftIn = 8
 *                       rightIn = 9                       
 */

public class Robot extends IterativeRobot {
    SendableChooser chooser;
    SendableChooser arm;
    final String lowAuto = "LowBar";
    final String moatAuto = "Moat";
    final String rampAuto = "Ramparts";
    final String rockAuto = "RockWall";
    final String roughAuto = "RoughTerrain";
    final String portAuto = "Guillotine";
    final String CdF = "Shovel the Fries";
    String autoSelected;
    
    AHRS ahrs;
    RobotDrive myRobot;  
    Joystick driver;
    Joystick assist;
    Encoder leftEn;
    Encoder rightEn;
    CANTalon frontLeft;
    CANTalon rearLeft;
    CANTalon frontRight;
    CANTalon rearRight;
    CANTalon rotateLeft;
    CANTalon rotateRight;
    CANTalon leftIn;
    CANTalon rightIn;
    CANTalon eject;
    DigitalInput limitSwitch;
    CameraServer camera;
    Timer timer;
    final double pD = 2;
    final double iD = 2;
    
    final double zero = 0.0;
    final double tenth= 0.1;
    final double twoT = 0.2;
    final double quarter = 0.25;
    final double threeT = 0.3;
    final double fourT = 0.4;
    final double half = 0.5;
    final double sixT = 0.6;
    final double sevenT = 0.7;
    final double threeQ=.75;
    final double eightT = 0.8;
    final double nineT = 0.9;
    final double one = 1.0;
    final double two = 2.0;
    final double three = 3.0;
    final double four = 4.0;
    final double five = 5.0;
    final double six = 6.0;
    final double seven = 7.0;
    final double eight = 8.0;
    final double nine = 9.0;
    final double ten = 10.0;
    final double eleven = 11.0;
    final double twelve = 12.0;
    final double thriteen = 13.0;
    final double fourteen = 14.0;
    final double fifteen = 15.0;
    final double reduce = 0.75;
    final double rotateSpeed = 1.0;
    final double shooterSpeed = 1.0;
    final double intakeSpeed = -0.5;
    final int zeroI = 0;
    final int oneI = 1;
    final int twoI = 2;
    final int threeI = 3;
    final int fourI = 4;
    final int target = 2500;
    final int fullScale = 4500;
    
    double error;
    double intError;
    double P;
    double I;
    double robotAngle;
	double leftEnRate;
	double rightEnRate;
	double leftEnRaw;
	double rightEnRaw;
	double leftStick;
	double rightStick;
	double leftOutput;
	double rightOutput;
    double armInput;
    double XtestXLS;
	double XtestYLS;
	double XtestLT;
	double XtestRT;
	double XtestXRS;
	double XtestYRS;
	boolean slowLB;
	boolean rotateFwdRT;
	boolean rotateBkwdLT;
	boolean testB;
	boolean shootA;
	boolean spinUpLS;
	boolean spinInRB;
	boolean XtestA;
	boolean XtestB;
	boolean XtestX;
	boolean XtestY;
	boolean XtestLB;
	boolean XtestRB;
	boolean XtestBK;
	boolean XtestST;
	boolean XtestLS;
	boolean XtestRS;
	boolean intakeLimit;
	boolean ONS = false;
	boolean ONS1 = false;
	boolean Toggle = false;
	boolean Toggle1 = false;
	boolean CDF;
	boolean CDF1;
	boolean CDF2;
	boolean CDF3;
	boolean off;
	
    public void robotInit(){
    	ahrs = new AHRS(SPI.Port.kMXP);        
        driver = new Joystick(0);
        assist = new Joystick(1);
        leftEn = new Encoder(0,1,false,EncodingType.k4X);
        rightEn = new Encoder(2,3,false,EncodingType.k4X);
        frontLeft = new CANTalon(2);
        rearLeft = new CANTalon(3);
        frontRight = new CANTalon(4);
        rearRight = new CANTalon(5);
        rotateLeft = new CANTalon(6);
        rotateRight= new CANTalon(7);
        leftIn = new CANTalon(8);
        rightIn = new CANTalon(9);
        eject = new CANTalon(10);
        limitSwitch = new DigitalInput(4);
        myRobot = new RobotDrive(frontLeft,rearLeft,frontRight,rearRight);
    	chooser = new SendableChooser();
        arm = new SendableChooser();
        timer = new Timer();   
        camera = CameraServer.getInstance();
        camera.startAutomaticCapture("cam0");
    	camera.setQuality(50);

        DashboardChoices();        
    	Interface();
        NavXData();
    	Dashboard();
    }
    public void disabledInit(){//Check for Motor Code :( 

    } 
    public void disabledPeriodic(){//Check for Motor Code :( 
    	Interface();
    	NavXData();
    	Dashboard();
    }
    public void autonomousInit(){
    	autoSelected = (String) chooser.getSelected();
        //autoSelected = SmartDashboard.getString("Auto Selector", defaultAuto);
    	leftEn.reset();
        rightEn.reset();
        error = 0;
        intError = 0;
        timer.start();
        CDF = false;
    	CDF1 = false;
    	CDF2 = false;
    	CDF3 = false;
        off = false;
    }
    public void autonomousPeriodic(){//Check Everything
    	switch(autoSelected) {
    	case lowAuto:
    	default:
    		if(CDF){
    			if(CDF1){
        			myRobot.tankDrive(zero, zero);
        		}
    			else if(timer.hasPeriodPassed(three)){
        			CDF1 = true;
        		}
        	    else if(Math.abs(leftEnRate) > Math.abs(rightEnRate)&& Math.abs(rightEnRate)>zero){
    				myRobot.tankDrive(sixT*Math.abs((rightEnRate/leftEnRate)), sixT);
    			}
    			else if(Math.abs(leftEnRate) < Math.abs(rightEnRate)&& Math.abs(leftEnRate)>zero){
    				myRobot.tankDrive(sixT, sixT*Math.abs((leftEnRate/rightEnRate)));
    			}
    			else{
    				myRobot.tankDrive(sixT, sixT);
    			}
    		}
    		else if(timer.hasPeriodPassed(two)){
    			CDF = true;
    		}
    		else{
    			rotateLeft.set(-threeQ);
    			rotateRight.set(-threeQ);
    		}
    		break;
    	case moatAuto:
    		if(CDF){
        		myRobot.tankDrive(zero, zero);
        	}
    		else if(timer.hasPeriodPassed(two)){
    			CDF = true;
        	}
        	else if(Math.abs(leftEnRate) > Math.abs(rightEnRate)&& Math.abs(rightEnRate)>zero){
        		myRobot.tankDrive(-eightT*Math.abs((rightEnRate/leftEnRate)), -eightT);
    		}
    		else if(Math.abs(leftEnRate) < Math.abs(rightEnRate)&& Math.abs(leftEnRate)>zero){
    			myRobot.tankDrive(-eightT, -eightT*Math.abs((leftEnRate/rightEnRate)));
    		}
    		else{
    			myRobot.tankDrive(-eightT, -eightT);
    		}
    		break;
    	case rampAuto:
    		if(CDF){
        		myRobot.tankDrive(zero, zero);
        	}
    		else if(timer.hasPeriodPassed(three+half)){
        		CDF = true;
        	}
        	else if(Math.abs(leftEnRate) > Math.abs(rightEnRate)&& Math.abs(rightEnRate)>zero){
        		myRobot.tankDrive(-sixT*Math.abs((rightEnRate/leftEnRate)), -sixT);
        	}
        	else if(Math.abs(leftEnRate) < Math.abs(rightEnRate)&& Math.abs(leftEnRate)>zero){
        		myRobot.tankDrive(-sixT, -sixT*Math.abs((leftEnRate/rightEnRate)));
        	}
        	else{
        		myRobot.tankDrive(-sixT, -sixT);
        	}
    		break;
    	case rockAuto:
    		if(CDF){
    			myRobot.tankDrive(zero, zero);
        	}
    		else if(timer.hasPeriodPassed(one+half)){
        		CDF = true;
        		myRobot.tankDrive(one, one);
        	}
        	else if(Math.abs(leftEnRate) > Math.abs(rightEnRate)&& Math.abs(rightEnRate)>zero){
        		myRobot.tankDrive(-one*Math.abs((rightEnRate/leftEnRate)), -one);
        	}
        	else if(Math.abs(leftEnRate) < Math.abs(rightEnRate)&& Math.abs(leftEnRate)>zero){
        		myRobot.tankDrive(-one, -one*Math.abs((leftEnRate/rightEnRate)));
        	}
        	else{
        		myRobot.tankDrive(-one, -one);
        	}
    		break;
    	case roughAuto:   		
    		if(CDF){
    			if(CDF1){
    				if(CDF2){
    					if(CDF3){
    						myRobot.tankDrive(zero, zero);
    					}
    					else if(timer.hasPeriodPassed(four)){
    						CDF3 = true;
    					}
    					else if(Math.abs(leftEnRate) > Math.abs(rightEnRate)&& Math.abs(rightEnRate)>zero){
    	    				myRobot.tankDrive(-sixT*Math.abs((rightEnRate/leftEnRate)), -sixT);
    	    			}
    	    			else if(Math.abs(leftEnRate) < Math.abs(rightEnRate)&& Math.abs(leftEnRate)>zero){
    	    				myRobot.tankDrive(-sixT, -sixT*Math.abs((leftEnRate/rightEnRate)));
    	    			}
    	    			else{
    	    				myRobot.tankDrive(-sixT, -sixT);
    	    			}
    				}
    				else if(timer.hasPeriodPassed(1)){
    					CDF2 = true;
    				}
    				else{
    					leftIn.set(-one);
    				}
        		}
    			else if(timer.hasPeriodPassed(four)){
        			CDF1 = true;
        		}
        	    else if(Math.abs(leftEnRate) > Math.abs(rightEnRate)&& Math.abs(rightEnRate)>zero){
    				myRobot.tankDrive(sixT*Math.abs((rightEnRate/leftEnRate)), sixT);
    			}
    			else if(Math.abs(leftEnRate) < Math.abs(rightEnRate)&& Math.abs(leftEnRate)>zero){
    				myRobot.tankDrive(sixT, sixT*Math.abs((leftEnRate/rightEnRate)));
    			}
    			else{
    				myRobot.tankDrive(sixT, sixT);
    			}
    		}
    		else if(timer.hasPeriodPassed(two)){
    			CDF = true;
    		}
    		else{
    			rotateLeft.set(-one);
    			rotateRight.set(-one);
    		}
    		break;
    	case portAuto:
    		if(CDF){
    			if(CDF1){
        			myRobot.tankDrive(zero, zero);
        		}
    			else if(timer.hasPeriodPassed(four)){
        			myRobot.tankDrive(-one, -one);
    				CDF1 = true;
        		}
        	    else if(Math.abs(leftEnRate) > Math.abs(rightEnRate)&& Math.abs(rightEnRate)>zero){
    				myRobot.tankDrive(sixT*Math.abs((rightEnRate/leftEnRate)), sixT);
    			}
    			else if(Math.abs(leftEnRate) < Math.abs(rightEnRate)&& Math.abs(leftEnRate)>zero){
    				myRobot.tankDrive(sixT, sixT*Math.abs((leftEnRate/rightEnRate)));
    			}
    			else{
    				myRobot.tankDrive(sixT, sixT);
    			}
    		}
    		else if(timer.hasPeriodPassed(two)){
    			CDF = true;
    		}
    		else{
    			rotateLeft.set(-one);
    			rotateRight.set(-one);
    		}
    		break;
    	case CdF:
    		if(CDF){
    			if(CDF1){
    				if(CDF2){
						if(CDF3){
							myRobot.tankDrive(zero, zero);
						}
						else if(timer.hasPeriodPassed(half)){
							CDF3 = true;
						}
						else{
							if(Math.abs(leftEnRate) > Math.abs(rightEnRate)&& Math.abs(rightEnRate)>zero){
								myRobot.tankDrive(one*Math.abs((rightEnRate/leftEnRate)), one);
							}
							else if(Math.abs(leftEnRate) < Math.abs(rightEnRate)&& Math.abs(leftEnRate)>zero){
								myRobot.tankDrive(one, one*Math.abs((leftEnRate/rightEnRate)));
							}
							else{
								myRobot.tankDrive(one, one);
							}
						}
					}
					else if(timer.hasPeriodPassed(one)){
						CDF2 = true;
					}
					else{
						rotateLeft.set(-one);
						rotateRight.set(-one);
					}
				}
				else if (timer.hasPeriodPassed(eightT)){
					CDF1 = true;
				}
				else{
					myRobot.tankDrive(fourT, fourT);
				}
			}
			else if(timer.hasPeriodPassed(half)){
				myRobot.tankDrive(-one, -one);
				CDF = true;
			}
			else{
				if(Math.abs(leftEnRate) > Math.abs(rightEnRate)&& Math.abs(rightEnRate)>zero){
					myRobot.tankDrive(sixT*Math.abs((rightEnRate/leftEnRate)), sixT);
				}
				else if(Math.abs(leftEnRate) < Math.abs(rightEnRate)&& Math.abs(leftEnRate)>zero){
					myRobot.tankDrive(sixT, sixT*Math.abs((leftEnRate/rightEnRate)));
				}
				else{
					myRobot.tankDrive(sixT, sixT);
				}
			}
    		/*distance = 73.25in
    		 *one rotation = 25.133in
    		 *one rotation = XXXX encoder units
    		 *distance = 2.914 rotations
    		 *distance = YYYY encoder units 
    		 */
    		/*P = (target-leftEnRaw)/fullScale;
    		if(leftEnRaw >= target){
				CDF = true;
			}
			else if(CDF){
				if(timer.hasPeriodPassed(six)){
					CDF1 = true;
				}
				else if (CDF1){
					if(timer.hasPeriodPassed(eight)){
						CDF2 = true;
					}
					else if(CDF2){
						myRobot.tankDrive(zero, zero);
					}
					else{
						if(Math.abs(leftEnRate) > Math.abs(rightEnRate)&& Math.abs(rightEnRate)>zero){
							myRobot.tankDrive(-one*Math.abs((rightEnRate/leftEnRate)), -one);
						}
						else if(Math.abs(leftEnRate) < Math.abs(rightEnRate)&& Math.abs(leftEnRate)>zero){
							myRobot.tankDrive(-one, -one*Math.abs((leftEnRate/rightEnRate)));
						}
						else{
							myRobot.tankDrive(-one, -one);
						}
					}
				}
				else{
					rotateLeft.set(-one);
					rotateRight.set(-one);
				}
			}
			else{
				if(Math.abs(leftEnRate) > Math.abs(rightEnRate)&& Math.abs(rightEnRate)>zero){
					myRobot.tankDrive(P*Math.abs((rightEnRate/leftEnRate)), P);
				}
				else if(Math.abs(leftEnRate) < Math.abs(rightEnRate)&& Math.abs(leftEnRate)>zero){
					myRobot.tankDrive(P, P*Math.abs((leftEnRate/rightEnRate)));
				}
				else{
					myRobot.tankDrive(P, P);
				}
			}*/
    		break;
    	}
    }
    public void teleopInit(){

    }
    public void teleopPeriodic(){
        Interface();
    	NavXData();
    	Dashboard();
    	Drive();
    	Rotate();
    	Intake();
    	Eject();
    }
    public void testInit(){

    }
    public void testPeriodic(){
    	Interface();
    	NavXData();
    	Dashboard();
    }
    public void DashboardChoices(){//Always put first in RobotInit()
    	chooser.addDefault("LowBar", lowAuto);
        chooser.addObject("Moat", moatAuto);
        chooser.addObject("Ramparts", rampAuto);
        chooser.addObject("RockWall", rockAuto);
        chooser.addObject("RoughTerrain", roughAuto);
        chooser.addObject("Guillotine", portAuto);
        chooser.addObject("Shovel the Fries", CdF);
        SmartDashboard.putData("Auto choices", chooser);
        

        arm.addDefault("Half", half);
        arm.addObject("Zero", zero);
        arm.addObject("Tenth", tenth);
        arm.addObject("Quarter", quarter);
        arm.addObject("Three-Quarter", threeQ);
        arm.addObject("Full", one);
        SmartDashboard.putData("Arm Choices", arm);
    }
    public void Interface(){//Put before Dashboard()    	     
    	armInput = (double)arm.getSelected();
    	
    	robotAngle = ahrs.getAngle();
       
    	leftStick = driver.getRawAxis(3);
    	rightStick = driver.getRawAxis(1);
    	slowLB = driver.getRawButton(11);  // spinUpLS = driver.getRawButton(5);
    	rotateFwdRT = driver.getRawButton(8);
    	rotateBkwdLT = driver.getRawButton(7);
    	shootA = driver.getRawButton(2);
    	spinUpLS = driver.getRawButton(5); // spinUpLS = driver.getRawButton(11);
    	spinInRB = driver.getRawButton(6);
    	testB = driver.getRawButton(3);
    	
    	XtestXLS = assist.getRawAxis(0);
    	XtestYLS = assist.getRawAxis(1);
    	XtestLT = assist.getRawAxis(2);
    	XtestRT = assist.getRawAxis(3);
    	XtestXRS = assist.getRawAxis(4);
    	XtestYRS = assist.getRawAxis(5);
    	XtestA = assist.getRawButton(1);
    	XtestB = assist.getRawButton(2);
    	XtestX = assist.getRawButton(3);
    	XtestY = assist.getRawButton(4);
    	XtestLB = assist.getRawButton(5);
    	XtestRB = assist.getRawButton(6);
    	XtestBK  = assist.getRawButton(7);
    	XtestST = assist.getRawButton(8);
    	XtestLS = assist.getRawButton(9);
    	XtestRS = assist.getRawButton(10);
    	
    	leftEnRate = leftEn.getRate();
    	rightEnRate = rightEn.getRate();
    	leftEnRaw = leftEn.getRaw();
    	rightEnRaw = rightEn.getRaw();
    	intakeLimit = limitSwitch.get();
    	
        rotateLeft.clearStickyFaults();
    	rotateRight.clearStickyFaults();
    }
    public void NavXData(){//Put after Interface()
    	/* Display 6-axis Processed Angle Data                                      */
        SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
        SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
        SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
        SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
        SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
        
        /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
        SmartDashboard.putNumber(   "IMU_FusedHeading",     ahrs.getFusedHeading());

        /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
        /* path for upgrading from the Kit-of-Parts gyro to the navx MXP            */
        
        SmartDashboard.putNumber(   "IMU_TotalYaw",         ahrs.getAngle());
        SmartDashboard.putNumber(   "IMU_YawRateDPS",       ahrs.getRate());

        /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
        
        SmartDashboard.putNumber(   "IMU_Accel_X",          ahrs.getWorldLinearAccelX());
        SmartDashboard.putNumber(   "IMU_Accel_Y",          ahrs.getWorldLinearAccelY());
        SmartDashboard.putNumber(   "IMU_Accel_Z",          ahrs.getWorldLinearAccelZ());
        SmartDashboard.putBoolean(  "IMU_IsMoving",         ahrs.isMoving());
        
        /*CPU Temperature in Celsius                                                 */
        SmartDashboard.putNumber(   "IMU_Temp_C",           ahrs.getTempC());
        
        /* Omnimount Yaw Axis Information                                           */
        /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
        AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
        SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
        SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );
        
        /* Sensor Board Information                                                 */
        SmartDashboard.putString(   "FirmwareVersion",      ahrs.getFirmwareVersion());
        
        /* Connectivity Debugging Support                                           */
        SmartDashboard.putNumber(   "IMU_Byte_Count",       ahrs.getByteCount());
        SmartDashboard.putNumber(   "IMU_Update_Count",     ahrs.getUpdateCount());
    }
    public void Dashboard(){//Put after NavXData()
    	SmartDashboard.putData("Auto choices", chooser);
        SmartDashboard.putData("Arm Choices", arm);
    	
        SmartDashboard.putNumber("Left Stick", leftStick);
		SmartDashboard.putNumber("Right Stick", rightStick);
		SmartDashboard.putNumber("Left Output", leftOutput);
		SmartDashboard.putNumber("Right Output", rightOutput);
		
		SmartDashboard.putNumber("X Left Stick", XtestXLS);
		
		SmartDashboard.putBoolean("RotateFwd", rotateFwdRT);
		SmartDashboard.putBoolean("RotateBkwd", rotateBkwdLT);
        
		SmartDashboard.putNumber("Error", error);
		SmartDashboard.putNumber("IntError", intError);
		SmartDashboard.putNumber("P", P);
		SmartDashboard.putNumber("Left Motor", leftEnRate);
    	SmartDashboard.putNumber("Right Motor", -rightEnRate);
		SmartDashboard.putNumber("Left Distance", leftEnRaw);
		SmartDashboard.putNumber("Right Distance", rightEnRaw);		
    }
    public void Drive(){
    	// This is for driving the practice bot
    	/*if(slowLB){
    		leftOutput = -reduce*leftStick;
    		rightOutput = -reduce*rightStick;
    	}
    	else{
    		leftOutput = -leftStick;
    		rightOutput = -rightStick;
    	}
			myRobot.tankDrive(-leftOutput, -rightOutput, true);*/
    	//This is for real bot
    	if(slowLB){
    		leftOutput = -reduce*rightStick;
    		rightOutput = -reduce*leftStick;
    	}
    	else{
    		leftOutput = -rightStick;
    		rightOutput = -leftStick;
    	}
		myRobot.tankDrive(leftOutput, rightOutput, true);
	}
    public void Rotate(){
	   if(rotateFwdRT){
		   rotateLeft.set(-rotateSpeed*armInput);
		   rotateRight.set(-rotateSpeed*armInput);
	   }
	   else if(rotateBkwdLT){
		   rotateLeft.set(rotateSpeed*armInput);
		   rotateRight.set(rotateSpeed*armInput);
	   }
	   else{
		   rotateLeft.set(zero);
		   rotateRight.set(zero);
	   }
    }       
    public void Eject(){
    	if(shootA){
    		eject.set(-quarter);
    	}
    	else if(testB){
    		eject.set(quarter);
    	}
    	else{
    		eject.set(zero);
    	}
    }
    public void Intake(){
    	
    		if(spinUpLS){
    			leftIn.set(one);
    		}
    		else if(spinInRB){
    			leftIn.set(-half);
    		}
    		else{
    			leftIn.set(zero);
    		}
    	
    }
    public boolean Toggle(boolean button){
    	ONS1 = button;
    	if(ONS1 != false){
    		if(ONS1){
    			Toggle=!Toggle;
    			Timer.delay(threeT);
    		}
    	}
    	if(Toggle){
    		return true;
    	}
    	else{
    		return false;
    	}
    }
    public boolean Toggle1(boolean button1){
    	ONS = button1;
    	if(ONS != false){
    		if(ONS){
    			Toggle1=!Toggle1;
    			Timer.delay(threeT);
    		}
    	}
    	if(Toggle1){
    		return true;
    	}
    	else{
    		return false;
    	}
    }
}