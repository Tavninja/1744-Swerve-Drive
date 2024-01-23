// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot;
 
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 
import com.revrobotics.AbsoluteEncoder;
//import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.AnalogInput;
//import edu.wpi.first.wpilibj.AnalogAccelerometer;
import edu.wpi.first.wpilibj.AnalogEncoder;
//import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
 
public class Robot extends TimedRobot {
 
  public static final double kFrontLeftAbsoluteEncoderRawValue = 1;
  public static final double kBackLeftAbsoluteEncoderRawValue = 1;
  public static final double kFrontRightAbsoluteEncoderRawValue = 1;
  public static final double kBackRightAbsoluteEncoderRawValue = 1;
 
// controllers
 
  private final XboxController m_operatorController = new XboxController(1);
 
//angle motors
 
  public final  CANSparkMax ma_frontLeft = new CANSparkMax(1, MotorType.kBrushless);
  public final  CANSparkMax ma_frontRight = new CANSparkMax(2, MotorType.kBrushless);
  public final  CANSparkMax ma_backLeft = new CANSparkMax(3, MotorType.kBrushless);
  public final  CANSparkMax ma_backRight = new CANSparkMax(4, MotorType.kBrushless);
 
//speed motors
 
  public final  CANSparkMax ms_frontLeft = new CANSparkMax(5, MotorType.kBrushless);
  public final  CANSparkMax ms_frontRight = new CANSparkMax(6, MotorType.kBrushless);
  public final  CANSparkMax ms_backLeft = new CANSparkMax(7, MotorType.kBrushless);
  public final  CANSparkMax ms_backRight = new CANSparkMax(8, MotorType.kBrushless);
  private final MotorControllerGroup motorspeed = new MotorControllerGroup(ms_backLeft, ms_backRight,ms_frontLeft,ms_frontRight);
 
//Anolog Encoders
 
  AnalogEncoder frontLeftEncoder = new AnalogEncoder(0);
  AnalogEncoder frontRightEncoder = new AnalogEncoder(1);
  AnalogEncoder backLeftEncoder = new AnalogEncoder(2);
  AnalogEncoder backRightEncoder = new AnalogEncoder(3);
 
  frontLeftEncoder.setPositionOffset(0.1);
 
 
//length and width to wheels?
 
  double L = 1;
  double W = 1;
  double motorAngleUp = .5;
  double motorAngledown = -.5;
 
// motor power
 
  @Override
  public void teleopPeriodic() {
 
   double y1 = m_operatorController.getLeftY()*.25;
   double x1 = m_operatorController.getLeftX()*.25;
   double x2 = m_operatorController.getRightX()*.25;
 
   double r = Math.sqrt((L*L)+(W*W)); 
   y1 *= -1;
 
   double a = x1 - x2 * (L/r);
   double b = x1 + x2 * (L/r);
   double c = y1 - x2 * (W/r);
   double d = y1 + x2 * (W/r);
 
   //calculate power
 
   double backRightPower = Math.sqrt((a*a)+(d*d));
   double backLeftPower= Math.sqrt((a*a)+(c*c));
   double frontRightPower = Math.sqrt((b*b)+(d*d));
   double frontLeftPower = Math.sqrt((b*b)+(c*c));
 
    //wanted angle
 
    double wantedFrontLeftAngle = ((Math.atan2(a,d)/Math.PI)/2)+.5;
    double wantedFrontRightAngle =((Math.atan2(a,c) / Math.PI)/2)+.5;
    double wantedBackLeftAngle = ((Math.atan2(b,d) / Math.PI)/2)+.5;
    double wantedBackRightAngle = ((Math.atan2(b,c) / Math.PI)/2)+.5;
 
    //set power
 
    ms_frontLeft.set(frontLeftPower);
    ms_frontRight.set(frontRightPower);
    ms_backLeft.set(backLeftPower);
    ms_backRight.set(backRightPower);
 
    //get encoder angle
 
    double RealFrontLeftAngle = (frontLeftEncoder.getAbsolutePosition());
    double RealFrontRightAngle = (frontRightEncoder.getAbsolutePosition());
    double RealBackLeftAngle = (backLeftEncoder.getAbsolutePosition());
    double RealBackRightAngle = (backRightEncoder.getAbsolutePosition());
 
    //display wanted angles
    SmartDashboard.putNumber("Wanted Front Left Angle", wantedFrontLeftAngle);
    SmartDashboard.putNumber("Wanted Front Right Angle", wantedFrontRightAngle);
    SmartDashboard.putNumber("Wanted Back Left Angle", wantedBackLeftAngle);
    SmartDashboard.putNumber("Wanted Back Right Angle", wantedBackRightAngle);
 
    //display input values
    SmartDashboard.putNumber("Y1", y1 );
    SmartDashboard.putNumber("X1", x1 );
    SmartDashboard.putNumber("X2", x2 ); 
 
    //display speed
    SmartDashboard.putNumber("FL power", frontLeftPower);
    SmartDashboard.putNumber("FR power", frontRightPower);
    SmartDashboard.putNumber("BL power", backLeftPower);
    SmartDashboard.putNumber("BR power", backRightPower);
 
    //wanted angles
    SmartDashboard.putNumber("Real Front Left Angle", RealFrontLeftAngle);
    SmartDashboard.putNumber("Real Front Right Angle", RealFrontRightAngle);
    SmartDashboard.putNumber("Real back Left Angle", RealBackLeftAngle);
    SmartDashboard.putNumber("Real back Right Angle", RealBackRightAngle);
 
  //make wanted angle, real angle
/* 
    if(wantedFrontLeftAngle < RealFrontLeftAngle){ ma_frontLeft.set(motorAngleUp); 
    } else if(wantedFrontLeftAngle > RealFrontLeftAngle){ ma_frontLeft.set(motorAngledown);
    } else { ma_frontLeft.set(0);}
 
    //front right
    if(wantedFrontRightAngle < RealFrontRightAngle){ ma_frontRight.set(motorAngleUp); 
    } else if(wantedFrontRightAngle > RealFrontRightAngle){ ma_frontRight.set(motorAngledown);
    } else { ma_frontRight.set(0);}
    //back left
    if(wantedBackLeftAngle < RealBackLeftAngle){ ma_backLeft.set(motorAngleUp); 
    } else if(wantedBackLeftAngle > RealBackLeftAngle){ ma_backLeft.set(motorAngledown);
    } else { ma_backLeft.set(0);}
    //back right
    if(wantedBackRightAngle < RealBackRightAngle){ ma_backRight.set(motorAngleUp); 
    } else if(wantedBackRightAngle > RealBackRightAngle){ ma_backRight.set(motorAngledown);
    } else { ma_backRight.set(0);}
*/
  }
 
  @Override
  public void robotPeriodic() {}
 
  @Override
  public void autonomousInit() {}
 
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}
 
  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}
 
  /** This function is called periodically during operator control. */
 
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}
 
  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}
 
  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}
 
  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
 
  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}
 
  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}