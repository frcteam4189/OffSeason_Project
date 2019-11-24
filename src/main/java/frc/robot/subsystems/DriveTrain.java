/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;
import frc.robot.Gains;
import frc.robot.Robot;
import frc.robot.commands.DriveWithJoySticks;
import jaci.pathfinder.followers.EncoderFollower;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class DriveTrain extends Subsystem implements PIDOutput{
  public WPI_TalonSRX driveLeftMaster;
  public WPI_TalonSRX driveRightMaster;
  public WPI_TalonSRX driveLeftSlave;
  public WPI_TalonSRX driveRightslave;
  public Encoder rightEncoder;
  public Encoder leftEncoder;
  public EncoderFollower rightEncoderFollower;
  public EncoderFollower leftEncoderFollower;
  public final AHRS ahrs;
  public Notifier followerNotifier;
  public DifferentialDrive chasisDrive;
  public SpeedControllerGroup driveLeftGroup;
  public SpeedControllerGroup driveRightGroup;
  

  public final PIDController turnController;

  public DriveTrain(){
    driveLeftMaster = new WPI_TalonSRX(Constants.kDriveLeftMasterID);
    driveRightMaster = new WPI_TalonSRX(Constants.kDriveRightMasterID);
    driveLeftSlave = new WPI_TalonSRX(Constants.kDriveLeftSlaveID);
    driveRightslave = new WPI_TalonSRX(Constants.kDriveRightSlaveID);

    Robot.initTalon(driveLeftMaster);
    Robot.initTalon(driveRightMaster);
    Robot.initTalon(driveLeftSlave);
    Robot.initTalon(driveRightslave);
    
    // leftFollower.follow(leftMaster);
    // rightFollower.follow(rightMaster);

    leftEncoder = new Encoder(Constants.kLeftEncoderPortA, Constants.kLeftEncoderPortB);
    rightEncoder = new Encoder(Constants.kRightEncoderPortA, Constants.kRightEncoderPortB);
    ahrs = new AHRS(SPI.Port.kMXP);

    driveLeftGroup = new SpeedControllerGroup(driveLeftMaster, driveLeftSlave);
    driveRightGroup = new SpeedControllerGroup(driveRightMaster, driveRightslave);

    turnController = new PIDController(Gains.kP, Gains.kI, Gains.kD, ahrs, this);
    turnController.setInputRange(-180.0f, 180.0f);
    turnController.setOutputRange(-.7, .7);
    turnController.setAbsoluteTolerance(0.25f);
    turnController.setContinuous();
    chasisDrive = new DifferentialDrive(driveLeftGroup, driveRightGroup);
  }  

  public void rotateDegrees(double angle){
    ahrs.reset();
    turnController.reset();
    turnController.setPID(Gains.kP, Gains.kI, Gains.kD);
    turnController.setSetpoint(angle);
    turnController.enable();
  }
  
  
  public void setSpeed(double leftSpeed, double rightSpeed){
    chasisDrive.tankDrive(leftSpeed, rightSpeed);

  }
  public void outputIntel() {
    SmartDashboard.putNumber("Gyro Angle", ahrs.getAngle());
    SmartDashboard.putString("Drive Train", Robot.driveTrain.getSubsystem());
    SmartDashboard.putNumber("Accel Z", ahrs.getRawAccelZ());
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveWithJoySticks());
  }

  @Override
  public void pidWrite(double output) {
    setSpeed(-output, output);
  }
}
