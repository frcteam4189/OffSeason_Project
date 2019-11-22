/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Gains;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithJoySticks;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem implements PIDOutput{
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public WPI_TalonSRX leftMaster;
  public WPI_TalonSRX rightMaster;
  public WPI_TalonSRX leftFollower;
  public WPI_TalonSRX rightFollower;
  public final AHRS ahrs;
  public DifferentialDrive chasisDrive;

  public final PIDController turnController;

  public DriveTrain(){
    //Differenetial Drive??
    leftMaster = new WPI_TalonSRX(RobotMap.leftFrontPort);
    rightMaster = new WPI_TalonSRX(RobotMap.rightFrontPort);
    leftFollower = new WPI_TalonSRX(RobotMap.leftBackPort);
    rightFollower = new WPI_TalonSRX(RobotMap.rightBackPort);
    ahrs = new AHRS(SPI.Port.kMXP);

    Robot.initTalon(leftMaster);
    Robot.initTalon(rightMaster);
    Robot.initTalon(leftFollower);
    Robot.initTalon(rightFollower);
    
    // leftFollower.follow(leftMaster);
    // rightFollower.follow(rightMaster);

    SpeedControllerGroup leftGroup = new SpeedControllerGroup(leftMaster, leftFollower);
    SpeedControllerGroup rightGroup = new SpeedControllerGroup(rightMaster, rightFollower);

    turnController = new PIDController(Gains.kP, Gains.kI, Gains.kD, ahrs, this);
    turnController.setInputRange(-180.0f, 180.0f);
    turnController.setOutputRange(-.6, .6);
    turnController.setAbsoluteTolerance(3.0f);
    turnController.setContinuous();
    chasisDrive = new DifferentialDrive(leftGroup, rightGroup);
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
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveWithJoySticks());
  }

  @Override
  public void pidWrite(double output) {
    setSpeed(-output, output);
  }
}
