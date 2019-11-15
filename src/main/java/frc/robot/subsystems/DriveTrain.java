/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Gains;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
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

  public final PIDController turnController;

  public DriveTrain(){

    //Test
    leftMaster = new WPI_TalonSRX(0);
    rightMaster = new WPI_TalonSRX(1);
    leftFollower = new WPI_TalonSRX(2);
    rightFollower = new WPI_TalonSRX(3);
    ahrs = new AHRS(SPI.Port.kMXP);

    Robot.initTalon(leftMaster);
    Robot.initTalon(rightMaster);
    Robot.initTalon(leftFollower);
    Robot.initTalon(rightFollower);
    
    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);

    turnController = new PIDController(Gains.kP, Gains.kI, Gains.kD, ahrs, this);
    turnController.setInputRange(-180.0f, 180.0f);
    turnController.setOutputRange(-.3, .3);
    turnController.setAbsoluteTolerance(3.0f);
    turnController.setContinuous();
  }  

  public void rotateDegrees(double angle){
    ahrs.reset();
    turnController.reset();
    turnController.setPID(Gains.kP, Gains.kI, Gains.kD);
    turnController.setSetpoint(angle);
    turnController.enable();
  }

  public void setSpeed(ControlMode mode, double leftSpeed, double rightSpeed){
    leftMaster.set(mode, leftSpeed);
    rightMaster.set(mode, rightSpeed);

  }
  public void dashData() {
    SmartDashboard.putNumber("Gyro Angle", ahrs.getAngle());
    
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  public void pidWrite(double output) {
    setSpeed(ControlMode.PercentOutput, -output, output);
  }
}
