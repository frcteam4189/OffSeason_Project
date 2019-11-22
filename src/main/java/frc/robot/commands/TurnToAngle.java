/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class TurnToAngle extends Command {

  double Angle;
  boolean isFinished = false;
  boolean inErrorZone = false;
  public int count;

  public TurnToAngle(double angle) {
    requires(Robot.driveTrain);
    Angle = angle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveTrain.rotateDegrees(Angle);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double error = Robot.driveTrain.turnController.getError();
    inErrorZone = Math.abs(error) < 3;
    if(inErrorZone){
      count++;
      isFinished = count >= 5;
      System.out.print(count);
    }
    else{
      count = 0;
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isFinished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.turnController.disable();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

  public void outputIntel(){
    SmartDashboard.putBoolean("Running PID?", isFinished);
    SmartDashboard.putData("Turn To Angle", new TurnToAngle(90));
  }


}
