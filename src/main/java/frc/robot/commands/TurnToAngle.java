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

  @Override
  protected void initialize() {
    Robot.driveTrain.rotateDegrees(Angle);
  }

  @Override
  protected void execute() {
    double error = Robot.driveTrain.turnController.getError();
    inErrorZone = Math.abs(error) < .25;
    if(inErrorZone){
      //isFinished = true;
      count++;
      isFinished = count >= 3;
      //System.out.print(count);
    }
    else{
      count = 0;
    }
  }

  @Override
  protected boolean isFinished() {
    return isFinished;
  }

  @Override
  protected void end() {
    Robot.driveTrain.turnController.disable();
  }

  @Override
  protected void interrupted() {
    end();
  }

  public void outputIntel(){
    System.out.print("Hello");
    SmartDashboard.putBoolean("Running PID?", isFinished);
    SmartDashboard.putData("Turn To Angle", new TurnToAngle(90));
  }

}
