/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.TurnToAngle;

public class OI {
  //public XboxController driveController = new XboxController(Constants.kDriveControllerPort); 
  public Joystick leftStick = new Joystick(Constants.kLeftStickPort);
  public Joystick rightStick = new Joystick(Constants.kRightStickPort);
  public Button turn = new JoystickButton(leftStick, Constants.kTurnToAngle);

  public OI(){
    turn.whenPressed(new TurnToAngle(90));
    
    // if(driveController.getAButton()){
    //   new TurnToAngle(90);
    // }
  }
}
