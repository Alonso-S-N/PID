// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubSystem.BracinSub;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PidCommand extends Command {
  private final BracinSub braceta;
  private Boolean YPressed = false;
  private boolean indoPraCima = false;
  private boolean ParouOBracin = false;
  private boolean Xpressed = false;
  
    private Joystick joyDelicioso = new Joystick(Constants.JoyDelicioso);
    
      public PidCommand(BracinSub braceta,Joystick joyDelicioso) {
       this.braceta = braceta;
       this.joyDelicioso = joyDelicioso;
     addRequirements(braceta);
    }
  
  
    @Override
    public void initialize() {
    }
  
    public void mexerBracinSlk(){
    if (joyDelicioso.getRawAxis(Constants.RT) > 0.1){
      braceta.MexePruLado();
    } else if (joyDelicioso.getRawAxis(Constants.LT) > 0.1){
      braceta.MexePruOutro();
    } else {
      braceta.StopBraceta();
    }
     //if (joyDelicioso.getRawButton(Constants.RB)){
       // braceta.MexePru(ANgulinQnoisQuer);
     // } else if (joyDelicioso.getRawButton(Constants.LB)){
       // braceta.MexePru(ANgulinQnoisQuer2);
      //}  
    }
  
    public void mexerIntakeSlk(){
      if (joyDelicioso.getRawButton(Constants.RB)){
        braceta.Pegar();
      } else if (joyDelicioso.getRawButton(Constants.LB)){
        braceta.Cuspir();
      } else {
        braceta.stopExtensor();
      }
    }
  
    public void MexerComPIDSLK(){
      if (YPressed){
        braceta.AcertaOCantoAi(braceta.posSubino);  
        if (braceta.ArmController.atSetpoint()){
        braceta.MoveTo(braceta.posIntakeExtend);
      }        
      } else if (Xpressed){ 
        braceta.MoveTo(braceta.posIntakeRetract);
        if (braceta.ExtensorController.atSetpoint()){
        braceta.AcertaOCantoAi(braceta.posDesceno);
    }
  }
}

   public void ButtonYgetPressed(){ 
    if (joyDelicioso.getRawButtonPressed(Constants.y)){
       YPressed = true; 
       } 
     }

    public void buttonXgetPressed(){
      if (joyDelicioso.getRawButtonPressed(Constants.x)){
        Xpressed = true;
        YPressed = false;
        
        }
    }


  public void StopBraceta(){
    if (joyDelicioso.getRawButtonPressed(Constants.b)){
      ParouOBracin = !ParouOBracin;
    braceta.StopBraceta();
    YPressed = false; // Cancela o modo PID
    Xpressed = false; // Cancela o modo PID
    braceta.ArmController.reset();
    braceta.ExtensorController.reset();
    }
  }

  
  @Override
  public void execute() {
    joyDelicioso.getRawButton(Constants.RB);
    joyDelicioso.getRawButton(Constants.LB);
    joyDelicioso.getRawAxis(Constants.RT);
    joyDelicioso.getRawAxis(Constants.LT);
    joyDelicioso.getRawButton(Constants.y);
    joyDelicioso.getRawButton(Constants.b);
    joyDelicioso.getRawButton(Constants.x);


   ButtonYgetPressed();
   buttonXgetPressed();
   StopBraceta();
   
   if (Xpressed || YPressed){
    MexerComPIDSLK();
   }else {
   mexerBracinSlk();
   mexerIntakeSlk();
   }

   SmartDashboard.putNumber("velocidade Braceta", braceta.armMotor.getAppliedOutput());
   SmartDashboard.putNumber("velocidade Intake", braceta.ExtensorMotor.getAppliedOutput());
   SmartDashboard.putBoolean("Button Y", joyDelicioso.getRawButton(Constants.y));
   SmartDashboard.putBoolean("Y Pressed", YPressed);
  }


  @Override
  public void end(boolean interrupted) {
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
