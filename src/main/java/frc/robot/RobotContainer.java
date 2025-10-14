package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.SubSystem.Drive;
import frc.robot.SubSystem.Vision;
import frc.robot.SubSystem.BracinSub;
import frc.robot.command.Auto.AutonomousCommand;
import frc.robot.command.Drive.Loc;
import frc.robot.command.Drive.PidCommand;

public class RobotContainer {

  // Subsystems
  private final Drive driveSubsystem = new Drive();

  // Input
  public final Joystick joyDeliciu = new Joystick(Constants.joy);

  // Commands
  private final Loc locCommand;
  
  private final AutonomousCommand auto;

  private final Vision vision = new Vision();

  private final PidCommand Pdiddy;
  
  private final BracinSub baby = new BracinSub();

  public RobotContainer() {

    Pdiddy = new PidCommand(baby,joyDeliciu);

    CommandScheduler.getInstance().registerSubsystem(driveSubsystem);

    // Initialize Loc command with drive subsystem and joystick
    locCommand = new Loc(driveSubsystem,joyDeliciu);

    auto = new AutonomousCommand(driveSubsystem,vision,Constants.targetArea);

    // Set default command
    driveSubsystem.setDefaultCommand(locCommand);

    baby.setDefaultCommand(Pdiddy);

  }

  public Command getAutonomousCommand(){
      return auto;
  
  }

  public Command getBracinCommand(){
    return Pdiddy;
  }

}
