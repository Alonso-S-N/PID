package frc.robot.command.Drive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.SubSystem.Drive;
import frc.robot.Calcs;
import frc.robot.Calcs.DriveSpeeds;

public class Loc extends Command {
  
  private final Drive driveSubsystem;
  private final Joystick joyDeliciu;
  DriveSpeeds speeds;
  
    private double B_Speed = 0;
    private boolean a, b, x;
  
    public Loc(Drive driveSubsystem,Joystick joyDeliciu) {
      this.driveSubsystem = driveSubsystem;
      this.joyDeliciu = joyDeliciu;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.reqDrive();
  }

  @Override
  public void execute() {
    Smart();
    button();
    MainControl();
    SmartDashboard.putString("Loc Status", "Rodando");
  }

private void setDriveSpeeds(double left, double right) {
    if (speeds != null) {
        driveSubsystem.rawTank(left, right);
    } else {
    }
}

  private void stopDrive() {
    setDriveSpeeds(0, 0);
  }

  public void button(){
    a = joyDeliciu.getRawButton(Constants.a);
    b = joyDeliciu.getRawButton(Constants.b);
    x = joyDeliciu.getRawButton(Constants.x);

    if (a) B_Speed = 0.25;
    else if (b) B_Speed = 0.5;
    else if (x) B_Speed = 1.0;
  }

  public void MainControl() {
    double X = joyDeliciu.getX();
    double Y = joyDeliciu.getY();
    double X1 = joyDeliciu.getRawAxis(Constants.X1);
    double Y2 = joyDeliciu.getRawAxis(Constants.Y2);

    if (joyDeliciu.getPOV() != Constants.povDeadZone){
      speeds = Calcs.calculatePovDrive(joyDeliciu, B_Speed);
    }
    else if (Math.abs(X) >= Constants.deadZone || Math.abs(Y) >= Constants.deadZone || Math.abs(X) < Constants.NegativeDeadZone || Math.abs(Y) < Constants.NegativeDeadZone) {
      speeds = Calcs.calculateAnalogDrive(joyDeliciu, B_Speed); 
    }
      else if (Math.abs(X1) >= Constants.deadZone || Math.abs(Y2) >= Constants.deadZone || Math.abs(X1) < Constants.NegativeDeadZone || Math.abs(Y2) < Constants.NegativeDeadZone){
       speeds = Calcs.calculateAnalogDrive2(joyDeliciu,B_Speed);
    }
      else {
      stopDrive();
      speeds = null;
      return;
    }
  
    setDriveSpeeds(speeds.left, speeds.right);
  }

  public void Smart(){
    if (speeds != null) {
      SmartDashboard.putNumber("Left Speed", speeds.left);
      SmartDashboard.putNumber("Right Speed", speeds.right);
    } else {
      SmartDashboard.putNumber("Left Speed", 0);
      SmartDashboard.putNumber("Right Speed", 0);
    }
    SmartDashboard.putBoolean("Button A", a);
    SmartDashboard.putBoolean("Button B", b);
    SmartDashboard.putBoolean("Button X", x);
    SmartDashboard.putNumber("VelB", B_Speed);
  }



  @Override
  public void end(boolean interrupted) {
    stopDrive();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}