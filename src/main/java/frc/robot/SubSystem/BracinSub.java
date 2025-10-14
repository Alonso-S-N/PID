// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SubSystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calcs;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.MathUtil;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class BracinSub extends SubsystemBase {

//Encoders de simulação:
  public final SparkMax armMotor = new SparkMax(Constants.m_Bracin, MotorType.kBrushless);
  public final SparkMax intakeMotor = new SparkMax(Constants.m_Intake, MotorType.kBrushless);
  Encoder armEncoder = new Encoder (0, 1, false, Encoder.EncodingType.k4X);
  Encoder IntakeEncoder = new Encoder (2, 3, false, Encoder.EncodingType.k4X);
  EncoderSim armEncoderSim = new EncoderSim(armEncoder);
  EncoderSim intakeEncoderSim = new EncoderSim(IntakeEncoder);
  
  //encoder do Motor(Braço):
    private final RelativeEncoder armSparkEncoder = armMotor.getEncoder();
  //encoder do Motor(Intake):
    private final RelativeEncoder intakeSparkEncoder = intakeMotor.getEncoder();
  //Razão de redução do Braço:
    private static final double ARMGEAR_RATIO = 100.0;

    ArmFeedforward feedforward = new ArmFeedforward(Constants.ks,Constants.kg, Constants.kv, Constants.ka);

    PIDController ArmController = new PIDController(0.001, 0.00001,0.0005);
    double ArmOutPut;


    public final double posSubino = (90 / Constants.GrausMax) * ARMGEAR_RATIO;
    public final double posDesceno = 0;

    private final LoggedMechanism2d mech = new LoggedMechanism2d(0.7, 0.5);
    private final LoggedMechanismRoot2d base = mech.getRoot("Base", 0.6, 0.7);
    private final LoggedMechanismLigament2d armLig = base.append(
        new LoggedMechanismLigament2d("Arm", 0.5, 90,10,new Color8Bit(192,192,192)));
    private final LoggedMechanismLigament2d intakeLig = armLig.append(
        new LoggedMechanismLigament2d("Intake", 0.3, 90,5,new Color8Bit(255,0,0)));


  public BracinSub() {
    armSparkEncoder.setPosition(0);
    intakeSparkEncoder.setPosition(0);
    armEncoder.setDistancePerPulse(1.0 / ARMGEAR_RATIO);
    IntakeEncoder.setDistancePerPulse(1.0 / ARMGEAR_RATIO);

    ArmController.setTolerance(Constants.tolerancia);
  }
  
  public void AcertaOCantoAi(double setpointRotations) {
       // Posição atual do encoder
       double posicaoAtual = armSparkEncoder.getPosition();

    
       if (Math.abs(setpointRotations - posicaoAtual) < Constants.tolerancia) {
           armMotor.setVoltage(
               feedforward.calculate(Math.toRadians(getArmAngleDegrees()), 0)
           );
           return; 
       }

    double pidOutput = ArmController.calculate(posicaoAtual, setpointRotations);

    double anguloAlvoRad = Math.toRadians((setpointRotations / ARMGEAR_RATIO) * Constants.GrausMax);
    double feedforwardOutput = feedforward.calculate(anguloAlvoRad, 0); // (angulo, velocidade)


    double totalVoltage = pidOutput + feedforwardOutput;
    totalVoltage = MathUtil.clamp(totalVoltage, Constants.MinVoltage, Constants.MaxVoltage); // Garante que não passe de 12V


    armMotor.setVoltage(totalVoltage);

    if (RobotBase.isSimulation()){
      Logger.recordOutput("Arm/PID/SetpointRotations", setpointRotations);
      Logger.recordOutput("Arm/PID/PositionRotations", posicaoAtual);
      Logger.recordOutput("Arm/PID/Error", setpointRotations - posicaoAtual);
      Logger.recordOutput("Arm/PID/OutputVolts", pidOutput);
      Logger.recordOutput("Arm/TotalVoltage", totalVoltage);
      Logger.recordOutput("Arm/AngleDegrees", getArmAngleDegrees());

    }
}

  public void IrPraUmCantoAi(){
   AcertaOCantoAi(posSubino);
  }
  
  public void VoltarProOutroCantoAi(){
   AcertaOCantoAi(posDesceno);
  }

  public double GetArmPose(){
  double pos = armSparkEncoder.getPosition();
  return pos;
  }

  public double getArmAngleDegrees() {
    return (armSparkEncoder.getPosition() / ARMGEAR_RATIO) * Constants.GrausMax;
}

  public void MexePruLado() {
    armMotor.set(0.10);
  }

  public void MexePruOutro(){
    armMotor.set(-0.10);
  }

  public void StopBraceta(){
    armMotor.set(0.0);
  }
  
  public void Cuspir(){
    intakeMotor.set(0.7);
  }
  public void Pegar(){
    intakeMotor.set(-0.7);
  }
  public void stopIntake(){
    intakeMotor.set(0.0);
  }

  @Override
  public void simulationPeriodic() {
      double motorOut = armMotor.get();
      double IntakeOut = intakeMotor.get();
      double simulateRate = IntakeOut * 5.0;
      double simulatedRate = motorOut * 5.0; 
      armEncoderSim.setRate(simulatedRate);
      armEncoderSim.setDistance(armEncoder.getDistance() + simulatedRate * 0.02);
      intakeEncoderSim.setRate(simulateRate);
      intakeEncoderSim.setDistance(IntakeEncoder.getDistance() + simulateRate * 0.02);

      intakeEncoderSim.getDistance();
      armEncoderSim.getDistance(); 
      double armAngleDeg = armEncoder.getDistance() * Constants.GrausMax; // em graus
      armLig.setAngle(armAngleDeg);
      double intakeAngleDeg = IntakeEncoder.getDistance() * Constants.GrausMax;
      intakeLig.setAngle(intakeAngleDeg);
      
      double armAngleRad = armEncoder.getDistance() * 2 * Math.PI; 
      Logger.recordOutput("Arm/Mechanism", new edu.wpi.first.math.geometry.Rotation2d(armAngleRad));
      Logger.recordOutput("Arm/PositionTicks", armEncoderSim.getDistance());
      Logger.recordOutput("Arm/Mechanism2d", mech);
      Logger.recordOutput("ArmPosition", armSparkEncoder.setPosition(armEncoderSim.getDistance()));
  
  
      Logger.recordOutput("Arm/VelocityTicksPerSec", armEncoder.getRate());
      Logger.recordOutput("Arm/VelocityPerTicksPerSecPID", armSparkEncoder.getVelocity() / 60.0 * ARMGEAR_RATIO);
  
      Logger.recordOutput("Arm/AppliedOutput", armMotor.getAppliedOutput());
  
      // Intake
      Logger.recordOutput("Intake/AppliedOutput", intakeMotor.getAppliedOutput());
      Logger.recordOutput("Intake/PositionTicks", IntakeEncoder.getDistance());

      Logger.recordOutput("Arm/PID/kP", ArmController.getP());
      Logger.recordOutput("Arm/PID/kI", ArmController.getI());
      Logger.recordOutput("Arm/PID/kD", ArmController.getD());
  }

  @Override
  public void periodic() {
   
  }
}
