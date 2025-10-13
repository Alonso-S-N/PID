// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SubSystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calcs;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

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

    public final double posSubino = (90 / 360.0) * ARMGEAR_RATIO;
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


   

   SparkMaxConfig cfg = new SparkMaxConfig();

   cfg.idleMode(SparkBaseConfig.IdleMode.kBrake);

   armMotor.getEncoder().setPosition(Constants.sensorPos);
   cfg.encoder.positionConversionFactor(1.0);
   cfg.closedLoop
       .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
       .p(0.001).i(0.00001).d(0.0005)
       .velocityFF(0.005)
       .outputRange(-0.5, 0.5);

   armMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
   
  }

  public void IrPraUmCantoAi(){
    armMotor.getClosedLoopController().setReference(posSubino, ControlType.kPosition);
  }
  
  public void VoltarProOutroCantoAi(){
    armMotor.getClosedLoopController().setReference(posDesceno, ControlType.kPosition);
  }

  public double GetArmPose(){
  double pos = armSparkEncoder.getPosition();
  return pos;
  }

  public double getArmAngleDegrees() {
    return (armSparkEncoder.getPosition() / ARMGEAR_RATIO) * 360.0;
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
      double simulatedRate = motorOut * 5.0; 
      armEncoderSim.setRate(simulatedRate);
      armEncoderSim.setDistance(armEncoder.getDistance() + simulatedRate * 0.02);

      double posAtual = armSparkEncoder.getPosition();
      double posAlvo = posSubino; // ou uma variável se quiser dinâmico
      double erro = posAlvo - posAtual;
      double saida = armMotor.getAppliedOutput();

      IntakeEncoder.getDistance();
      armEncoder.getDistance(); 
      double armAngleDeg = armEncoder.getDistance() * 360.0; // em graus
      armLig.setAngle(armAngleDeg);
      double intakeAngleDeg = IntakeEncoder.getDistance() * 360.0;
      intakeLig.setAngle(intakeAngleDeg);
  
      double armAngleRad = armEncoder.getDistance() * 2 * Math.PI; 
      Logger.recordOutput("Arm/Mechanism", new edu.wpi.first.math.geometry.Rotation2d(armAngleRad));
      Logger.recordOutput("Arm/PositionTicks", armEncoder.getDistance());
      Logger.recordOutput("Arm/Mechanism2d", mech);
  
  
      Logger.recordOutput("Arm/VelocityTicksPerSec", armEncoder.getRate());
  
  
      Logger.recordOutput("Arm/AppliedOutput", armMotor.getAppliedOutput());
  
      // Intake
      Logger.recordOutput("Intake/AppliedOutput", intakeMotor.getAppliedOutput());
      Logger.recordOutput("Intake/PositionTicks", IntakeEncoder.getDistance());


      Logger.recordOutput("Arm/PID/Position", posAtual);
      Logger.recordOutput("Arm/PID/Target", posAlvo);
      Logger.recordOutput("Arm/PID/Error", erro);
      Logger.recordOutput("Arm/PID/Output", saida);

      Logger.recordOutput("Arm/PID/kP", 0.001);
      Logger.recordOutput("Arm/PID/kI", 0.00001);
      Logger.recordOutput("Arm/PID/kD", 0.0005);


  }

  @Override
  public void periodic() {
   
  }
}
