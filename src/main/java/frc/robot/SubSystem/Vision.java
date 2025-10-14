package frc.robot.SubSystem;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.IOException;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

public class Vision extends SubsystemBase {

        private double tx,ta,ty;
        private boolean tv;
        private final VisionSystemSim visionSim = new VisionSystemSim("limelight-front");
        TargetModel targetModel = new TargetModel(0.5, 0.25);

     
        private final NetworkTable limelight;
                
                
                    private final LoggedMechanism2d Limelight = new LoggedMechanism2d(0.3, 0.1);
                    private final LoggedMechanismRoot2d base = Limelight.getRoot("Limelight", 0.6, 0.3);
                    private final LoggedMechanismLigament2d limelightLig = base.append(
                        new LoggedMechanismLigament2d("Limelight", 0.1, 90,5,new Color8Bit(0,0,0)));
                    private final Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));
                    private final VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);
                    private final SimCameraProperties cameraProp = new SimCameraProperties();
                    
                           
                    
                    public Vision(){
                    limelight = NetworkTableInstance.getDefault().getTable("limelight-front");

                 
                    cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));   
                    cameraProp.setCalibError(0.25, 0.08);
                    cameraProp.setFPS(60);
                    cameraProp.setAvgLatencyMs(35);
                    cameraProp.setLatencyStdDevMs(5);

                    Translation3d robotToCameraTrl = new Translation3d(0.6, 0.3, 0.5);
                    Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
                    Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

                    AprilTagFieldLayout tagLayout;
                    try {
                        tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
                    } catch (IOException e) {
                        e.printStackTrace();
                        tagLayout = null; 
                    }    
                    visionSim.addAprilTags(tagLayout);
                   
        }
    
        public boolean hasTarget() {
         return limelight.getEntry("tv").getDouble(0.0) == 1.0;
        }
    
        public double getTx() {
         return limelight.getEntry("tx").getDouble(0.0);
        }
    
        public double getTa() {
         return limelight.getEntry("ta").getDouble(0.0);
        }
    
        @Override
        public void periodic() {
            SmartDashboard.putBoolean("Limelight Target", hasTarget());
            SmartDashboard.putNumber("Limelight TX", getTx());
            SmartDashboard.putNumber("Limelight TA", getTa());
          
        }
    
        @Override
        public void simulationPeriodic(){
        Logger.recordOutput("LimelightMech",Limelight);
        Logger.recordOutput("Limelight/tx", tx);
        Logger.recordOutput("Limelight/ty", ty);
        Logger.recordOutput("Limelight/ta", ta);
        Logger.recordOutput("Limelight/tv", tv);
      
        visionSim.getDebugField();
    }
 }

