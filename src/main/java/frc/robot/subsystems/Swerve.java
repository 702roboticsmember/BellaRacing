package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.LimelightHelpersCameronEdition;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Our main drive subsystem
 */
public class Swerve extends SubsystemBase {
public SwerveModule[] swerveModules;
public static AHRS gyro;
public RobotConfig config;
public LimelightHelpersCameronEdition.PoseEstimate limelightMeasurement;
public LimelightHelpersCameronEdition.PoseEstimate limelightMeasurementTurret;
public static SwerveDrivePoseEstimator swervePoseEstimator;


public Swerve(){
    // gyro = new Pigeon2(Constants.Swerve.GyroId, "rio");
    gyro = new AHRS(NavXComType.kMXP_SPI);
    gyro.reset();
    try {
        config = RobotConfig.fromGUISettings();
    } catch (IOException | ParseException e){
        e.printStackTrace();
    }

    swerveModules = new SwerveModule[]{
        new SwerveModule(0, Constants.Swerve.Mod1.constants),
        new SwerveModule(1, Constants.Swerve.Mod0.constants),
        new SwerveModule(2, Constants.Swerve.Mod2.constants),
        new SwerveModule(3, Constants.Swerve.Mod3.constants)
    };
    if(gyro.isConnected()){
        swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.KINEMATICS, getGyroYaw(), getModulePositions(), new Pose2d());
    }else{
        swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.KINEMATICS, new Rotation2d(Math.toRadians(0)), getModulePositions(), new Pose2d());
    }

    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getRobotRelativeSpeeds,
        (speeds , feedforwards) -> driveRobotRelative(speeds),
        Constants.Swerve.PATHPLANNER_FOLLOWER_CONFIG,
        config,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()){
                    if(alliance.get() == DriverStation.Alliance.Red){
                        
                    }else{
                        
                    }
                return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
    );
}

private void driveRobotRelative(ChassisSpeeds speeds) {
    var swerveModuleStates = Constants.Swerve.KINEMATICS.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.Swerve.MAX_SPEED);
    
    for (int i = 0; i < swerveModuleStates.length; i++) {
        swerveModules[i].setDesiredState(swerveModuleStates[i], false);
    }
}

private ChassisSpeeds getRobotRelativeSpeeds() {
    return Constants.Swerve.KINEMATICS.toChassisSpeeds(getModuleStates());
}

private void resetPose(Pose2d startingPosition) {
    SmartDashboard.putNumber("xi", startingPosition.getX());
    SmartDashboard.putNumber("yi", startingPosition.getY());
    SmartDashboard.putNumber("ai", startingPosition.getRotation().getDegrees());
    swervePoseEstimator.resetPosition(
        gyro.getRotation2d(),
        this.getModulePositions(),
        startingPosition);
}

public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }
    }

    public void driveAdjustedHeading(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, Rotation2d TurretOffset) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading().plus(TurretOffset))
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveModules) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : swerveModules) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swervePoseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    
public Rotation2d getGyroYaw(){

    return (Constants.Swerve.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - gyro.getAngle())
            : gyro.getRotation2d();
}
public void resetModulesToAbsoulute(){
    for(SwerveModule mod : swerveModules){
        mod.resetToAbsolute();
    }
}
/*public Pose2d limelightTurrertPoseAdjustedToRobot(Pose2d pose){
    double y = Constants.Swerve.LIMELIGHT_TURRET_POSE_Y;
    double x = -Constants.Swerve.LIMELIGHT_TURRET_POSE_X;
    Rotation2d a = pose.getRotation().minus(new Rotation2d(Math.toRadians(Constants.TurretConstants.angle)));
    Pose2d returnpose = new Pose2d(pose.getX()+(a.getCos()*x)- (a.getSin()*y), pose.getY() + (a.getCos()* y)- (a.getSin()* x), a);
    return returnpose;
}*/

/*public Pose2d RobotPoseAdjustedTolimelightTurret(Pose2d pose){
    double y = -Constants.Swerve.LIMELIGHT_TURRET_POSE_Y;
    double x = Constants.Swerve.LIMELIGHT_TURRET_POSE_X;
    Rotation2d a = getHeading();
    return new Pose2d(pose.getX() + (a.getCos()* x) - (a.getSin() * y), pose.getY() + (a.getCos()* y)- (a.getSin() * x), new Rotation2d(Math.toRadians(LimelightHelpersCameronEdition.getIMUData(Constants.limelightConstants.limelightTurret).robotYaw)));
}*/
public void addmt1VisionMeasurement(LimelightHelpersCameronEdition.PoseEstimate mt1){
        boolean doRejectUpdate = false;
        if (mt1 != null){
         if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
      {
        if(mt1.rawFiducials[0].ambiguity > .7)
        {
          doRejectUpdate = true;
        }
        if(mt1.rawFiducials[0].distToCamera > 1.5)
        {
          doRejectUpdate = true;
        }
      }
      if(mt1.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(mt1.avgTagDist > 5.8)
      {
        doRejectUpdate = true;
      }
      if(Double.isNaN(mt1.std[0]) || Double.isNaN(mt1.std[1]) || Double.isNaN(mt1.std[2])){
            mt1.std[0] = .05;
            mt1.std[1] =.05;
            mt1.std[2] = .1;
        }

      if(!doRejectUpdate)
      {
        
        swervePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(mt1.std[0] * 20,mt1.std[1] * 20, mt1.std[2]/2));
        swervePoseEstimator.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
            SmartDashboard.putBoolean("ran", true);
            SmartDashboard.putNumberArray("std", mt1.std);
             SmartDashboard.putNumber("dist",  mt1.avgTagDist);
      }
        }
    }
@Override
public void periodic(){
    if(Double.isNaN(swervePoseEstimator.getEstimatedPosition().getX())){
        swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.KINEMATICS, getGyroYaw(), getModulePositions(), new Pose2d());
    }
    limelightMeasurement = LimelightHelpersCameronEdition.getBotPoseEstimate_wpiBlue(Constants.limelightConstants.limelightBack);
    limelightMeasurementTurret = LimelightHelpersCameronEdition.getBotPoseEstimate_wpiBlue(Constants.limelightConstants.limelightTurret);
    if(gyro.isConnected())swervePoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getGyroYaw(), getModulePositions());
    SmartDashboard.putNumber("gyro", getHeading().getDegrees());
    for(SwerveModule mod : swerveModules){
    SmartDashboard.putNumber("Mod"+mod.moduleNumber + "CANcoder", mod.getCANcoder().getDegrees());
    SmartDashboard.putNumber("Mod"+mod.moduleNumber + "Angle", mod.getPosition().angle.getDegrees());
    SmartDashboard.putNumber("Mod"+mod.moduleNumber + "velocity", mod.getState().speedMetersPerSecond);
    }
    if(this.limelightMeasurementTurret != null){
        if(limelightMeasurementTurret.pose != null && limelightMeasurement.pose.getRotation() != null){
        //    Pose2d pose = limelightTurretPoseAdjustedToRobot(limelightMeasurementTurret.pose);
           // limelightMeasurementTurret.pose = pose;
            addmt1VisionMeasurement(limelightMeasurementTurret);
        }
    }
    ChassisSpeeds speed = getRobotRelativeSpeeds();
    Constants.Swerve.speeds = speed;
    SmartDashboard.putNumber("chassisx", speed.vxMetersPerSecond);
    SmartDashboard.putNumber("chassisy", speed.vyMetersPerSecond);
    if(this.limelightMeasurement != null){
        addmt1VisionMeasurement(limelightMeasurement);
    }
}
}