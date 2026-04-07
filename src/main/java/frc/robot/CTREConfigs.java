package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();
    public TalonFXConfiguration shooterConfigs = new TalonFXConfiguration();
    public TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();
    public TalonSRXConfiguration turretConfig = new TalonSRXConfiguration();
    public SparkMaxConfig releaseConfig = new SparkMaxConfig();

    public CTREConfigs() {
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.CANCODER_INVERT;

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = Constants.Swerve.ANGLE_MOTOR_INVERT;
        swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.Swerve.ANGLE_NEUTRAL_MODE;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.ANGLE_GEAR_RATIO;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.StatorCurrentLimitEnable = Constants.Swerve.ANGLE_ENABLE_STATOR_CURRENT_LIMIT;
        swerveAngleFXConfig.CurrentLimits.StatorCurrentLimit = Constants.Swerve.ANGLE_STATOR_CURRENT_LIMIT;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.ANGLE_ENABLE_CURRENT_LIMIT;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.ANGLE_CURRENT_LIMIT;
        // swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.ANGLE_CURRENT_THRESHOLD; //these variables got deleted, didnt find a replacement
        // swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.ANGLE_CURRENT_THRESHOLD_TIME;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.ANGLE_PID.kP;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.ANGLE_PID.kI;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.ANGLE_PID.kD;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = Constants.Swerve.DRIVE_MOTOR_INVERT;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.Swerve.DRIVE_NEUTRAL_MODE;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.DRIVE_GEAR_RATIO;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.StatorCurrentLimitEnable = Constants.Swerve.DRIVE_ENABLE_STATOR_CURRENT_LIMIT;
        swerveDriveFXConfig.CurrentLimits.StatorCurrentLimit = Constants.Swerve.DRIVE_STATOR_CURRENT_LIMIT;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.DRIVE_ENABLE_CURRENT_LIMIT;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.DRIVE_CURRENT_LIMIT;
        //swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.DRIVE_CURRENT_THRESHOLD; //these variables got deleted, didnt find a replacement
        //swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.DRIVE_CURRENT_THRESHOLD_TIME;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.DRIVE_PID.kP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.DRIVE_PID.kI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.DRIVE_PID.kD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.OPEN_LOOP_RAMP;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.OPEN_LOOP_RAMP;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.CLOSED_LOOP_RAMP;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.CLOSED_LOOP_RAMP;
    
        // //turret configs
        //     turretConfig.peakCurrentLimit = Constants.TurretConstants.CURRENT_LIMIT;
        //     turretConfig.continuousCurrentLimit = Constants.TurretConstants.CURRENT_LIMIT;
        //     //turretConfig.
            

        //     turretConfig.forwardSoftLimitEnable = Constants.TurretConstants.LimitEnable;
        //     turretConfig.forwardSoftLimitThreshold = Constants.TurretConstants.forwardSoftLimit;
        //     turretConfig.reverseSoftLimitEnable = Constants.TurretConstants.LimitEnable;
        //     turretConfig.reverseSoftLimitThreshold = Constants.TurretConstants.reverseSoftLimit;

        // turretConfig.inverted(Constants.TurretConstants.MotorInverted);
    
        // turretConfig.smartCurrentLimit(Constants.TurretConstants.CURRENT_LIMIT);
        
        // turretConfig.idleMode(Constants.TurretConstants.MotorMode);
        
        
        // turretConfig.voltageCompensation(0);
        // turretConfig.softLimit.forwardSoftLimit(Constants.TurretConstants.forwardSoftLimit);
        // turretConfig.softLimit.forwardSoftLimitEnabled(Constants.TurretConstants.LimitEnable);
        // turretConfig.softLimit.reverseSoftLimit(Constants.TurretConstants.reverseSoftLimit);
        // turretConfig.softLimit.reverseSoftLimitEnabled(Constants.TurretConstants.LimitEnable);
   

        //shooter configs
        // var s_currentlimits = shooterConfigs.CurrentLimits;
        // s_currentlimits.StatorCurrentLimit = Constants.ShootSubsystem.STATOR_CURRENT_LIMIT;
        // s_currentlimits.StatorCurrentLimitEnable = Constants.ShootSubsystem.ENABLE_STATOR_CURRENT_LIMIT;
        // s_currentlimits.SupplyCurrentLimit = Constants.ShootSubsystem.CURRENT_LIMIT;
        // s_currentlimits.SupplyCurrentLimitEnable = Constants.ShootSubsystem.ENABLE_CURRENT_LIMIT;

        // var s_motoroutput = shooterConfigs.MotorOutput;
        // s_motoroutput.NeutralMode = Constants.ShootSubsystem.ShootMotorMode;
        // s_motoroutput.Inverted = Constants.ShootSubsystem.ShootMotorInverted;

        // var s_slot0Configs = shooterConfigs.Slot0;
        // s_slot0Configs.kS = Constants.ShootSubsystem.kS; // Add 0.25 V output to overcome static friction
        // s_slot0Configs.kV = Constants.ShootSubsystem.kV; // A velocity target of 1 rps results in 0.12 V output
        // s_slot0Configs.kA = Constants.ShootSubsystem.kA; // An acceleration of 1 rps/s requires 0.01 V output
        // s_slot0Configs.kP = Constants.ShootSubsystem.kP; // An error of 1 rps results in 0.11 V output
        // s_slot0Configs.kI = Constants.ShootSubsystem.kI; // no output for integrated error
        // s_slot0Configs.kD = Constants.ShootSubsystem.kD; // no output for error derivative

        // set Motion Magic Velocity settings
        // var s_motionMagicConfigs = shooterConfigs.MotionMagic;
        // s_motionMagicConfigs.MotionMagicAcceleration = Constants.ShootSubsystem.MotionMagicAcceleration; // Target acceleration of 400 rps/s (0.25 seconds to max)
        // s_motionMagicConfigs.MotionMagicJerk = Constants.ShootSubsystem.MotionMagicJerk; // Target jerk of 4000 rps/s/s (0.1 seconds)


        //intake configs
        // var i_currentlimits = intakeConfigs.CurrentLimits;
        // i_currentlimits.StatorCurrentLimit = Constants.IntakeConstants.STATOR_CURRENT_LIMIT;
        // i_currentlimits.StatorCurrentLimitEnable = Constants.IntakeConstants.ENABLE_STATOR_CURRENT_LIMIT;
        // i_currentlimits.SupplyCurrentLimit = Constants.IntakeConstants.CURRENT_LIMIT;
        // i_currentlimits.SupplyCurrentLimitEnable = Constants.IntakeConstants.ENABLE_CURRENT_LIMIT;

        var i_motoroutput = intakeConfigs.MotorOutput;
        i_motoroutput.NeutralMode = NeutralModeValue.Brake;
        i_motoroutput.Inverted = InvertedValue.Clockwise_Positive;

        // //release configs
        // releaseConfig.inverted(Constants.ReleaseConstants.MotorInverted);
    
        // releaseConfig.smartCurrentLimit(Constants.ReleaseConstants.CURRENT_LIMIT);
    
        // releaseConfig.idleMode(Constants.ReleaseConstants.MotorMode);

    }
}