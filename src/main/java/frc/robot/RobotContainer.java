package frc.robot;




import com.fasterxml.jackson.databind.node.IntNode;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final XboxController driver = new XboxController(0);
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    // private final JoystickButton fastMode = new JoystickButton(driver, XboxController.Button.kB.value);
    // private final JoystickButton slowMode = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton intake = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton shoot = new JoystickButton(driver, XboxController.Button.kB.value);
    public static double power = .5;
    public static boolean robotCentric = false;
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Command> teamChooser;
    
    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private final HoodSubsystem hoodSubsystem = new HoodSubsystem();
    private final IntakeArmSubsystem intakeArmSubsystem = new IntakeArmSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    private final DigitalInput ringLimitSwitch = new DigitalInput(Constants.ringLimitSwitchID);
    
    Command ArmOut() {
        return new IntakeArmPIDCommand(intakeArmSubsystem, Constants.IntakeArmConstants.OutAngle);
    }

    Command ArmIn() {
        return new IntakeArmPIDCommand(intakeArmSubsystem, -5);
    }

    Command Intake() {
        return new IntakeCommand(intakeSubsystem, () -> ringLimitSwitch.get());
    }

    Command IntakeOut() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> intakeSubsystem.setSpeed(-Constants.intakeConstants.speed)),
            new WaitCommand(1),
            new InstantCommand(() -> intakeSubsystem.setSpeed(0))
        );
    }

    Command IntakeAndReady() {
        return new SequentialCommandGroup(
            ArmOut(),
            Intake(),
            ArmIn()
        );
    }

    Command Shoot() {
        return new ParallelRaceGroup(shooterSubsystem.spin(),
            new SequentialCommandGroup(
                new WaitCommand(1.5), 
                IntakeOut()
        ));
    }

    Command IntakeAndShoot() {
        return new SequentialCommandGroup(
            IntakeAndReady(),
            Shoot()
        );
    }

    Command HoodOut() {
        return new HoodPIDCommand(hoodSubsystem, Constants.HoodConstants.OutAngle);
    }

    Command HoodIn(){
        return new HoodPIDCommand(hoodSubsystem, 0);
    }


    public RobotContainer() {
        
        Field2d field = new Field2d();
        SmartDashboard.putData("Field", field);
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            field.setRobotPose(pose);
        });
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            field.getObject("target pose").setPose(pose);
        });
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            field.getObject("path").setPoses(poses);
        });

        swerve.setDefaultCommand(new TeleopSwerve(swerve, 
        ()-> -driver.getRawAxis(1) * power, 
        ()-> -driver.getRawAxis(0) * power,
        ()-> driver.getRawAxis(4) * power, 
        ()->robotCentric));

       
        //r_ReleaseSubsystem.setDefaultCommand(r_ReleaseSubsystem.run(()-> codriver.getRawAxis(3) * 0.3));
        
        //s_ShooterSubsystem.setDefaultCommand(s_ShooterSubsystem.runCmd(()-> codriver.getRawAxis(2) * 1));

        configureButtonBindings();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        teamChooser = new SendableChooser<>();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData("Team Chooser", teamChooser);
        SmartDashboard.putNumber("Input Distance", 0);
    }

    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new ParallelCommandGroup(new InstantCommand(() -> swerve.zeroHeading()), new InstantCommand(()->swerve.gyro.reset())));
        // slowMode.onTrue(new InstantCommand(() -> RobotContainer.power = .333));
        // fastMode.onTrue(new InstantCommand(() -> RobotContainer.power = 1));  
        // in.whileTrue(ArmIn());
        // out.whileTrue(ArmOut());
        intake.onTrue(IntakeAndReady());
        shoot.onTrue(Shoot());
        // new Trigger(()-> ringLimitSwitch.get()).onChange(new InstantCommand(()->{SmartDashboard.putBoolean("ring", ringLimitSwitch.get());}))
        // m_driverController. (new InstantCommand(() -> ArmOut())
        
        
    }
    
    public Command getAutonomousCommand() {
        // return new SequentialCommandGroup((new InstantCommand(() -> {
        //      swerve.gyro.reset();
        //     // s_Swerve.zeroHeading();
        // })), autoChooser.getSelected());
        return new SequentialCommandGroup(IntakeAndShoot(), new InstantCommand(() -> CommandScheduler.getInstance().schedule(getAutonomousCommand())));
    }
}
