package frc.robot;




import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
    private final JoystickButton fastMode = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton slowMode = new JoystickButton(driver, XboxController.Button.kA.value);
    public static double power = 1;
    public static boolean robotCentric = false;
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Command> teamChooser;
    

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

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

        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, 
        ()-> -driver.getRawAxis(1) * power, 
        ()-> -driver.getRawAxis(0) * power,
        ()-> -driver.getRawAxis(4) * power, 
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
        zeroGyro.onTrue(new ParallelCommandGroup(new InstantCommand(() -> s_Swerve.zeroHeading()), new InstantCommand(()->s_Swerve.gyro.reset())));
        slowMode.onTrue(new InstantCommand(() -> RobotContainer.power = .333));
        fastMode.onTrue(new InstantCommand(() -> RobotContainer.power = 1));  
        
        
    }
    
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup((new InstantCommand(() -> {
             s_Swerve.gyro.reset();
            // s_Swerve.zeroHeading();
        })), autoChooser.getSelected());
    }
}
