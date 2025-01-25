package frc.robot;

import java.util.List;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ClimbingKookDown;
import frc.robot.commands.ClimbingKookUp;
import frc.robot.commands.GyroResetCmd;
import frc.robot.commands.AutoIntakeCmd;
//import frc.robot.commands.IntakeCmd;
//import frc.robot.commands.IntakeStopCmd;
import frc.robot.commands.ShootCmd;
import frc.robot.commands.ShootStopCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ClimbingKook;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final ClimbingKook climbingkook = new ClimbingKook();    

    private final CommandXboxController controller = new CommandXboxController(0);
    //private final XboxController controller = new XboxController(0);
    //private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);

    /*
    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

        configureButtonBindings();
    }
 */
    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -controller.getLeftY(),
                () -> controller.getLeftX(),
                () -> controller.getRightX(),
                () -> !controller.leftTrigger(0.2).getAsBoolean(),
                () -> !controller.leftStick().getAsBoolean()));

        SmartDashboard.putString("Color Sensor: ", "1 Intake Instance Created");
        configureButtonBindings();
    }
    private void configureButtonBindings() {
        //() -> swerveSubsystem.zeroHeading()
        controller.back().onTrue(new GyroResetCmd(swerveSubsystem));
        //new JoystickButton(driverJoytick, 2).onTrue(() -> swerveSubsystem.zeroHeading());
        //EventLoop GyroReset = new EventLoop();
        //GyroReset.bind(swerveSubsystem.zeroHeading());
        //controller.back(GyroReset);
        controller.rightBumper().onTrue(new ShootCmd(shooterSubsystem));
        controller.rightBumper().onFalse(new ShootStopCmd(shooterSubsystem));
        //controller.leftBumper().onTrue(new InstantCommand(() -> shooterSubsystem.runIntake()));
        //controller.leftBumper().onFalse(new InstantCommand(() -> shooterSubsystem.stopIntake()));
        //controller.leftBumper().whileTrue(new IntakeCmd(shooterSubsystem));
        controller.leftBumper().whileTrue(new InstantCommand( () -> shooterSubsystem.runIntake()));
        controller.leftBumper().onFalse(new InstantCommand(() -> shooterSubsystem.stopIntake()));
        controller.a().onTrue(new ClimbingKookDown(climbingkook));
        controller.y().onTrue(new ClimbingKookUp(climbingkook));
        controller.a().onFalse(Commands.runOnce(() -> climbingkook.StopHammerTime()));
        controller.y().onFalse(Commands.runOnce(() -> climbingkook.StopHammerTime()));
        controller.rightTrigger(0.2).onTrue(Commands.runOnce(() -> shooterSubsystem.shootamp()));
        controller.rightTrigger(0.2).onFalse(Commands.runOnce(() -> shooterSubsystem.StopShooter()));
    }

    

    public Command getAutonomousCommand() {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(0.2, 0)),
                new Pose2d(1.524, 0, Rotation2d.fromDegrees(0)),
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(
                trajectory1,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 2. Generate trajectory
        Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(1.524, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(0.2, 0.05)),
                new Pose2d(-1, 0, Rotation2d.fromDegrees(0)),
                trajectoryConfig);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
                trajectory2,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory1.getInitialPose())),
                Commands.waitSeconds(0.5),
                new InstantCommand(() -> shooterSubsystem.StartShooter()),
                Commands.waitSeconds(0.5),
                new InstantCommand(() -> shooterSubsystem.StopShooter()),
                swerveControllerCommand1,
                new AutoIntakeCmd(shooterSubsystem),
                new InstantCommand(() -> shooterSubsystem.stopIntake()),
                swerveControllerCommand2,
                new InstantCommand(() -> shooterSubsystem.StartShooter()),
                Commands.waitSeconds(0.5),
                new InstantCommand(() -> shooterSubsystem.StopShooter()),
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }
}
