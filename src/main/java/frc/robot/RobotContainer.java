// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.commands.AutomatedClimb;
import frc.robot.commands.SetPositionCommand;
import frc.robot.commands.ShootCommand;

import frc.robot.subsystems.VisionSubsystem_generated;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.KrakenPositionSubsystem;
import frc.robot.subsystems.Shoot;

import edu.wpi.first.cscore.HttpCamera;


public class RobotContainer {

    // SPEED LIMITS
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    
    // SWERVE DRIVE
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.025)
            .withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    
   private final HttpCamera photonCam = new HttpCamera("Driver_Camera","http://10.68.75.11:5800/stream.mjpg");

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // CONTROLLERS
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // SUBSYSTEMS
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionSubsystem_generated visionSubsystem_generated = new VisionSubsystem_generated(drivetrain);

    private final Intake intake;
    private final KrakenPositionSubsystem krakenSubsystem;
    private final Shoot shoot;

    // CLIMB POSITION CONSTANTS
    // Contrl KrakenPositionSubsystem during auto climb
    // TUNE: Adjust based on actual climb mechanism travel
    private static final double HOME_POSITION = 0.0;
    private static final double POSITION_1 = 10.0;
    private static final double POSITION_2 = 4.0;
    private static final double POSITION_3 = 4.0;


    public RobotContainer() { 
      
      krakenSubsystem = new KrakenPositionSubsystem(17);
      intake = new Intake(14);
      //shoot = new Shoot(1);
      shoot = new Shoot(/* topMotorCanId= */ 16, /* bottomMotorCanId= */ 15); //FIX THIS IS FOR 2-MOTOR SHOOT; WE HAVE 1

      // CameraServer.startAutomaticCapture(photonCam);
      // CameraServer.addCamera(photonCam);
      
      //configureAutoBuilder(); // for pathplanner

      configureBindings();
        
    }

    // private void configureAutoBuilder() {
    //     try {
    //         RobotConfig config = RobotConfig.fromGUISettings();

    //         AutoBuilder.configure(
    //             drivetrain.getState().Pose, 
    //             drivetrain.seedFieldCentric(), 
    //             drivetrain.getState().Speeds, 
    //             drivetrain.setControl(
    //                 new SwerveRequest.ApplyRobotSpeeds().withSpeeds(chassisSpeeds);
    //             ), 
    //             new PPHolonomicDriveController(
    //                 new PIDConstants(5.0, 0.0, 0.0), 
    //                 new PIDConstants(5.0, 0.0, 0.0)
    //             ), 
    //             config, 
    //             false, 
    //             drivetrain
    //             );
    //         SmartDashboard.putString("AutoBuilder/Status", "Configured OK");
    //     } catch (Exception e){
    //         SmartDashboard.putString("AutoBuilder/Status", "ERROR: "+ e.getMessage() + "--Open PathPlanner app and configure Robot Config");
    //     }
    // }
    

    private void configureBindings() {
        
        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * -driverController.getLeftY() * -driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * -driverController.getLeftX() * -driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * -driverController.getRightX() * -driverController.getRightX() *  MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));


        operatorController.y().whileTrue( new frc.robot.commands.IntakeCommand(intake,3)  );
        operatorController.x().whileTrue( new frc.robot.commands.IntakeCommand(intake,-3)  );
          // A button - Run automated sequence (3 full cycles)
        //operatorController.a().onTrue( new AutomatedClimb( krakenSubsystem, POSITION_1, POSITION_2, POSITION_3, HOME_POSITION));
        operatorController.b().whileTrue(new ShootCommand(shoot));

        
        // Calculate drivetrain commands from Joystick values
        double forward = -driverController.getLeftY() * TunerConstants.kMaxSpeedMetersPerSecond;
        double strafe = -driverController.getLeftX() * TunerConstants.kMaxSpeedMetersPerSecond;
        double turn = -driverController.getRightX() * TunerConstants.kMaxAngularSpeed;


        //----------this is in the Visiion subsystem now----------------
        // // Read in relevant data from the Camera
        // boolean targetVisible = false;
        // double targetYaw = 0.0;
        // double  results = kCameraName.getAllUnreadResults();
        // if (!results.isEmpty()) {
        //     // Camera processed a new frame since last
        //     // Get the last one in the list.
        //     var result = results.get(results.size() - 1);
        //     if (result.hasTargets()) {
        //         // At least one AprilTag was seen by the camera
        //         for (var target : result.getTargets()) {
        //             if (target.getFiducialId() == 7) {
        //                 // Found Tag 7, record its information
        //                 targetYaw = target.getYaw();
        //                 targetVisible = true;
        //             }
        //         }
        //     }
        // }
        //
        //     // Auto-align when requested
        //     if (driverController.a() && targetVisible) {
        //         // Driver wants auto-alignment to tag 7
        //         // And, tag 7 is in sight, so we can turn toward it.
        //         // Override the driver's turn command with an automatic one that turns toward the tag.
        //         turn = -1.0 * targetYaw * VISION_TURN_kP * TunerConstants.Swerve.kMaxAngularSpeed;
        //     }
        //
        //     // Command drivetrain motors based on target speeds
        //     drivetrain.drive(forward, strafe, turn);
//
        //     // Put debug information to the dashboard
        //     SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
        //
        // }
        //
        //if(driverController.a().onTrue){
      //  }
      //  else{
      //      driverController.a().onTrue(new SetKrakenPosition(krakenSubsystem, 0));
      //  }
      //-----------------------------------------------------

        // put Driver and operator buttons here!!-------------

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
