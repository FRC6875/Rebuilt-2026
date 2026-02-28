// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.KrakenPositionSubsystem;
import frc.robot.commands.AutomatedClimb;
import frc.robot.commands.SetPositionCommand;

import edu.wpi.first.cscore.HttpCamera;

public class RobotContainer {
    
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.025).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    
   private final HttpCamera photonCam = new HttpCamera("Driver_Camera","http://10.68.75.11:5800/stream.mjpg");

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
     
     private final Intake Intake;
     private final KrakenPositionSubsystem krakenSubsystem;


      // Constants for preset positions
        private static final double HOME_POSITION = 0.0;
        private static final double POSITION_1 = 10.0;
        private static final double POSITION_2 = 4.0;
        private static final double POSITION_3 = 4.0;


    public RobotContainer() { 
      // CameraServer.startAutomaticCapture(photonCam);
      // CameraServer.addCamera(photonCam);
     krakenSubsystem = new KrakenPositionSubsystem(16);
     Intake = new Intake(15);

        configureBindings();
        
    }
    

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


        operatorController.y().whileTrue( new frc.robot.commands.IntakeCommand(Intake,0.5)  );

          // A button - Run automated sequence (3 full cycles)
        operatorController.a().onTrue( new AutomatedClimb( krakenSubsystem, POSITION_1, POSITION_2, POSITION_3, HOME_POSITION));
        


        //if(driverController.a().onTrue){
      //  }
      //  else{
      //      driverController.a().onTrue(new SetKrakenPosition(krakenSubsystem, 0));
      //  }

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
