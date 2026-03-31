/*
 * Copyright (C) 2026 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If
 * not, see <https://www.gnu.org/licenses/>.
 */

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.LoggedDashboardChooser;
import frc.lib.util.AutoRoutine;
import frc.lib.util.CommandXboxControllerExtended;
import frc.lib.util.FieldUtil;
import frc.robot.Constants.PathConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.autos.*;
import frc.robot.subsystems.arm.ArmSuperstructure;
import frc.robot.subsystems.arm.ArmSuperstructureConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDsConstants;
import frc.robot.subsystems.vision.VisionConstants;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;

/**
 * Container class for the robot that holds all subsystems, controllers, and command bindings. This
 * class is responsible for:
 * <ul>
 * <li>Instantiating all subsystems</li>
 * <li>Configuring controller button bindings</li>
 * <li>Providing autonomous command selection</li>
 * <li>Setting up dashboard controls and telemetry</li>
 * </ul>
 */
public class RobotContainer {
    private final RobotState robotState = RobotState.getInstance();

    // Subsystems
    public final Drive drive;
    private final LEDs leds;
    private final Intake intake;
    private final ArmSuperstructure arm;

    // Controller
    private final CommandXboxControllerExtended controller =
        new CommandXboxControllerExtended(0).withDeadband(0.1);

    // Dashboard inputs
    private final LoggedDashboardChooser<AutoRoutine> autoChooser;
    public final Field2d autoPreviewField = new Field2d();

    private final Trigger isAutonomous = new Trigger(DriverStation::isAutonomous);

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */
    @SuppressWarnings("unchecked")
    public RobotContainer() {

        VisionConstants.create();
        drive = DriveConstants.get();
        leds = LEDsConstants.get();
        intake = IntakeConstants.get();
        arm = ArmSuperstructureConstants.get();


        LoggedDashboardChooser<Command> ldc = new LoggedDashboardChooser<>("ArmState");
        for (ArmSuperstructure.State s : ArmSuperstructure.State.values()) {
            ldc.addOption(s.toString(), arm.setArmState(s));
        }
        autoChooser = new LoggedDashboardChooser<>("Auto Choices");
        SmartDashboard.putData("Auto Preview", autoPreviewField);

        // Default - No Auto
        autoChooser.addDefaultOption("None", new NoneAuto());



        autoChooser.onChange(auto -> {
            autoPreviewField.getObject("path")
                .setPoses(auto.getAllPathPoses().stream()
                    .map(p -> FieldUtil.apply(p)).toArray(Pose2d[]::new));
        });

        autoChooser.addOption("Drive Wheel Radius Characterization",
            new WheelCharacterizationAuto(drive));

        autoChooser.addOption("Wheel Slip Characterization", new WheelSlipAuto(drive));

        SmartDashboard.putData("Intake Push", intake.push());
        SmartDashboard.putData("Intake Pull", intake.pull());



        // Configure the button bindings
        configureButtonBindings();
        initializeDashboard();
        configureLEDTriggers();
    }

    /**
     * Configures button bindings for the Xbox controller. Maps controller inputs to robot commands
     * for teleop control.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()));

        controller.a().whileTrue(
            intake.pull());
        controller.b().whileTrue(
            intake.push());

    }

    /**
     * Initializes SmartDashboard with test commands and controls for subsystems. Adds commands to
     * the dashboard for manual testing and debugging.
     */
    private void initializeDashboard() {
        SmartDashboard.putData("Face Target",
            DriveCommands.joystickDriveFacingTarget(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX()));
    }

    /** Creates and/or binds triggers to LED states */
    private void configureLEDTriggers() {
        isAutonomous
            .onTrue(leds.scheduleStateCommand(LEDs.State.RUNNING_AUTO))
            .onFalse(leds.unscheduleStateCommand(LEDs.State.RUNNING_AUTO));
    }

    /**
     * Gets the selected autonomous command from the dashboard chooser.
     *
     * @return the autonomous command to run
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    /**
     * Checks and displays the robot's starting pose accuracy relative to the selected autonomous
     * path. This function is called periodically by Robot.java when disabled.
     */
    public void checkStartPose() {

        /* Starting pose checker for auto */
        autoPreviewField.setRobotPose(robotState.getEstimatedPose());

        try {
            Pose2d startPose = autoPreviewField.getObject("path").getPoses().get(0);
            autoPreviewField.getObject("startPose").setPose(startPose);

            Distance distanceFromStartPose =
                Meters.of(robotState.getEstimatedPose().getTranslation()
                    .getDistance(startPose.getTranslation()));
            double degreesFromStartPose = Math.abs(robotState.getEstimatedPose().getRotation()
                .minus(startPose.getRotation())
                .getDegrees());

            double[] startPoseArray =
                {startPose.getX(), startPose.getY(), startPose.getRotation().getDegrees()};
            SmartDashboard.putNumberArray("Start Pose (x, y, degrees)", startPoseArray);

            SmartDashboard.putNumber("Auto Pose Check/Inches from Start",
                (int) Math.round(distanceFromStartPose.in(Inches) * 100.0) / 100.0);
            SmartDashboard.putBoolean(
                "Auto Pose Check/Robot Position Within Tolerance",
                distanceFromStartPose.in(Inches) < PathConstants.STARTING_POSE_DRIVE_TOLERANCE
                    .in(Inches));
            SmartDashboard.putNumber("Auto Pose Check/Degrees from Start",
                (int) Math.round(degreesFromStartPose * 100.0) / 100.0);
            SmartDashboard.putBoolean(
                "Auto Pose Check/Robot Rotation Within Tolerance",
                degreesFromStartPose < PathConstants.STARTING_POSE_ROT_TOLERANCE_DEGREES
                    .in(Degrees));

        } catch (Exception e) {
            SmartDashboard.putNumber("Auto Pose Check/Inches from Start", -1);
            SmartDashboard.putBoolean(
                "Auto Pose Check/Robot Position Within Tolerance",
                false);
            SmartDashboard.putNumber("Auto Pose Check/Degrees from Start", -1);
            SmartDashboard.putBoolean(
                "Auto Pose Check/Robot Rotation Within Tolerance",
                false);
        }
    }
}
