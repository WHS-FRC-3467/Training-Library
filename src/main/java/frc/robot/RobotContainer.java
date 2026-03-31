/*
 * Copyright (C) 2025 Windham Windup
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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.SteppableCommandGroup;
import frc.lib.util.LoggedTuneableProfiledPID;
import frc.lib.util.CommandXboxControllerExtended;
import frc.robot.Constants.PathConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.OnTheFlyPathCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import java.util.ArrayList;
import java.util.Arrays;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
    private final RobotState robotState = RobotState.getInstance();

    // Subsystems
    public final Drive drive;

    // Controller
    private final CommandXboxControllerExtended controller = new CommandXboxControllerExtended(0);

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */
    public RobotContainer() {
        drive = DriveConstants.get();
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()));

        // Lock to 0° when A button is held
        controller
            .a()
            .whileTrue(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -controller.getLeftY(),
                    () -> -controller.getLeftX(),
                    () -> new Rotation2d()));

        // Switch to X pattern when X button is pressed
        controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro to 0° when B button is pressed
        controller
            .b()
            .onTrue(
                Commands.runOnce(
                    () -> robotState.resetPose(
                        new Pose2d(robotState.getEstimatedPose().getTranslation(),
                            new Rotation2d())))
                    .ignoringDisable(true));

        // Pathfind to Pose when the Y button is pressed
        controller.y().onTrue(
            DriveCommands.pathFindToPose(() -> robotState.getEstimatedPose(),
                new Pose2d(1, 4, Rotation2d.kZero),
                PathConstants.ON_THE_FLY_PATH_CONSTRAINTS, MetersPerSecond.of(0.0),
                PathConstants.PATHGENERATION_DRIVE_TOLERANCE));

        // On-the-fly path with waypoints while the Right Bumper is held
        controller.rightBumper().whileTrue(
            new OnTheFlyPathCommand(drive, () -> robotState.getEstimatedPose(),
                new ArrayList<>(Arrays.asList()), // List
                // of
                // waypoints
                new Pose2d(6, 6, Rotation2d.k180deg), PathConstants.ON_THE_FLY_PATH_CONSTRAINTS,
                MetersPerSecond.of(0.0), false, PathConstants.PATHGENERATION_DRIVE_TOLERANCE,
                PathConstants.PATHGENERATION_ROT_TOLERANCE));

        LoggedTuneableProfiledPID linearController =
            new LoggedTuneableProfiledPID("DriveToPose/LinearController", 3.0, 0, 0.1, 0, 3.0);

        // LoggedTuneableProfiledPID drivePID =
        // new LoggedTuneableProfiledPID("DrivePID", 60.0, 0, 0, 5.0, 0);
        // SmartDashboard.putData("SubmitDrivePID", drive.setDrivePID(drivePID.convert()));

        SmartDashboard.putData("DriveToPose Command",
            new DriveToPose(drive, () -> new Pose2d(5, 5, Rotation2d.fromDegrees(90)))
                .withTolerance(Inches.of(3), Degrees.of(5)));

        Command steppableCommand = new SteppableCommandGroup(
            controller.x(),
            controller.y(),
            Commands.runOnce(() -> System.out.println("Step 1")),
            Commands.runOnce(() -> System.out.println("Step 2")),
            Commands.runOnce(() -> System.out.println("Step 3")));

        SmartDashboard.putData("Steppable Command", steppableCommand);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return Commands.none();
        // autoChooser.get();
    }

}
