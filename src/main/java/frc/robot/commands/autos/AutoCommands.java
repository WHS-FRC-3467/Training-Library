// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.FieldUtil;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;

/**
 * Class containing useful individual commands or small-group command sequences that can be strung
 * together into larger command units (AutoSegments). Command logic layer.
 */
public class AutoCommands {
    /**
     * Resets the robot's odometry to the starting pose of the specified path. Handles alliance
     * flipping if necessary. ONLY RUNS IN SIMULATION.
     *
     * @param drive the drive subsystem
     * @param path the PathPlanner path containing the starting pose
     * @return a command that resets the robot's pose to the path's starting position
     */
    public static Command resetSimOdom(Drive drive, PathPlannerPath path) {
        if (RobotBase.isSimulation()) {
            final RobotState robotState = RobotState.getInstance();
            return drive.runOnce(
                () -> {
                    Pose2d pose =
                        path.getStartingHolonomicPose().get();
                    if (FieldUtil.shouldFlip()) {
                        pose = FieldUtil.apply(pose);
                    }

                    robotState.resetPose(pose);
                });
        } else

        {
            return Commands.none();
        }
    }


}
