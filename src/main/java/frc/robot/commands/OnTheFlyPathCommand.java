/* Copyright (C) 2025 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class OnTheFlyPathCommand extends Command {

    PathPlannerPath path;
    Supplier<Pose2d> currentPose;
    List<Pose2d> waypointPoses;
    Pose2d targetPose;
    PathConstraints constraints;
    double goalEndVelocity;
    boolean shouldFlipPath;
    boolean shouldMirrorPath;
    double tolerance;
    double rotToleranceDegrees;
    Command command;

    private Field2d pathGenerationTrajectory = new Field2d();

    /**
     * Automatically generates a PathPlanner path on-the-fly based on dynamic inputs.
     * @param drive The Drive subsystem to get the current pose from
     * @param currentPose A Supplier that provides the current pose of the robot.
     * @param waypointPoses A list of Pose2d waypoints to include in the path.
     * @param targetPose The final target pose for the path.
     * @param constraints The PathConstraints to apply to the path.
     * @param goalEndVelocity The final velocity in meters/sec.
     * @param shouldMirrorPath Whether to mirror the path from the left/right side of the field
     * @param tolerance The allowed tolerance, in meters, of the robot's position from the target pose.
     * Saves the path and command for retrieval to be executed by the CommandScheduler in Robot.java
     */
    public OnTheFlyPathCommand(Drive drive, Supplier<Pose2d> currentPose, List<Pose2d> waypointPoses, Pose2d targetPose, 
        PathConstraints constraints, double goalEndVelocity, boolean shouldMirrorPath, double tolerance, double rotToleranceDegrees) {
        addRequirements(drive);
        this.currentPose = currentPose;
        this.waypointPoses = waypointPoses;
        this.targetPose = targetPose;
        this.constraints = constraints;
        this.goalEndVelocity = goalEndVelocity;
        this.shouldMirrorPath = shouldMirrorPath;
        this.tolerance = tolerance;
        this.rotToleranceDegrees = rotToleranceDegrees;

        // Create a Field in the Dashboard to visualize automatically generated paths
        SmartDashboard.putData("Path Generation Preview", pathGenerationTrajectory);
    }

    @Override
    public void initialize() {
        if (shouldMirrorPath) {
            double FIELD_WIDTH = Units.inchesToMeters(317); // Should be in a FieldConstants.java file
            if (waypointPoses != null) {
                for (int i = 0; i < waypointPoses.size(); i++) {
                    waypointPoses.set(i, new Pose2d(waypointPoses.get(i).getX(), FIELD_WIDTH - waypointPoses.get(i).getY(), 
                        waypointPoses.get(i).getRotation()));
                }
            }
            targetPose = new Pose2d(targetPose.getX(), FIELD_WIDTH - targetPose.getY(), targetPose.getRotation());
        }

        List<Waypoint> waypoints = new ArrayList<>();
        if (waypointPoses == null) {
            waypoints = PathPlannerPath.waypointsFromPoses(
                currentPose.get(), targetPose);
        } else {
            // Construct a single list with all the poses because waypointsFromPoses() does not accept poses AND a list of poses
            List<Pose2d> completePoseList = new ArrayList<>();
            completePoseList.add(currentPose.get());
            completePoseList.addAll(waypointPoses);
            completePoseList.add(targetPose);
            
            waypoints = PathPlannerPath.waypointsFromPoses(
                completePoseList);
        }

        PathConstraints pathConstraints = null;
        if (constraints == null) {
            pathConstraints = PathConstraints.unlimitedConstraints(12.0); // Unlimited constraints, only limited by motor torque and nominal battery voltage
        } else {
            pathConstraints = constraints;
        }
                
        // Create the path using the waypoints created above
        path = new PathPlannerPath(
            waypoints,
            pathConstraints,
            new IdealStartingState(0, currentPose.get().getRotation()), // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState(goalEndVelocity, targetPose.getRotation()) // Goal end state. You can set a holonomic rotation here.
        );
        
        // Prevent the path from being flipped - if the robot is switching alliances, then input the correct end pose
        path.preventFlipping = true;

        command = AutoBuilder.followPath(path);
        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();

        pathGenerationTrajectory.setRobotPose(currentPose.get());

        // Displays robot poses from the onTheFlyPath on Field2d widget while the command is being executed
        pathGenerationTrajectory.getObject("PathGenerationTrajectory").setPoses(path.getPathPoses());
        Logger.recordOutput("OnTheFlyPathCommand/Poses", path.getPathPoses().toArray(Pose2d[]::new));
    }

    @Override
    public boolean isFinished() {
        // Is the magnitude of the difference between the current pose and the target pose (last pose of the path) less than the tolerance?
        // Check rotation as well
        return (currentPose.get().minus(path.getPathPoses().get(path.getPathPoses().size()-1)).getTranslation().getNorm() < tolerance)
            && (Math.abs(currentPose.get().minus(path.getPathPoses().get(path.getPathPoses().size()-1)).getRotation().getDegrees()) < rotToleranceDegrees);
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);

        // Clear the on-the-fly path poses from the Field2d widget
        pathGenerationTrajectory.getObject("PathGenerationTrajectory").setPoses(new Pose2d[]{});
        Logger.recordOutput("OnTheFlyPathCommand/Poses", new Pose2d[]{});
    }
}