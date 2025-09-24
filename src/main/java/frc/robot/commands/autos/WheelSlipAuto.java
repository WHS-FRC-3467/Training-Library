// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.AutoCommand;
import frc.robot.subsystems.drive.Drive;

public class WheelSlipAuto extends AutoCommand {
    private final Drive drive;

    public WheelSlipAuto(Drive drive)
    {
        this.drive = drive;
        addCommands(Commands.sequence(
            Commands.run(() -> drive.runCharacterization(0.0)).withTimeout(2),
            rampUntilVelocity(drive, 0.2, RotationsPerSecond.of(1))));
    }

    @Override
    public List<Pose2d> getAllPathPoses()
    {
        return Collections.emptyList();
    }

    @Override
    public Pose2d getStartingPose()
    {
        return drive.getPose();
    }

    private static Command rampUntilVelocity(Drive drive, double rampRate,
        AngularVelocity speedLimit)
    {
        Timer timer = new Timer();

        return Commands.sequence(
            Commands.runOnce(() -> timer.restart()),
            Commands.deadline(
                Commands.waitUntil(() -> drive.getFFCharacterizationVelocity() > speedLimit
                    .in(RotationsPerSecond)),
                Commands.run(() -> {
                    double current = timer.get() * rampRate;
                    drive.runCharacterization(current);
                    Logger.recordOutput("Wheel Slip Current: ", current);
                })));
    }
}
