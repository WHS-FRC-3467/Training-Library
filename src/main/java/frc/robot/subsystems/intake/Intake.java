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
package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import static frc.robot.subsystems.intake.IntakeConstants.MAX_VELOCITY;

public class Intake extends SubsystemBase {

    private final FlywheelMechanism<?> io;

    Intake(FlywheelMechanism<?> io) {
        this.io = io;
    }

    private void runVelocity(AngularVelocity velocity) {
        io.runVelocity(velocity, IntakeConstants.MAX_ACCELERATION, PIDSlot.SLOT_0);
    }

    public Command pull() {
        return this.startEnd(
            () -> runVelocity(MAX_VELOCITY),
            () -> stop());
    }

    public Command push() {
        return this.startEnd(
            () -> runVelocity(RotationsPerSecond.of(-MAX_VELOCITY.in(RotationsPerSecond))),
            () -> stop());
    }

    private void stop() {
        io.runBrake();
    }

    @Override
    public void periodic() {
        io.periodic();

    }
}
