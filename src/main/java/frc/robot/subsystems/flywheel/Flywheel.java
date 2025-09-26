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

package frc.robot.subsystems.flywheel;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.util.LoggerHelper;

public class Flywheel extends SubsystemBase {
    private final FlywheelMechanism io;

    public Flywheel(FlywheelMechanism io)
    {
        this.io = io;
    }

    @Override
    public void periodic()
    {
        LoggerHelper.recordCurrentCommand(FlywheelConstants.NAME, this);
        io.periodic();
    }

    public Command shoot()
    {
        return runOnce(() -> io.runVelocity(FlywheelConstants.MAX_VELOCITY,
            FlywheelConstants.MAX_ACCELERATION, PIDSlot.SLOT_0)).withName("Shoot");
    }

    public Command stop()
    {
        return runOnce(() -> io.runCoast()).withName("Stop");
    }

    public AngularVelocity getVelocity()
    {
        return io.getVelocity();
    }
}
