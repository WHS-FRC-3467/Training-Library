// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import java.util.function.Supplier;
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

    public Command shoot(Supplier<AngularVelocity> velocity)
    {
        return this.runOnce(() -> io.runVelocity(
            velocity.get(),
            FlywheelConstants.MAX_ACCELERATION,
            PIDSlot.SLOT_0))
            .withName("Shoot");
    }

    public Command stop()
    {
        return this.runOnce(() -> io.runCoast()).withName("Stop");
    }

    public AngularVelocity getVelocity()
    {
        return io.getVelocity();
    }

    public void close()
    {
        io.close();
    }
}
