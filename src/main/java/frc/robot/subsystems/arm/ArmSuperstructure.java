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
package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.rotary.RotaryMechanism;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class ArmSuperstructure extends SubsystemBase {
    private final RotaryMechanism<?, ?> leader;
    private final RotaryMechanism<?, ?> follower;

    @Getter
    @RequiredArgsConstructor
    @SuppressWarnings("ImmutableEnumChecker")
    public enum State {
        STOWED(Degrees.zero()),
        INTAKE(Degrees.of(1.0)),
        SUBWOOFER(Degrees.of(1.0)),
        AMP(Degrees.of(93.0)),
        PODIUM(Degrees.of(23.0)),
        WING(Degrees.of(30.0)),
        CLIMB(Degrees.of(88.0)),
        TRAP(Degrees.of(-3.0)),
        FEED(Degrees.of(10.0)),
        HARMONY(Degrees.of(122.0));
       private final Angle angle;
       

        
    }
    
    private State armState = State.STOWED;

    ArmSuperstructure(RotaryMechanism<?, ?> leader, RotaryMechanism<?, ?> follower) {
        this.leader = leader;
        this.follower = follower;
    }

    public Command runPosition(RotaryMechanism<?, ?> mechanism, State armState) {
        return this.runOnce(() -> mechanism.runPosition(armState.getAngle(), PIDSlot.SLOT_0));
    }

    public Command setArmState(State armState) {
        return this.runOnce(
            
         () -> Commands.sequence(
            runPosition(leader, armState),
            runPosition(follower, armState)
          )
           
        );
    }
}
