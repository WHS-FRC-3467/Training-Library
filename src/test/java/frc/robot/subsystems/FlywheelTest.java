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

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.parallel.ResourceAccessMode;
import org.junit.jupiter.api.parallel.ResourceLock;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.TestUtil;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelConstants;

@ResourceLock(value = "SimState", mode = ResourceAccessMode.READ_WRITE)
public class FlywheelTest {

    Flywheel flywheel;

    @BeforeEach
    void constructDevices()
    {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

        flywheel = new Flywheel(FlywheelConstants.getSim());

        /* enable the robot */
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        /* delay ~100ms so the devices can start up and enable */
        Timer.delay(0.100);
    }

    @Test
    public void stop()
    {
        TestUtil.runTest(flywheel.stop(), 1, flywheel);
        try {
            System.out
                .println("Flywheel velocity: " + flywheel.getVelocity().in(RotationsPerSecond));
            assertEquals(0.0, flywheel.getVelocity().in(RotationsPerSecond), 0.1);
        } catch (Exception e) {
            fail("Flywheel failed to get to velocity" + e.getMessage());
        }
    }

    @Test
    public void shoot()
    {
        TestUtil.runTest(flywheel.shoot(), 5, flywheel);
        try {
            System.out
                .println("Flywheel velocity: " + flywheel.getVelocity().in(RotationsPerSecond));
            assertEquals(FlywheelConstants.MAX_VELOCITY.in(RotationsPerSecond),
                flywheel.getVelocity().in(RotationsPerSecond), 0.1);
        } catch (Exception e) {
            fail("Flywheel failed to get to velocity" + e.getMessage());
        }
    }


}


