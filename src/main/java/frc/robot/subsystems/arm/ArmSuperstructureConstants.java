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
import static edu.wpi.first.units.Units.Foot;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import java.util.Optional;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Velocity;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorIOSim;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.io.motor.MotorIOTalonFX;
import frc.lib.io.motor.MotorIOTalonFX.TalonFXFollower;
import frc.lib.io.motor.MotorIOTalonFXSim;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.mechanisms.flywheel.FlywheelMechanismReal;
import frc.lib.mechanisms.flywheel.FlywheelMechanismSim;
import frc.lib.mechanisms.rotary.RotaryMechanism;
import frc.lib.mechanisms.rotary.RotaryMechanism.RotaryAxis;
import frc.lib.mechanisms.rotary.RotaryMechanism.RotaryMechCharacteristics;
import frc.lib.mechanisms.rotary.RotaryMechanismReal;
import frc.lib.mechanisms.rotary.RotaryMechanismSim;
import frc.robot.Constants;
import frc.robot.Ports;

public class ArmSuperstructureConstants {
    public static final String NAME = "Arm";

    public static final Angle TOLERANCE = Degrees.of(1.0);

    public static final AngularVelocity CRUISE_VELOCITY =
        RadiansPerSecond.of(10);
    public static final AngularAcceleration ACCELERATION =
        RadiansPerSecondPerSecond.of(100);
    public static final Velocity<AngularAccelerationUnit> JERK =
        RadiansPerSecondPerSecond.per(Second).of(0);

    private static final double ROTOR_TO_SENSOR = (50.0 / 1.0);
    private static final double SENSOR_TO_MECHANISM = 1.0;

    public static final Angle MIN_ANGLE = Degrees.of(-90.0);
    public static final Angle MAX_ANGLE = Degrees.of(90.0);
    public static final Angle STARTING_ANGLE = Radians.zero();
    public static final Distance ARM_LENGTH = Foot.one();

    public static final RotaryMechCharacteristics CONSTANTS =
        new RotaryMechCharacteristics(
            ARM_LENGTH,
            MIN_ANGLE,
            MAX_ANGLE,
            STARTING_ANGLE,
            RotaryAxis.PITCH);

    public static final DCMotor DCMOTOR = DCMotor.getKrakenX60(1);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.25);

    // Positional PID
    private static final Slot0Configs SLOT_0_CONFIG = new Slot0Configs()
        .withKP(10.0)
        .withKI(2.0)
        .withKD(8)
        .withKS(0.07)
        .withKV(0.1);

    public static TalonFXConfiguration getFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = false;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

        config.CurrentLimits.StatorCurrentLimitEnable = false;
        config.CurrentLimits.StatorCurrentLimit = 120.0;

        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGLE.in(Rotations);

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ANGLE.in(Rotations);

        config.Feedback.RotorToSensorRatio = ROTOR_TO_SENSOR;
        config.Feedback.SensorToMechanismRatio = SENSOR_TO_MECHANISM;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        config.Slot0 = SLOT_0_CONFIG;
        config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY.in(RotationsPerSecond);
        config.MotionMagic.MotionMagicAcceleration = ACCELERATION.in(RotationsPerSecondPerSecond);
        config.MotionMagic.MotionMagicJerk = JERK.in(RotationsPerSecondPerSecond.per(Second));

        return config;
    }

    public static ArmSuperstructure get() {
       ArmSuperstructure<?> mechanism;
        switch (Constants.currentMode) {
            case REAL:
                mechanism =
                         new ArmSuperstructure( new RotaryMechanismReal(
                                "Left " + NAME,
                                new MotorIOTalonFX(
                                        "Left " + NAME,
                                        getFXConfig(),
                                        Ports.arm,
                                        new TalonFXFollower(Ports.armf, false)), null, java.util.Optional.empty(), null));
                break;
            case SIM:
                mechanism =
                        new ArmSuperstructure( new RotaryMechanismSim(
                                "Left " + NAME,
                                new MotorIOSim(
                                        "Left " + NAME,
                                        getFXConfig(),
                                        Ports.arm,
                                        new TalonFXFollower(Ports.armf, false)), null, java.util.Optional.empty(), null));
                break;
            case REPLAY:
                mechanism = new FlywheelMechanism<>("Left " + NAME, new MotorIO() {}) {};
                break;
            default:
                throw new IllegalStateException("Unrecognized Robot Mode");
        }
        mechanism.enableTunablePID(PIDSlot.SLOT_0, SLOT0_PID);
        return mechanism;
    }
}
