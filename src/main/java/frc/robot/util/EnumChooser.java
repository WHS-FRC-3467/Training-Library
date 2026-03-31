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
package frc.robot.util;

import java.lang.reflect.ParameterizedType;
import java.util.function.Function;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.LoggedDashboardChooser;

public class EnumChooser<E extends Enum<E>> {
    LoggedDashboardChooser<Command> enumChooser;

    @SuppressWarnings("unchecked")
    private E[] getEnumConstants() {
        // Reflection Hackery
        return ((Class<E>) ((ParameterizedType) getClass().getGenericSuperclass())
            .getActualTypeArguments()[0]).getEnumConstants();
    }

    public EnumChooser(String key, Function<E, Command> commandSupplier) {
        enumChooser = new LoggedDashboardChooser<>(key);
        for (E variant : getEnumConstants()) {
            enumChooser.addOption(key + "/" + variant.name(), commandSupplier.apply(variant));
        }
    }

}
