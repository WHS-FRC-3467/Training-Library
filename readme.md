# Flywheel Subsystem Tutorial

Welcome to the tutorial for creating the `Flywheel.java` file! This guide will help you understand how to create a Flywheel subsystem for your FRC robot. By the end, you'll have a working subsystem that can control a flywheel mechanism.

## Step 1: What is a Flywheel Subsystem?

A subsystem in FRC programming represents a physical part of the robot, such as a flywheel. The flywheel is a spinning mechanism used to store rotational energy, often for launching game pieces. 

In programming, subsystems are used to organize the robot's code. Each subsystem contains methods and commands that control its specific hardware. For example, the Flywheel subsystem will include methods to spin the flywheel at a certain speed, stop it, or retrieve its current state.


## Step 2: Setting Up the File

1. **Open Your Project**: Open your FRC robot project in your IDE. If you're using VS Code, make sure you have the WPILib extension installed.
2. **Navigate to the Subsystems Folder**: In the `src/main/java/frc/robot/subsystems` directory, locate or create a folder named `flywheel`. This folder will contain all files related to the Flywheel subsystem.
3. **Create the File**: Inside the `flywheel` folder, create a new Java file named `Flywheel.java`. This file will contain the code for the Flywheel subsystem.

## Step 3: Writing the Code

### 3.1 Importing Required Libraries

At the top of the file, you need to import libraries that provide the tools to build your subsystem. Here's what each library does:
- `edu.wpi.first.wpilibj2.command.SubsystemBase`: This is the base class for all subsystems. It provides the structure and functionality needed to integrate the subsystem into the robot's command-based framework.
- `frc.lib.mechanisms.flywheel.FlywheelMechanism`: This class represents the physical flywheel hardware and provides methods to control it.
- `frc.lib.util.LoggerHelper`: This utility helps log the current state of the subsystem, which is useful for debugging.
- Utility classes like `AngularVelocity` and `Current` are used to represent physical quantities in the code.

### 3.2 Defining the Flywheel Class

Define the `Flywheel` class and extend `SubsystemBase`. This tells the robot code that `Flywheel` is a subsystem:
```java
public class Flywheel extends SubsystemBase {
    // This is where we will define the flywheel mechanism and its behavior.
}
```
The `SubsystemBase` class provides methods like `periodic()` that are called automatically during the robot's operation.

### 3.3 Adding the Flywheel Mechanism

The `FlywheelMechanism` class represents the hardware for the flywheel. To use it, create a private field and initialize it in the constructor:
```java
private final FlywheelMechanism io;

public Flywheel(FlywheelMechanism io) {
    this.io = io;
}
```
The `FlywheelMechanism` object (`io`) will be used to send commands to the flywheel hardware.

### 3.4 Implementing the `periodic` Method

The `periodic` method is called repeatedly (about every 20ms) during the robot's operation. Use it to log the current command and update the flywheel mechanism:
```java
@Override
public void periodic() {
    LoggerHelper.recordCurrentCommand(FlywheelConstants.NAME, this);
    io.periodic();
}
```
This ensures that the subsystem's state is updated and logged regularly.

### 3.5 Adding Commands

Commands are actions that the subsystem can perform. Here are the commands you'll add:

1. **Shoot Command**: Spins the flywheel at maximum velocity. This is useful for launching game pieces.
   ```java
   public Command shoot() {
       return this.runOnce(() -> io.runVelocity(FlywheelConstants.MAX_VELOCITY,
           FlywheelConstants.MAX_ACCELERATION, PIDSlot.SLOT_0)).withName("Shoot");
   }
   ```
   - `runVelocity` sets the flywheel's speed and acceleration.
   - `PIDSlot.SLOT_0` specifies which PID controller settings to use.

2. **Stop Command**: Stops the flywheel. This is useful for conserving energy when the flywheel is not in use.
   ```java
   public Command stop() {
       return this.runOnce(() -> io.runCoast()).withName("Stop");
   }
   ```

### 3.6 Adding Utility Methods

Utility methods provide information about the flywheel's state or perform cleanup tasks:

2. **Get Velocity**: Returns the flywheel's current velocity.
   ```java
   public AngularVelocity getVelocity() {
       return io.getVelocity();
   }
   ```

## Step 4: Testing the Flywheel Subsystem

1. **Add the Subsystem to RobotContainer**: In your `RobotContainer.java` file, create an instance of the `Flywheel` subsystem.
2. **Create Commands**: Use buttons or triggers on your joystick/controller to call the `shoot` and `stop` commands.
3. **Deploy and Test**: Deploy the code to your robot and test the flywheel functionality. Make sure the flywheel spins at the correct speed and stops when commanded.

## Summary

You have successfully created the `Flywheel.java` file! This subsystem is now ready to control your flywheel mechanism. Experiment with different commands and configurations to optimize its performance.

Remember, programming is all about testing and improving. Don't be afraid to make changes and see how they affect your robot's behavior.

Happy coding!
