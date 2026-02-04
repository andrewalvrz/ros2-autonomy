graph TD
    subgraph "High-Level (Linux / ROS 2)"
        Cam[USB Camera] -->|OpenCV| Tag[AprilTag Node]
        Tag -->|geometry_msgs/Twist| Bridge[Serial Bridge Node]
    end

    subgraph "Low-Level (ESP32 Firmware)"
        Bridge -->|UART: V,vx,vy,wz| MCU[ESP32 MCU]
        MCU -->|PWM| Drv[Motor Drivers]
        Enc[Encoders] -->|Interrupts| MCU
        ToF[VL53L0X] -->|I2C| MCU
        IMU[MPU6050] -->|I2C| MCU
    end

    subgraph "Physical Actuation"
        Drv -->|Voltage| Motors[Mecanum Wheels]
    end

    MCU -.->|Odometry Data| Bridge
