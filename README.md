# kRPC / MPU 6050

Flight Hardware: Arduino Nano + MPU6050 + servo + knob

Raspberry Pi: read IMU data and actuator state from the Arduino, translate that into control commands, send those commands to KSP over TCP using kRPC, and stream KSP telemetry back for logging/visualization.

Simulator Target: KSP + kRPC on your PC
