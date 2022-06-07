# EFLL-Balancing-Robot
Project of Two-wheel Self-balancing robot with fuzzy logic control using embedded fuzzy logic library (eFLL).

## MPU6050
Multifunction semsor.
```cpp
// Starting from register 0x3B -> ACCEL_XOUT_H
Wire.write(0x3B);
Wire.endTransmission(false);
Wire.requestFrom(address, 14, true);
acc_x = (Wire.read() << 8 | Wire.read()) * 2.0 / 32768.0;
acc_y = (Wire.read() << 8 | Wire.read()) * 2.0 / 32768.0;
acc_z = (Wire.read() << 8 | Wire.read()) * 2.0 / 32768.0;
temp_ = (Wire.read() << 8 | Wire.read());
gyro_x = (Wire.read() << 8 | Wire.read()) * 250.0 / 32768.0;
gyro_y = (Wire.read() << 8 | Wire.read()) * 250.0 / 32768.0;
gyro_z = (Wire.read() << 8 | Wire.read()) * 250.0 / 32768.0;
```

**Complementary Filter**

**Kalman Filter**

## Fuzzy Logic System