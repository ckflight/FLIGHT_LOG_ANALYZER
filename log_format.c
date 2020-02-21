    //This is the logging format of my CK FlightController Firmware.

    // LOG 32 Bytes here at each loop

    // 1 Byte start indicator.
    flightLog.log_buffer_1[flightLog.buffer_index++] = flightLog.log_start_byte;

    // All axis of gyro for raw and filtered results (12 bytes)
    for(int axis = 0; axis < XYZ_AXIS; axis++){

      int16_t gyroRaw 	 = gyro.gyroADCRaw[axis];
      int16_t gyroFiltered = (int16_t)gyro.gyroADCf[axis];

      flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(gyroRaw >> 8);   	   // HighByte
      flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(gyroRaw & 0xFF); 	   // LowByte

      flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(gyroFiltered >> 8);   // HighByte
      flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(gyroFiltered & 0xFF); // LowByte

    }

    // Altitude hold adjustment / 10 is sent (1 Byte)
    uint8_t altHold_throttle = ((uint16_t)CK_ALTITUDE_GetThrottleAdjustment_AltitudeHold()) / 10;
    flightLog.log_buffer_1[flightLog.buffer_index++] = altHold_throttle;

    // RCData / 10 is sent (1 Byte each, 4 Bytes total)
    for(int axis = 0; axis <= RC_THROTTLE_CHANNEL; axis++){

      uint8_t rcData = CK_RCData[axis] / 10;

      flightLog.log_buffer_1[flightLog.buffer_index++] = rcData;
    }

    // Motor final result / 10 is sent (1 Byte each, 4 Bytes total)
    uint8_t motor_result = 0;
    for(int i = 1; i <= 4; i++){

      motor_result = CK_MIXER_GetMotorFinalResult(i) / 10;

      flightLog.log_buffer_1[flightLog.buffer_index++] = motor_result;
    }

    // Imu results are sent same except yaw divided to 2 (1 Byte each, 3 Byte total)
    uint8_t imuAngle = 0;
    for(int axis = 0; axis < XYZ_AXIS; axis++){

      if(axis != AXIS_YAW){

        imuAngle = imu.eulerAngles[axis];

        flightLog.log_buffer_1[flightLog.buffer_index++] = imuAngle;
      }
      else{

        imuAngle = imu.eulerAngles[axis] / 2;

        flightLog.log_buffer_1[flightLog.buffer_index++] = imuAngle;
      }
    }

    // Last loop cycle time (2 Bytes)
    uint16_t loopTime = (uint16_t)currentLoopTime;
    flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(loopTime >> 8);   // HighByte
    flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(loopTime & 0xFF); // LowByte


    // Flight flags (2 Bytes)
    uint16_t flags_encoded = 0;//flags.flags_encoded;
    flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(flags_encoded >> 8); 	// HighByte
    flightLog.log_buffer_1[flightLog.buffer_index++] = (uint8_t)(flags_encoded & 0xFF); // LowByte

    // 2 Bytes available
    flightLog.log_buffer_1[flightLog.buffer_index++] = 0;
    flightLog.log_buffer_1[flightLog.buffer_index++] = 0;

    // 1 Byte end indicator.
    flightLog.log_buffer_1[flightLog.buffer_index++] = flightLog.log_end_byte;

