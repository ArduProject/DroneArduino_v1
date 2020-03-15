///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MPU6050
// https://arduproject.es/mpu6050-y-su-programacion/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool set_gyro_angles, accCalibOK = false;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Subrutina inicializacializacion
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void init_MPU6050() {
  Wire.beginTransmission(gyro_address);
  Wire.write(0x6B);                                                          //Registro 6B hex)
  Wire.write(0x00);                                                          //00000000 para activar giroscopio
  Wire.endTransmission();
  Wire.beginTransmission(gyro_address);
  Wire.write(0x1B);                                                          //Register 1B hex
  Wire.write(0x08);                                                          //Girscopio a 500dps (full scale)
  Wire.endTransmission();
  Wire.beginTransmission(gyro_address);
  Wire.write(0x1C);                                                          //Register (1A hex)
  Wire.write(0x10);                                                          //Acelerometro a  +/- 8g (full scale range)
  Wire.endTransmission();

  Wire.beginTransmission(gyro_address);    // Test MPU6050
  Wire.write(0x1B);
  Wire.endTransmission();
  Wire.requestFrom(gyro_address, 1);
  while (Wire.available() < 1);
  if (Wire.read() != 0x08) {
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("MPU error");  // Si hay un error, avisamos y enclavamos el programa
    while (1)delay(10);
  }
  // Activar y configurar filtro pasa bajos LPF que incorpora el sensor
  Wire.beginTransmission(gyro_address);
  Wire.write(0x1A);
  Wire.write(0x04);     // Filtro pasa bajos -> 256Hz(0ms):0x00 <-> 188Hz(2ms):0x01 <-> 98Hz(3ms):0x02 <-> 42Hz(4.9ms):0x03 <-> 20Hz(8.5ms):0x04 <-> 10Hz(13.8ms):0x05 <-> 5Hz(19ms):0x06
  Wire.endTransmission();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Calibracion MPU6050
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Calib_MPU6050() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calibrando Giro ");
  if (AvisoVbatLow == true) {
    lcd.setCursor(0, 1);
    lcd.print("LOW BAT!!");
  }
  for (cal_int = 0; cal_int < 3000 ; cal_int ++) {  // Calibrar giroscopio tomando 3000 muestras
    MPU_6050();
    gyro_X_cal += gx;
    gyro_Y_cal += gy;
    gyro_Z_cal += gz;
    delayMicroseconds(1000);
  }
  gyro_X_cal = gyro_X_cal / 3000; // Calculamos el valor medio de las 3000 muestras
  gyro_Y_cal = gyro_Y_cal / 3000;
  gyro_Z_cal = gyro_Z_cal / 3000;

  lcd.setCursor(0, 0);
  lcd.print("Calibrando acc ");
  if (mEstable == 1) {
    for (cal_int = 0; cal_int < 3000 ; cal_int ++) { // Calibrar acelerometro tomando 3000 muestras
      MPU_6050();
      angle_pitch_acc_cal += ax;
      angle_roll_acc_cal += ay;
      angle_yaw_acc_cal += az;
    }
    angle_pitch_acc_cal = angle_pitch_acc_cal / 3000;  // Calculamos el valor medio de las 3000 muestras
    angle_roll_acc_cal = angle_roll_acc_cal / 3000;
    angle_yaw_acc_cal = angle_yaw_acc_cal / 3000;
    accCalibOK = true;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Lectura MPU6050
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MPU_6050() {
  emerg = 0;
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);

  Wire.beginTransmission(gyro_address);     //Empezar comunicacion con MPU6050
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(gyro_address, 14);
  while (Wire.available() < 14) {
    //    emerg++;
    //    //    Serial.println(emerg);
    //    if (emerg > 30) {
    //      while (1);
    //    }
  }
  tiempo_ejecucion = (micros() - loop_timer1) / 1000;
  loop_timer1 = micros();
  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  temperature = Wire.read() << 8 | Wire.read();
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();

  if (accCalibOK == true) { // Restar valores de calibracion
    ax -= angle_pitch_acc_cal;
    ay -= angle_roll_acc_cal;
    az -= angle_yaw_acc_cal;
    az = az + 4096;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Calculo de angulo
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ProcesMPU() {
  // ================================================================================================== Calculo w
  pitchGyro = (gx - gyro_X_cal) / 65.5; // 65.5: si leo 65.5 en raw, significa que gira a 1º/s
  rollGyro = (gy - gyro_Y_cal) / 65.5;
  yawGyro = (gz - gyro_Z_cal) / 65.5;

  gyro_roll_input = rollGyro;
  gyro_pitch_input = pitchGyro;
  gyro_yaw_input = yawGyro;
  //  gyro_roll_input = (gyro_roll_input * 0.5) + (rollGyro * 0.5);
  //  gyro_pitch_input = (gyro_pitch_input * 0.5) + (pitchGyro * 0.5);
  //  gyro_yaw_input = (gyro_yaw_input * 0.5) + (yawGyro * 0.5);

  // ================================================================================================== Calculo ang
  angulo_pitch += pitchGyro * tiempo_ejecucion / 1000 ;
  angulo_roll += rollGyro * tiempo_ejecucion / 1000 ;
  angulo_pitch += angulo_roll * sin((gz - gyro_Z_cal) * tiempo_ejecucion  * 0.000000266);      // tiempo_ejecucion/1000 /65.5 * PI/180
  angulo_roll -= angulo_pitch * sin((gz - gyro_Z_cal) * tiempo_ejecucion  * 0.000000266);

  acc_total_vector = sqrt(pow(ay, 2) + pow(ax, 2) + pow(az, 2));
  angle_pitch_acc = asin((float)ay / acc_total_vector) * 57.2958;     // 57.2958 = Conversion de radianes a grados 180/PI
  angle_roll_acc = asin((float)ax / acc_total_vector) * -57.2958;

  if (set_gyro_angles) {
    angulo_pitch = angulo_pitch * 0.995 + angle_pitch_acc * 0.005;  // Angulo Pitch de inclinacion
    angulo_roll = angulo_roll * 0.995 + angle_roll_acc * 0.005;     // Angulo Roll de inclinacion
  }
  else {
    angulo_pitch = angle_pitch_acc;
    angulo_roll = angle_roll_acc;
    set_gyro_angles = true;
  }
}
