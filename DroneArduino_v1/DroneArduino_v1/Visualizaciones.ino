// Sisualizar variables por el canal serie
int loop_counter;
int LoopMax;
int z;

int VisuSelect = 3; // --> 0:mando, 1: giro, 2: acc, 3: ang, 4: esc, 5: mando RC
void Visualizaciones() {
  z++;
  if (visu == 1) {
    if (VisuSelect == 0) {
      LoopMax = 5;
      if (loop_counter == 0)Serial.print(wPitchConsigna);
      if (loop_counter == 1)Serial.print("\t");
      if (loop_counter == 2)Serial.print(wRollConsigna);
      if (loop_counter == 3)Serial.print("\t");
      if (loop_counter == 4)Serial.println(wYawConsigna);
      //        Serial.print("\t");
      //        Serial.println(PotenciaFilt);
    }

    if (VisuSelect == 1) {
      Serial.print((gx - gyro_X_cal) / 16.4);
      Serial.print("\t");
      Serial.print((gy - gyro_Y_cal) / 16.4);
      Serial.print("\t");
      Serial.println((gz - gyro_Z_cal) / 16.4);
    }

    if (VisuSelect == 2) {
      Serial.print(ax / 4096);
      Serial.print("\t");
      Serial.print(ay / 4096);
      Serial.print("\t");
      Serial.println(az / 4096);
    }

    if (VisuSelect == 3) {
      LoopMax = 3;
      if (loop_counter == 0)Serial.print(angulo_pitch);
      if (loop_counter == 1)Serial.print("\t");
      if (loop_counter == 2)Serial.println(angulo_roll);
    }

    if (VisuSelect == 4) {
      LoopMax = 7;
      if (loop_counter == 0)Serial.print(esc1);
      if (loop_counter == 1)Serial.print("\t");
      if (loop_counter == 2)Serial.print(esc2);
      if (loop_counter == 3)Serial.print("\t");
      if (loop_counter == 4)Serial.print(esc3);
      if (loop_counter == 5)Serial.print("\t");
      if (loop_counter == 6)Serial.println(esc4);
    }

    loop_counter ++;
    if (loop_counter == LoopMax)loop_counter = 0;
    if (VisuSelect == 4) {
      Ki_pitch_w = 0;
      Ki_roll_w = 0;
      Ki_yaw_w = 0;
      Ki_pitch_ang = 0;
      Ki_roll_ang = 0;
    }
  }

  // https://arduproject.es/como-leer-variables-de-arduino-en-matlab/
  if (z == 10 && matlab == 1) {

    if (loop_counter == 0)Serial.print(esc1);
    if (loop_counter == 1)Serial.print("\t");
    if (loop_counter == 2)Serial.print(esc2);
    if (loop_counter == 3)Serial.print("\t");
    if (loop_counter == 4)Serial.print(esc3);
    if (loop_counter == 5)Serial.print("\t");
    if (loop_counter == 6)Serial.println(esc4);

    loop_counter ++;
    if (loop_counter == 7)loop_counter = 0;
    z = 0;
  }
}
