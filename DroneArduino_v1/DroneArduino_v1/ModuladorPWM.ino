bool aM1, aM2, aM3, aM4 = false;
float accion_m1, accion_m2, accion_m3, accion_m4;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Modulador --> Calcular consignas para los motores
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Modulador () {
  // Si el throttle es menos a 1300us, el control de estabilidad se desactiva. La parte integral
  // de los controladores PID se fuerza a 0.
  if (pulsoPotencia <= 1300) {
    ITerm_pitch_w = 0;
    ITerm_roll_w = 0;
    ITerm_yaw_w = 0;
    ITerm_pitch_ang = 0;
    ITerm_roll_ang = 0;
    esc1 = PotenciaFilt;
    esc2 = PotenciaFilt;
    esc3 = PotenciaFilt;
    esc4 = PotenciaFilt;
    if (esc1 < 1000) esc1 = 950; // Si lo motores giran con el Stick de throttle al mínimo, recudir este valor
    if (esc2 < 1000) esc2 = 950;
    if (esc3 < 1000) esc3 = 950;
    if (esc4 < 1000) esc4 = 950;
  }

  // Si el throttle es mayor a 1300us, el control de estabilidad se activa.
  if (pulsoPotencia > 1300) {
    if (pulsoPotencia > 1800)pulsoPotencia = 1800; // Limitar throttle a 1800 para dejar manrgen a los PID
    esc1 = PotenciaFilt + PID_pitch_w - PID_roll_w - PID_yaw_w; // Motor 1
    esc2 = PotenciaFilt + PID_pitch_w + PID_roll_w + PID_yaw_w; // Motor 2
    esc3 = PotenciaFilt - PID_pitch_w + PID_roll_w - PID_yaw_w; // Motor 3
    esc4 = PotenciaFilt - PID_pitch_w - PID_roll_w + PID_yaw_w; // Motor 4
    //    esc1 = PotenciaFilt; // Solo para testeos
    //    esc2 = PotenciaFilt;
    //    esc3 = PotenciaFilt;
    //    esc4 = PotenciaFilt;

    if (esc1 < 1100) esc1 = 1100; // Evitamos que alguno de los motores de detenga completamente en pleno vuelo
    if (esc2 < 1100) esc2 = 1100;
    if (esc3 < 1100) esc3 = 1100;
    if (esc4 < 1100) esc4 = 1100;
    if (esc1 > 2000) esc1 = 2000; // Evitamos mandar consignas mayores a 2000us a los motores
    if (esc2 > 2000) esc2 = 2000;
    if (esc3 > 2000) esc3 = 2000;
    if (esc4 > 2000) esc4 = 2000;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Salidas PWM --> Motores
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PWM() {
  digitalWrite(3, HIGH); //Motor 1 HIGH
  digitalWrite(4, HIGH); //Motor 2 HIGH
  digitalWrite(5, HIGH); //Motor 3 HIGH
  digitalWrite(6, HIGH); //Motor 4 HIGH

  aM1 = true;
  aM2 = true;
  aM3 = true;
  aM4 = true;

  // ===================================================== 1ms max!!!!!!!
  tiempo_1 = micros();
  accion_m1 = esc1 + loop_timer;
  accion_m2 = esc2 + loop_timer;
  accion_m3 = esc3 + loop_timer;
  accion_m4 = esc4 + loop_timer;

  lectura_mandoRC(); // Leer mando RC
  ONled();           // LED parpadeo
  VbatCheck();       // Leer Vbat

  // Si la duracion entre tiempo_1 y tiempo_2 ha sido mayor de 900us, encender led de aviso.
  // Nunca hay que sobrepasar 1ms de tiempo en estado HIGH.
  tiempo_2 = micros();
  tiempo_ON = tiempo_2 - tiempo_1;
  if (tiempo_ON > 900) digitalWrite(13, HIGH);   // Tiempo excedido
  // ===================================================== 1ms max!!!!!!!

  while (aM1 || aM2 || aM3 || aM4 == true) {
    esc_loop_timer = micros();
    if (accion_m1 <= esc_loop_timer) { // Motor 1 LOW
      aM1 = false;
      digitalWrite(3, LOW);
    }
    esc_loop_timer = micros();
    if (accion_m2 <= esc_loop_timer) { // Motor 2 LOW
      aM2 = false;
      digitalWrite(4, LOW);
    }
    esc_loop_timer = micros();
    if (accion_m3 <= esc_loop_timer) { // Motor 3 LOW
      aM3 = false;
      digitalWrite(5, LOW);
    }
    esc_loop_timer = micros();
    if (accion_m4 <= esc_loop_timer) { // Motor 4 LOW
      aM4 = false;
      digitalWrite(6, LOW);
    }
  }
}
