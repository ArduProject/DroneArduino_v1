///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PID
// https://arduproject.es/control-de-estabilidad-y-pid/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Parametros PID PITCH W (lazo velocidad)
float Kp_pitch_w = 1.9, Ki_pitch_w = 0.07, Kd_pitch_w = 12;
int salidaPI_pitch_w_max1 = 380;  // Limitar parte integral (anti-wind-up)
int salidaPI_pitch_w_min1 = -380; // Limitar parte integral (anti-wind-up)
int salidaPI_pitch_w_max2 = 380;  // Limitar salida del PID
int salidaPI_pitch_w_min2 = -380; // Limitar salida del PID

// Parametros PID ROLL W (lazo velocidad)
float Kp_roll_w = 1.9, Ki_roll_w = 0.07, Kd_roll_w = 12;
int salidaPI_roll_w_max1 = 380;  // Limitar parte integral (anti-wind-up)
int salidaPI_roll_w_min1 = -380; // Limitar parte integral (anti-wind-up)
int salidaPI_roll_w_max2 = 380;  // Limitar salida del PID
int salidaPI_roll_w_min2 = -380; // Limitar salida del PID

// Parametros PID YAW W (lazo velocidad)
float Kp_yaw_w = 1.5, Ki_yaw_w = 0.05, Kd_yaw_w = 0;
int salidaPI_yaw_w_max1 = 380;  // Limitar parte integral (anti-wind-up)
int salidaPI_yaw_w_min1 = -380; // Limitar parte integral (anti-wind-up)
int salidaPI_yaw_w_max2 = 380;  // Limitar salida del PID
int salidaPI_yaw_w_min2 = -380; // Limitar salida del PID

// Parametros PID PITCH ang (lazo inclinacion)
float Kp_pitch_ang = 2.2, Ki_pitch_ang = 0.06, Kd_pitch_ang = 15 ;
int salidaPI_pitch_ang_max1 = 130;  // Limitar parte integral (anti-wind-up)
int salidaPI_pitch_ang_min1 = -130; // Limitar parte integral (anti-wind-up)
int salidaPI_pitch_ang_max2 = 130;  // Limitar salida del PID
int salidaPI_pitch_ang_min2 = -130; // Limitar salida del PID

// Parametros PID ROLL ang (lazo inclinacion)
float Kp_roll_ang = 2.2, Ki_roll_ang =  0.06, Kd_roll_ang = 15;
int salidaPI_roll_ang_max1 = 130;  // Limitar parte integral (anti-wind-up)
int salidaPI_roll_ang_min1 = -130; // Limitar parte integral (anti-wind-up)
int salidaPI_roll_ang_max2 = 130;  // Limitar salida del PID
int salidaPI_roll_ang_min2 = -130; // Limitar salida del PID

// Parametros PID yaw ang (lazo inclinacion)
//float Kp_yaw_ang = 1.6, Ki_yaw_ang =  0.022, Kd_yaw_ang = 0 * 3;
//float  yaw_ang_giroscopio, yaw_ang_error_prop, ITerm_yaw_ang, yaw_ang_giroscopio_anterior, PID_yaw_ang, PID_yaw_ang_ant, Dyaw_ang;
//int salidaPI_yaw_ang_max1 = 100;  // Limitar parte integral (anti-wind-up)
//int salidaPI_yaw_ang_min1 = -100; // Limitar parte integral (anti-wind-up)
//int salidaPI_yaw_ang_max2 = 100;  // Limitar salida del PID
//int salidaPI_yaw_ang_min2 = -100; // Limitar salida del PID

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PID angle (inclinacion) --> Lazo exterior
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PID_ang() {
  //============================================================================= PITCH GYRO ang
  pitch_ang_error_prop = wPitchConsigna - angulo_pitch;                                           // error entre lectura y consigna
  ITerm_pitch_ang += (Ki_pitch_ang * pitch_ang_error_prop);                                       // Parte integral (sumatorio del error en el tiempo)
  ITerm_pitch_ang = constrain(ITerm_pitch_ang, salidaPI_pitch_ang_min1, salidaPI_pitch_ang_max1); // Limitar parte integral
  DPitch_ang = Kd_pitch_ang * (angulo_pitch - pitch_ang_giroscopio_anterior);                     // Parte Derivativa (diferencia entre el error actual y el anterior)

  PID_pitch_ang = Kp_pitch_ang * pitch_ang_error_prop + ITerm_pitch_ang - DPitch_ang;             //--> salida PID
  PID_pitch_ang = constrain(PID_pitch_ang, salidaPI_pitch_ang_min2, salidaPI_pitch_ang_max2);     // Limitar salida del PID
  pitch_ang_giroscopio_anterior = angulo_pitch;                                                   // Error enterior
  //============================================================================= ROLL GYRO ang
  roll_ang_error_prop = wRollConsigna - angulo_roll;
  ITerm_roll_ang += (Ki_roll_ang * roll_ang_error_prop);
  ITerm_roll_ang = constrain(ITerm_roll_ang, salidaPI_roll_ang_min1, salidaPI_roll_ang_max1);
  DRoll_ang = Kd_roll_ang * (angulo_roll - roll_ang_giroscopio_anterior);

  PID_roll_ang = Kp_roll_ang * roll_ang_error_prop + ITerm_roll_ang - DRoll_ang; // salida PID
  PID_roll_ang = constrain(PID_roll_ang, salidaPI_roll_ang_min2, salidaPI_roll_ang_max2);
  roll_ang_giroscopio_anterior =  angulo_roll;
  //  //=============================================================================== YAW GYRO ang
  //  yaw_ang_error_prop = wYawConsigna;
  //  ITerm_yaw_ang += (Ki_yaw_ang * yaw_ang_error_prop);
  //  ITerm_yaw_ang = constrain(ITerm_yaw_ang, salidaPI_yaw_ang_min1, salidaPI_yaw_ang_max1);
  //  Dyaw_ang = Kd_yaw_ang * (yaw_ang_error_prop - yaw_ang_giroscopio_anterior);
  //
  //  PID_yaw_ang = Kp_yaw_ang * yaw_ang_error_prop + ITerm_yaw_ang + Dyaw_ang; // salida PID
  //  PID_yaw_ang = constrain(PID_yaw_ang, salidaPI_yaw_ang_min2, salidaPI_yaw_ang_max2);
  //  yaw_ang_giroscopio_anterior =  yaw_ang_error_prop;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PID w (velocidad) --> Lazo interior
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PID_w() {
  // En funcion del modo de vuelo que hayamos seleccionado, las consignas de los PID serán diferentes
  // En modo Acro solo controlamos la velocidad de cada eje (un PID por eje). La consigna  del PID se da en º/s
  if (mAcro == 1) {
    PIDwInPitch = wPitchConsigna;
    PIDwInRoll = wRollConsigna;
  }
  // En modo Estable controlamos la velocidad y la inclinacion de cada eje (dos PID por eje). La consigna del primer PID (inclinacion) se da en º
  // La salida del PID de inclinacion será la consigna de velocidad para el PID de velocidad
  else {
    PIDwInPitch = PID_pitch_ang;
    PIDwInRoll = PID_roll_ang;
  }
  //============================================================================= PITCH GYRO w
  pitch_w_error_prop = PIDwInPitch - gyro_pitch_input;
  ITerm_pitch_w += (Ki_pitch_w * pitch_w_error_prop);
  ITerm_pitch_w = constrain(ITerm_pitch_w, salidaPI_pitch_w_min1, salidaPI_pitch_w_max1);
  DPitch_w = Kd_pitch_w * (gyro_pitch_input - pitch_w_giroscopio_anterior);

  PID_pitch_w = Kp_pitch_w * pitch_w_error_prop + ITerm_pitch_w - DPitch_w; // salida PID
  PID_pitch_w = constrain(PID_pitch_w, salidaPI_pitch_w_min2, salidaPI_pitch_w_max2);
  pitch_w_giroscopio_anterior = gyro_pitch_input;
  //============================================================================= ROLL GYRO w
  roll_w_error_prop = PIDwInRoll - gyro_roll_input;
  ITerm_roll_w += (Ki_roll_w * roll_w_error_prop);
  ITerm_roll_w = constrain(ITerm_roll_w, salidaPI_roll_w_min1, salidaPI_roll_w_max1);
  DRoll_w = Kd_roll_w * (gyro_roll_input - roll_w_giroscopio_anterior);

  PID_roll_w = Kp_roll_w * roll_w_error_prop + ITerm_roll_w - DRoll_w; // salida PID
  PID_roll_w = constrain(PID_roll_w, salidaPI_roll_w_min2, salidaPI_roll_w_max2);
  roll_w_giroscopio_anterior =  gyro_roll_input;
  //=============================================================================== YAW GYRO w
  yaw_w_error_prop = wYawConsigna - gyro_yaw_input;
  ITerm_yaw_w += (Ki_yaw_w * yaw_w_error_prop);
  ITerm_yaw_w = constrain(ITerm_yaw_w, salidaPI_yaw_w_min1, salidaPI_yaw_w_max1);
  DYaw_w = Kd_yaw_w * (gyro_yaw_input - yaw_w_giroscopio_anterior);

  PID_yaw_w = Kp_yaw_w * yaw_w_error_prop + ITerm_yaw_w - DYaw_w; // salida PID
  PID_yaw_w = constrain(PID_yaw_w, salidaPI_yaw_w_min2, salidaPI_yaw_w_max2);
  yaw_w_giroscopio_anterior =  gyro_yaw_input;
}
