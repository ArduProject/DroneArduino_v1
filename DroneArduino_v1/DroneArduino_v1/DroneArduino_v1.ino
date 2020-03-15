/////////////////////////////////
//        <ARDUPROJECT>        //
//            v1.01            //
//   https://arduproject.es/   //
//  dronedesdecero@gmail.com   //
//    Actualizado 5/12/2018    //
/////////////////////////////////

// ==========================================
int matlab = 0;
int visu = 0;   // En vuelo las dos a 0!!

int mEstable = 0;  // Modo estable
int mAcro = 0;     // Modo acrobatico (no recomendado)

#define usCiclo 6000  // Ciclo de ejecucion de software en microsegundos
// ==========================================

#include <EnableInterrupt.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

//=============================================//////////////////// VARIABLES GLOBALES ////////////////////=============================================
// TIEMPOS
long tiempo_ejecucion, loop_timer, loop_timer1, esc_loop_timer, tiempo_ON, tiempo_1, tiempo_2;

// VBAT
bool AvisoVbatLow = false;
float tension_bateria, lectura_bat = 0.00;
int cbat, Cont_low_Vbat, low_bat = 0;

// MPU6050
#define gyro_address 0x68
float angulo_pitch, angulo_roll, angulo_yaw, yawGyro, pitchGyro, rollGyro, angle_pitch_acc, angle_roll_acc, temperature;
int emerg, gx, gy, gz;
float acc_total_vector, ax, ay, az;
float gyro_X_cal, gyro_Y_cal, gyro_Z_cal, gyro_roll_input , gyro_pitch_input , gyro_yaw_input;
float angle_pitch_acc_cal, angle_roll_acc_cal, angle_yaw_acc_cal;

/// MOTORES
float esc1, esc2, esc3, esc4;

// MANDO RC
float calPotencia, pulsoPotencia, pulsoPotenciaAnt, PotenciaFilt = 0.00;
float wPitchConsigna, wRollConsigna, wYawConsigna = 0.00;
float calPitch, wPitch,  wPitchFilt = 0.00;
float calRoll, wRoll, wRollFilt = 0.00;
float calYaw, wYaw,  wYawlFilt = 0.00;

// PID
float PIDwInPitch, pitch_w_giroscopio, pitch_w_error_prop, ITerm_pitch_w, pitch_w_giroscopio_anterior, PID_pitch_w, DPitch_w;
float PIDwInRoll, roll_w_giroscopio, roll_w_error_prop, ITerm_roll_w, roll_w_giroscopio_anterior, PID_roll_w, DRoll_w;
float PIDwInYaw, yaw_w_giroscopio, yaw_w_error_prop, ITerm_yaw_w, yaw_w_giroscopio_anterior, PID_yaw_w, PID_yaw_w_ant, DYaw_w;
float pitch_ang_giroscopio, pitch_ang_error_prop, ITerm_pitch_ang, pitch_ang_giroscopio_anterior, PID_pitch_ang, DPitch_ang;
float  roll_ang_giroscopio, roll_ang_error_prop, ITerm_roll_ang, roll_ang_giroscopio_anterior, PID_roll_ang, DRoll_ang;

// CALIBRACION
int cal_int = 0;

//=============================================//////////////////// SETUP ////////////////////=============================================
void setup() {
  init_General();  // Iniciacion general
  init_MPU6050();  // Iniciacion MPU6050
  VbatCheck();     // Leer tension de bateria
  Calib_MandoRC(); // Calibrar mando RC
  Calib_MPU6050(); // Calibrar MPU6050
  Calib_Motores(); // Calibrar motores

  loop_timer = micros();
}

// =============================================//////////////////// LOOP ////////////////////=============================================
void loop() {
  ProcesMPU();                     // Obtener inclinacion (º)
  if (mEstable == 1)PID_ang();     // Obtener salida PID inclinacion (PIDang->PIDw->Planta)
  PID_w();                         // Obtener salida PID velocidad
  Modulador();                     // Modulador en base a salidas PID
  // Si la duracion del ciclo ha sido mayor de usCiclo+50us, encender led de aviso.
  // Nunca hay que sobrepasar el tiempo de ciclo que hayamos estipulado.
  if (micros() - loop_timer > usCiclo + 50) digitalWrite(13, HIGH);
  while (micros() - loop_timer < usCiclo); // Comienzo de un nuevo ciclo
  loop_timer = micros();          // Registrar instante de comienzo del ciclo
  PWM();                          // Generar PWMs de salida
  // A continuacin esperamos 2200us para leer el sensor MPU6050. La duracion de la salida PWM puede variar entre
  // 1000us y 2000us. Lo que hacemos es esperar siempre 2200us y asegurarnos de leer el sensor siempre en el mismo
  // instante.
  while (micros() - loop_timer < 2200);
  MPU_6050();    // Leer MPU6050

  if (visu == 1 || matlab == 1)Visualizaciones();             // Visualizacicion de variables
}
