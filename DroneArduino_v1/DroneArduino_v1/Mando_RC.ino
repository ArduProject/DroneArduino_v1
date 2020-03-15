///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Mando RC
// http://arduproject.es/mando-rc-y-receptor-programacion-en-arduino-sketch/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// MANDO POTENCIA
const int PulsoMaxPotencia = 1830;
const int PulsoMinPotencia = 930;
const float tMaxPotencia = 1.83;  // <-- Si teneis la entrada Throttle invertida sustituid este valor
const float tMinPotencia = 1.12;  // <-- por este y viceversa 

const float tMax = 2;
const float tMin = 1;

// MANDO PITCH
const int wMaxPitch = -75; // <-- Si teneis la entrada Pitch invertida sustituid este valor
const int wMinPitch = 75;  // <-- por este y viceversa 

// MANDO ROLL
const int wMaxRoll = 60;  // <-- Si teneis la entrada Roll invertida sustituid este valor
const int wMinRoll = -60; // <-- por este y viceversa 

// MANDO YAW
const int wMaxYaw = 60;  // <-- Si teneis la entrada Yaw invertida sustituid este valor
const int wMinYaw = -60; // <-- por este y viceversa 

// SEGURIDAD
int cont_seguridad2;

//=============================================//////////////////// INTERRUPCIONES ////////////////////=============================================
volatile long contPotenciaInit; // LEER MANDO RC --> POTENCIA (Throttle)
volatile int PulsoPotencia;
void INTpotencia() {
  if (digitalRead(8) == HIGH) contPotenciaInit = micros();
  if (digitalRead(8) == LOW)PulsoPotencia = micros() - contPotenciaInit;
}

volatile long contPitchInit; // LEER MANDO RC --> PITCH
volatile int PulsoPitch;
void INTpitch() {
  if (digitalRead(12) == HIGH) contPitchInit = micros();
  if (digitalRead(12) == LOW) PulsoPitch = micros() - contPitchInit;
}

volatile long contRollInit; // LEER MANDO RC --> ROLL
volatile int PulsoRoll;
void INTroll() {
  if (digitalRead(9) == HIGH) contRollInit = micros();
  if (digitalRead(9) == LOW) PulsoRoll = micros() - contRollInit;
}

volatile long contYawInit; // LEER MANDO RC --> YAW
volatile int PulsoYaw;
void INTyaw() {
  if (digitalRead(7) == HIGH) contYawInit = micros();
  if (digitalRead(7) == LOW) PulsoYaw = micros() - contYawInit;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Calibracion mando RC
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Calib_MandoRC() {
  // Hasta no encender el mando no salimos de este bucle por seguridad
  while (pulsoPotencia > 2500 || pulsoPotencia < 500) {
    pulsoPotencia =  ((PulsoMaxPotencia - PulsoMinPotencia) / (tMinPotencia - tMaxPotencia)) * ((PulsoPotencia) / 1000.00 - tMaxPotencia) + PulsoMinPotencia;
    delay(100);
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calibrando ");
  lcd.setCursor(0, 1);
  lcd.print("mando.....");

  delay(100);
  // Calibracion del mando
  for (cal_int = 0; cal_int < 100 ; cal_int ++) {
    calPotencia = calPotencia + PulsoPotencia;
    calPitch = calPitch + PulsoPitch;
    calRoll = calRoll + PulsoRoll;
    calYaw = calYaw + PulsoYaw;
    delay(20);
  }

  calPotencia = calPotencia / 100 - 990;
  calPitch = calPitch / 100 - 1500;
  calRoll = calRoll / 100 - 1500;
  calYaw = calYaw / 100 - 1500;

  // Tras calibrar el mando, hacemos un comprobacion de las lecturas y vemos si entan dentro de rango
  wPitch =   ((wMinPitch - wMaxPitch) / (tMax - tMin)) * ((PulsoPitch - calPitch) / 1000.00 - tMin) + wMaxPitch;
  wRoll =   ((wMaxRoll - wMinRoll) / (tMax - tMin)) * ((PulsoRoll - calRoll) / 1000.00 - tMin) + wMinRoll;
  wYaw =  ((wMinYaw - wMaxYaw) / (tMax - tMin)) * ((PulsoYaw - calYaw) / 1000.00 - tMin) + wMaxYaw;
  if (wPitch < -4 || wPitch > 4 || wRoll > 4 || wRoll < -4 || wYaw > 4 || wYaw < -4 ) { // Test
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ERROR CALIBR. MANDO");
    lcd.setCursor(0, 1);
    lcd.print("Reset");
    while (1)delay(10);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Lectura mando RC
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void lectura_mandoRC() {
  // Procesamiento de la señal --> funcion map() de Arduino
  pulsoPotencia =  ((PulsoMaxPotencia - PulsoMinPotencia) / (tMinPotencia - tMaxPotencia)) * ((PulsoPotencia) / 1000.00 - tMaxPotencia) + PulsoMinPotencia;
  wPitch =   ((wMinPitch - wMaxPitch) / (tMax - tMin)) * ((PulsoPitch - calPitch) / 1000.00 - tMin) + wMaxPitch;
  wRoll =   ((wMaxRoll - wMinRoll) / (tMax - tMin)) * ((PulsoRoll - calRoll) / 1000.00 - tMin) + wMinRoll;
  wYaw =  ((wMinYaw - wMaxYaw) / (tMax - tMin)) * ((PulsoYaw - calYaw) / 1000.00 - tMin) + wMaxYaw;

  //  Filtro de lecturas del mando
  PotenciaFilt = PotenciaFilt * 0.9 + pulsoPotencia * 0.1;
  wPitchFilt = wPitchFilt * 0.9 + wPitch * 0.1;
  wRollFilt = wRollFilt * 0.9 + wRoll * 0.1;
  wYawlFilt = wYawlFilt * 0.9 + wYaw * 0.1;
  //  wPitchFilt = wPitch;
  //  wRollFilt = wRoll;
  //  wYawlFilt = wYaw;

  wPitchConsigna = wPitchFilt;
  wRollConsigna = wRollFilt;
  wYawConsigna = wYawlFilt;

  // Si las lecturas son cercanas a 0, las forzamos a 0 para evitar inclinar el drone
  // por error
  if (wPitchFilt < 9  && wPitchFilt > -9)wPitchConsigna = 0;  // '9' por un error de fabrica de mi mando
  if (wRollFilt < 3 && wRollFilt > -3)wRollConsigna = 0;
  if (wYawlFilt < 3  && wYawlFilt > -3)wYawConsigna = 0;

  if (mAcro == 1) {
    // Si volamos en modo estable, controlamos la velocidad de cada eje
    wPitchConsigna = wPitchConsigna;
    wRollConsigna = wRollConsigna;
  }
  else {
    // Si volamos en modo estable, controlamos la inclinacion de cada eje, por lo que
    // dividimos a consigna entre 2.6 para suavizar los movimientos (evitar movimientos bruscos)
    wPitchConsigna = wPitchConsigna / 2.6;
    wRollConsigna = wRollConsigna / 2.6;
  }

  // Detectar perdida mando: si la lectura del Mando RC es indentica a la medida anterior durante
  // un numero definido de lecturas, entendemos que hemos perdido la conexion con el mando RC.
  if (pulsoPotenciaAnt == pulsoPotencia)cont_seguridad2++;
  else cont_seguridad2 = 0;
  if (cont_seguridad2 > 150) { // Perdida conexion con mando RC
    // Tomar alguna medida si se desea
  }
  pulsoPotenciaAnt = pulsoPotencia;
}
