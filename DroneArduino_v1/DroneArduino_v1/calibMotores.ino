///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Calibracion motores
// https://arduproject.es/motores-esc-y-su-programacion-en-arduino/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Calib_Motores() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calibrar motores");
  // Calibrar motores --> Mover stick de Roll a la derecha para continuar. Simplemente mandamos pulsos de 1000us (pulso minimo) a los motores.
  // Hasta no hacerlo no salimos de este bucle. Con esto conseguimos entrar al loop principal con los motores listos para girar.
  // Se puede comentar si se quiere, no es imprescindible.
  while (wRoll < 20) {
    pulsoPotencia =  ((PulsoMaxPotencia - PulsoMinPotencia) / (tMinPotencia - tMaxPotencia)) * ((PulsoPotencia) / 1000.00 - tMaxPotencia) + PulsoMinPotencia;
    wRoll =   ((wMaxRoll - wMinRoll) / (tMax - tMin)) * ((PulsoRoll - calRoll) / 1000.00 - tMin) + wMinRoll;
    digitalWrite(3, HIGH); //Motor 1
    digitalWrite(4, HIGH); //Motor 2
    digitalWrite(5, HIGH); //Motor 3
    digitalWrite(6, HIGH); //Motor 4
    delayMicroseconds(pulsoPotencia);
    digitalWrite(3, LOW); //Motor 1
    digitalWrite(4, LOW); //Motor 2
    digitalWrite(5, LOW); //Motor 3
    digitalWrite(6, LOW); //Motor 4
    delayMicroseconds(usCiclo - pulsoPotencia);
  }

  analogWrite(A2, 0);
  lcd.clear();
  lcd.noBacklight();
}
