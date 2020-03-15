///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Lectura de la tension de bateria
// https://arduproject.es/bateria-lipo/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void VbatCheck() {
  lectura_bat = analogRead(A6); // Leer entrada analogica
  tension_bateria = 2.95 * (lectura_bat * 4.92  / 1023);

  // Si la tension es inferior a 10.5V, se activa la señal de bateria baja
  if (tension_bateria < 10.5) {
    Cont_low_Vbat++;
    AvisoVbatLow = true;
  }
  else Cont_low_Vbat = 0;
  if (visu == 0 && matlab == 0 && Cont_low_Vbat > 500) {
    low_bat = 1;

    //      lcd.backlight();
    //      lcd.setCursor(0, 0);
    //      lcd.print(tension_bateria);
  }
  else low_bat = 0;
}
