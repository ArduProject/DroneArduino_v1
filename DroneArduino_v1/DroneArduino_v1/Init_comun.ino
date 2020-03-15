void init_General() {
  Wire.begin();
  TWBR = 24;    //Set the I2C clock speed to 400kHz.
  lcd.init();
  lcd.backlight();
  if (mEstable && mAcro == 0)mEstable = 1; // Si nos olvidamos de activar uno de los modos, por defecto modo estable

  // LEDs
  pinMode(2, OUTPUT);  // En desuso
  pinMode(A2, OUTPUT); // Led naranja, bateria
  pinMode(11, OUTPUT); // Led azul
  pinMode(10, OUTPUT); // Led rojo, IMU error
  pinMode(13, OUTPUT); // Led rojo tiempo excedido

  // MandoRC: declaracion de interrupciones
  pinMode(7, INPUT_PULLUP);                  // YAW
  enableInterrupt(7, INTyaw, CHANGE);
  pinMode(8, INPUT_PULLUP);                  // POTENCIA
  enableInterrupt(8, INTpotencia, CHANGE);
  pinMode(12, INPUT_PULLUP);                 // PITCH
  enableInterrupt(12, INTpitch, CHANGE);
  pinMode(9, INPUT_PULLUP);                  // ROLL
  enableInterrupt(9, INTroll, CHANGE);

  // Declarar motores
  pinMode(3, OUTPUT);  //Motor 1
  pinMode(4, OUTPUT);  //Motor 2
  pinMode(5, OUTPUT);  //Motor 3
  pinMode(6, OUTPUT);  //Motor 4

  digitalWrite(13, LOW);
  if (visu == 1 || matlab == 1) { // Para cuando visualizamos variables
    Serial.begin(115200);
    analogWrite(A2, 255);
  }

  digitalWrite(11, HIGH);
  lcd.setCursor(0, 0);
  lcd.print("Encender mando");
  lcd.setCursor(0, 1);
  if (mEstable == 1) lcd.print("<--mEstable-->");
  else lcd.print("<--mAcro-->");
}
