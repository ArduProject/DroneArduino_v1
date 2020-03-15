///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// LED bucle while
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// El led 11 parpadea cuendo entremos en el loop principal
// y aqui s hace la gestion del parpadeo. 

int led1 = 0;

void ONled() { // LED bucle while
  if (led1 > 50 / 5 && led1 < 105 / 5) {
    if (low_bat == 1) { // Poca bateria
      analogWrite(A2, 0);
    }
    digitalWrite(11, HIGH);
    if (led1 == 100 / 5) led1 = 0;
  }
  else {
    if (low_bat == 1) {  // Poca bateria
      analogWrite(A2, 255);
    }
    digitalWrite(11, LOW);
  }
  led1++;
}
