void alarmState(int x) {
  if (x < 50) {
    if (alarmGreen == true) {
    } else {
      alarmGreen = true;
      alarmYellow = false;
      alarmRed = false;
      alarmtime = millis();
    }
  } else if (x < 80) {
    if (alarmYellow == true) {
    } else {
      alarmGreen = false;
      alarmYellow = true;
      alarmRed = false;
      alarmtime = millis();
    }
  } else if (x <= 100) {
    if (alarmRed == true) {
    } else {
      alarmGreen = false;
      alarmYellow = false;
      alarmRed = true;
      alarmtime = millis();
    }
  }
}

/*
void alarmState(int x) {
  if ((x <= 50) && alarmGreen == false) {
    alarmGreen = true;
    alarmYellow = false;
    alarmRed = false;
    alarmtime = millis();
  } else if ((50 < x <= 80) && alarmYellow == false) {
    alarmGreen = false;
    alarmYellow = true;
    alarmRed = false;
    alarmtime = millis();
  } else if ((80 < x <= 100) && alarmRed == false) {
    alarmGreen = false;
    alarmYellow = false;
    alarmRed = true;
    alarmtime = millis();
  }
}
*/

void alarmTurnOn() {
  timer = millis();
  if ((timer - alarmtime) > 800) {
    if (alarmRed == true || alarmFS == true) {
      // Serial.println("AlarmState: RED");
      tone(BUZZER, 1500, 250);
      alarmtime = timer;
    } else if (alarmYellow == true) {
      // Serial.println("AlarmState: YELLOW");
      tone(BUZZER, 1000, 250);
      alarmtime = timer;

    } else {
      // Serial.println("AlarmState: Green");
      alarmtime = timer;
    }
  }
}
