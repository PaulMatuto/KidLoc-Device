/************* gps  ******************************************/
void getGPS()
{
  //  Serial.println("GPS");
  // This sketch displays information every time a new sentence is correctly encoded.
  while (SerialGPS.available() > 0)
    if (gps.encode(SerialGPS.read()))
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true);
  }

}

void displayInfo()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    latn = gps.location.lat();
    longn = gps.location.lng();
    Serial.print(latn, 6); Serial.print(","); Serial.println(longn, 6);
    //    Serial.print(gps.location.lat(), 6);
    //    Serial.print(F(","));
    //    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }
}
/************* initialize gsm **************************************************/
void sendATCommand(String comms)
{
  Serial.print("Sending: "); Serial.println(comms);
  SerialAT.println(comms); getGsmReply();
}
void initGSM()
{
  Serial.println("INITIALIZING GSM");
  //  SerialGSM.listen(); SerialGSM.flush();
  sendATCommand("AT");
  sendATCommand("AT");
  sendATCommand("ATE0");
  sendATCommand("AT+CMGF=1");
  sendATCommand("AT+CFUN=1");
  sendATCommand("AT+CNMI=0,0,0,0,0");
}
void getGsmReply()
{
  //  String reply = "";
  //  while (!SerialAT.available());
  //  reply = SerialAT.readStringUntil('\n');
  //  Serial.print("REC: ");  Serial.print(reply);
  //  delay(50);
  String reply = "";
  long timeout = millis() + 2000; // 3 seconds timeout

  while (millis() < timeout) {
    while (SerialAT.available()) {
      char c = SerialAT.read();
      reply += c;
    }
  }

  Serial.print("REC: ");
  Serial.println(reply);
}

/***** send message **********************************************************/
void smsSend(String msg, String num)
{
  //  SerialGSM.listen(); SerialGSM.flush();
  Serial.println("Sending Message");
  Serial.print("MSG: ");Serial.println(msg);
  Serial.print("NUM: ");Serial.println(num);
  sendATCommand("AT+CMGF=1"); delay(500);
  String reply;
  SerialAT.print("AT+CMGS=\"");
  SerialAT.print(num);
  SerialAT.println("\"");
  delay(500);

  SerialAT.print(msg);
  delay(500);

  SerialAT.write(26); // CTRL+Z
  delay(1000);
  getGsmReply();
//  delay(3000);

}
