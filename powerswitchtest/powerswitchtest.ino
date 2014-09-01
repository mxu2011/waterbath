void setup() {               
  pinMode(13, OUTPUT);   
}

void loop() {

      digitalWrite(13, HIGH); // Turn the Powertail on
      Serial.println("Switch ON");
      delay(5000);
      
      digitalWrite(13, LOW); // Turn the Powertail on
      Serial.println("Switch OFF");
      delay(5000);
}
