void setup(){
    Serial.begin(115200);
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    digitalWrite(2,HIGH);
    digitalWrite(3,HIGH);
    pinMode(11, INPUT_PULLUP);
    pinMode(12, INPUT_PULLUP);
    pinMode(14, INPUT);
    pinMode(15, INPUT);
    pinMode(16, INPUT);
    pinMode(17, INPUT);
}

void loop(){
    float vrx1 = (analogRead(14) / 512.0) - 1.0;
    float vry1 = (analogRead(15) / 512.0) - 1.0;
    float vrx2 = (analogRead(16) / 512.0) - 1.0;
    float vry2 = (analogRead(17) / 512.0) - 1.0;

    float angle1 = (atan2(-vrx1, -vry1) * (180/PI)) + 180;
    float speed1 = (sqrt(sq(vrx1) + sq(vry1))) / 1.41; 
    float angle2 = (atan2(-vrx2, -vry2) * (180/PI)) + 180;
    float speed2 = (sqrt(sq(vrx2) + sq(vry2))) / 1.41; 

    /*Serial.println("btn1: " + String(digitalRead(11)) + 
                  " btn2: " + String(digitalRead(12)) +
                  " vrx1: " + String(vrx1) +
                  " vry1: " + String(vry1) +
                  " vrx2: " + String(vrx2) +
                  " vry2: " + String(vry2) +
                  " angle1: " + String(angle1) +
                  " speed1: " + String(speed1) +
                  " angle2: " + String(angle2) +
                  " speed2: " + String(speed2));*/

    Serial.write(0x55);
    Serial.write(0x55);
    Serial.write((byte)(angle1 * 0.7));
    Serial.write((byte)(angle2 * 0.7));
    Serial.write((byte)(speed1 * 100));
    Serial.write((byte)(speed2 * 100));
    Serial.write((byte)digitalRead(11));
    Serial.write((byte)digitalRead(12));

    delay(20);
}