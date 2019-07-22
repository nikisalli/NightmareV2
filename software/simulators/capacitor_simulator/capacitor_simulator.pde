long prev_time;
float setpoint = 900;
float prev_setpoint;
float cap_vol = 0;
float prev_vol = 0;
float e = 2.7818;

void setup() {
  size(1920, 1080);
}

void draw() {
  stroke(0);
  fill(0);
  if (millis()<17000) {
    rect(millis()/10 + 10, 1080 - val(), 2, 2);
  }
  if (millis()>5000) {
    setpoint = 10;
  }
  print(millis());
  print(" ");
  println(cap_vol);
}

float val() {
  float interv = (millis() - prev_time)/1000.0;
  float a = interv / (1+interv);
  cap_vol = ((1 - a) * prev_vol) + (a * setpoint);
  prev_vol = cap_vol;
  prev_time = millis();
  return cap_vol;
}
