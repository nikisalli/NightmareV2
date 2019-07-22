import controlP5.*;
import g4p_controls.*;

ControlP5 cp5;
Slider abc;

float cx = 6.5;
float fm = 13;
float tb = 17;

float x = 0;
float y  = 10;
float z   = -10;

void setup() {
  size(1920, 1080);
  
  PFont pfont = createFont("Arial", 30, false); // use true/false for smooth/no-smooth
  ControlFont font = new ControlFont(pfont, 30);
  
  textFont(pfont);
  
  G4P.setGlobalColorScheme(GCScheme.BLUE_SCHEME);
  cp5 = new ControlP5(this);
  
  cp5.addSlider("x").setPosition(20, 60).setRange(-20, 20).setSize(400, 40).setFont(font);
  cp5.addSlider("y").setPosition(20, 120).setRange(-20, 20).setSize(400, 40).setFont(font);
  cp5.addSlider("z").setPosition(20, 180).setRange(-20, 20).setSize(400, 40).setFont(font);
}

void draw() {
  background(180);
  
  if(x == 0){
    x=0.0001;
  }
  
  if(y == 0){
    y=0.0001;
  }
  
  if(z == 0){
    z=0.0001;
  }
  
  //inverse kinematics
  float L1 = sqrt(sq(x) + sq(y));
  float gama = - (atan(x / y) / PI * 180);
  float L = sqrt(sq(L1 - cx) + sq(-z));
  float beta = - (acos((sq(tb) + sq(fm) - sq(L)) / (2 * tb * fm)) / PI * 180) + 180;
  float alpha1 = acos(-z / L) / PI * 180;
  float alpha2 = acos((sq(fm) + sq(L) - sq(tb)) / (2 * fm * L)) / PI * 180;
  float alpha = - (alpha1 + alpha2) + 90;
  
  //forward kinematics
  float L2 = cx + cos(radians(alpha))*fm + cos(radians((beta)+alpha))*tb;
  float x1 = sin(radians(-gama))*L2;
  float y1 = cos(radians(-gama))*L2;
  float z1 = -(sin(radians(alpha))*fm+sin(radians((beta)+alpha))*tb);
  
  text(Float.toString(L1),40,300);
  text(Float.toString(L2),300,300);
  text(Float.toString(x),40,340);
  text(Float.toString(x1),300,340);
  text(Float.toString(y),40,380);
  text(Float.toString(y1),300,380);
  text(Float.toString(z),40,420);
  text(Float.toString(z1),300,420);
  text(Float.toString(alpha),40,460);
  text(Float.toString(beta),300,460);
  text(Float.toString(gama),560,460);
} //<>//
