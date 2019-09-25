import g4p_controls.*;
import controlP5.*;

ControlP5 cp5;
Slider abc;

float[][] spos = {
  { 22, 8, -8, -22, -22, -8, 8, 22}, //X
  {-17, -25, -25, -17, 17, 25, 25, 17}, //Y
  {-10, -10, -10, -10, -10, -10, -10, -10}, //Z
};

boolean done = true;
boolean stopped = true;

float[][] pos = new float[3][8];
float arc = 4;
float step_h = 6;
int slider = -10;
int sliderx = 0;
int slidery = 0;
float ang, dirx, diry;
float[][] bpos = new float[3][8];
boolean first_step = true;

int[] sequence = {4, 2, 6, 0, 3, 5, 1, 7};
int[] tarantula = {4, 2, 7, 1, 3, 5, 0, 6};

void setup() {
  size(1920, 1500);
  for (int i=0; i<8; i++) {
    pos[0][i] = spos[0][i]; //x
    pos[1][i] = spos[1][i]; //y
  }

  PFont pfont = createFont("Arial", 10, false); // use true/false for smooth/no-smooth
  ControlFont font = new ControlFont(pfont, 30);
  G4P.setGlobalColorScheme(GCScheme.BLUE_SCHEME);
  cp5 = new ControlP5(this);
  cp5.addSlider("slider").setPosition(760, 1320).setRange(-15, 15).setSize(400, 40).setFont(font);
  cp5.addSlider("sliderx").setPosition(760, 1260).setRange(-5, 5).setSize(400, 40).setFont(font);
  cp5.addSlider("slidery").setPosition(760, 1200).setRange(-5, 5).setSize(400, 40).setFont(font);
  cp5.addButton("toggle").setValue(0).setPosition(460, 1320).setSize(200, 40).setFont(font);
}

void draw() {
  if (done) {
    thread("lol");
  }
  background(200);
  ang = slider;
  dirx = sliderx;
  diry = slidery;
  bodytopos();
}

void lol() {
  done = false;
  for (int a=0; a<2; a++) {
    for (int c=0; c<4; c++) {
      int i = c*2+(1-a);
      bpos[0][i] = pos[0][i];
      bpos[1][i] = pos[1][i];
      bpos[2][i] = pos[2][i];
    }
    for (float b=0; b<1; b+=0.01) {
      for (int c=0; c<4; c++) {
        int i = c*2+a;
        float[] ar = RmatrixZ(pos[0][i], pos[1][i], pos[2][i], -(ang/100), 0, 0, 0);
        ar = Tmatrix(ar[0], ar[1], ar[2], -(dirx/100), -(diry/100), 0);
        pos[0][i] = ar[0];
        pos[1][i] = ar[1];
      }
      for (int c=0; c<4; c++) {
        int i = c*2+(1-a);
        float[] cr=RmatrixZ(spos[0][i], spos[1][i], spos[2][i], ang/2.0, 0, 0, 0);
        cr = Tmatrix(cr[0], cr[1], cr[2], (dirx/2), (diry/2), 0);
        float[][] bm = {//prev
          {cr[0], cr[0], bpos[0][i], bpos[0][i]}, 
          {cr[1], cr[1], bpos[1][i], bpos[1][i]}, 
          {spos[2][i], spos[2][i]+step_h, spos[2][i]+step_h, spos[2][i]}, 
        };
        float[] dr = Spline3D(bm, b);
        pos[0][i] = dr[0];
        pos[1][i] = dr[1];
        pos[2][i] = dr[2];
      }
      delay(6);
    }
    first_step = false;
  } 
  done = true;
}

void bodytopos() {
  stroke(0);
  fill(0, 0, 255);
  rect(1920/2, 1080/2, 16, 16);
  stroke(0);
  fill(255, 0, 0);
  for (int i=0; i<8; i++) {
    rect((pos[0][i]*12)+(1920/2)-8, (pos[1][i]*12)+(1080/2)-8, 16, 16);
  }
  if (stopped) {
    stroke(0);
    fill(0, 0, 255);
    for (int i=0; i<8; i++) {
      rect((bpos[0][0]*12)+(1920/2)-8, (bpos[1][0]*12)+(1080/2)-8, 16, 16);
      if (!first_step) {
        rect((bpos[0][1]*12)+(1920/2)-8, (bpos[1][1]*12)+(1080/2)-8, 16, 16);
      }
    }
  }
}

float[] Spline2D(float[][] p, float ti) {
  float[] calc = new float[4];
  float[] calc_ = new float[4];
  for (int i = 0; i < 4; i++) {
    calc[i] = p[0][i];
    calc_[i] = p[1][i];
  }
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3 - i; j++) {
      calc[j] = timeVal(ti, calc[j], calc[j + 1]);
      calc_[j] = timeVal(ti, calc_[j], calc_[j + 1]);
    }
  }
  float[] kek = {calc[0], calc_[0]};
  return kek;
}

float[] Spline3D(float[][] p, float ti) {
  float[] calc = new float[4];
  float[] calc_ = new float[4];
  float[] calc__ = new float[4];
  for (int i = 0; i < 4; i++) {
    calc[i] = p[0][i];
    calc_[i] = p[1][i];
    calc__[i] = p[2][i];
  }
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3 - i; j++) {
      calc[j] = timeVal(ti, calc[j], calc[j + 1]);
      calc_[j] = timeVal(ti, calc_[j], calc_[j + 1]);
      calc__[j] = timeVal(ti, calc__[j], calc__[j + 1]);
    }
  }
  float[] kek = {calc[0], calc_[0], calc__[0]};
  return kek;
}

float timeVal(float t, float higher, float lower) { //0->1 = higher->lower
  return t * (higher - lower) + lower;
}

float[] Tmatrix(float Px, float Py, float Pz, float Dx, float Dy, float Dz) {
  float[] kek = {Px+Dx, Py+Dy, Pz+Dz};
  return kek;
}

float[] RmatrixX(float Px, float Py, float Pz, float alfa, float Cx, float Cy, float Cz) {
  float TPx = Px;
  float TPy = Py;
  float TPz = Pz;
  Py = cos(toRad(alfa)) * (TPy - Cy) + sin(toRad(alfa)) * (TPz - Cz) + Cy;
  Pz = cos(toRad(alfa)) * (TPz - Cz) - sin(toRad(alfa)) * (TPy - Cy) + Cz;
  float[] kek = {Px, Py, Pz};
  return kek;
}

float[] RmatrixY(float Px, float Py, float Pz, float beta, float Cx, float Cy, float Cz) {
  float TPx = Px;
  float TPy = Py;
  float TPz = Pz;
  Px = cos(toRad(beta)) * (TPx - Cx) - sin(toRad(beta)) * (TPz - Cz) + Cx;
  Pz = sin(toRad(beta)) * (TPx - Cx) + cos(toRad(beta)) * (TPz - Cz) + Cz;
  float[] kek = {Px, Py, Pz};
  return kek;
}

float[] RmatrixZ(float Px, float Py, float Pz, float gamma, float Cx, float Cy, float Cz) {
  float TPx = Px;
  float TPy = Py;
  float TPz = Pz;
  Px = cos(toRad(gamma)) * (TPx - Cx) + sin(toRad(gamma)) * (TPy - Cy) + Cx;
  Py = cos(toRad(gamma)) * (TPy - Cy) - sin(toRad(gamma)) * (TPx - Cx) + Cy;
  float[] kek = {Px, Py, Pz};
  return kek;
}

float[] Rmatrix(float x_, float y_, float z_, float alfa, float beta, float gama, float centerX_, float centerY_, float centerZ_) {
  float[] kek = {x_, y_, z_};
  kek = RmatrixX(kek[0], kek[1], kek[2], alfa, centerX_, centerY_, centerZ_);
  kek = RmatrixY(kek[0], kek[1], kek[2], beta, centerX_, centerY_, centerZ_);
  kek = RmatrixZ(kek[0], kek[1], kek[2], gama, centerX_, centerY_, centerZ_);
  return kek;
}

float toRad(float deg) {
  return deg * (PI / 180.0);
}

float toDeg(float rad) {
  return rad * (180.0 / PI);
}

public void toggle() {
  stopped = !stopped;
}
