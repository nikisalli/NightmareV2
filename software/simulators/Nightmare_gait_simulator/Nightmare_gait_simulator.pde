import g4p_controls.*; //<>//
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
float Cx, Cy, cx, cy, ray, walkcirc, ang;
float[][] bpos = new float[3][2];
float[][] cbpos = new float[3][2];
boolean first_step = true;

int[] sequence = {4, 2, 6, 0, 3, 5, 1, 7};
int[] tarantula = {4, 2, 7, 1, 3, 5, 0, 6};

void setup() {
  size(1920, 1080);
  for (int i=0; i<8; i++) {
    pos[0][i] = spos[0][i]; //x
    pos[1][i] = spos[1][i]; //y
  }

  PFont pfont = createFont("Arial", 10, false); // use true/false for smooth/no-smooth
  ControlFont font = new ControlFont(pfont, 30);
  G4P.setGlobalColorScheme(GCScheme.BLUE_SCHEME);
  cp5 = new ControlP5(this);
  cp5.addSlider("slider").setPosition(760, 980).setRange(-50, 50).setSize(400, 40).setFont(font);
  cp5.addButton("toggle").setValue(0).setPosition(460, 980).setSize(200, 40).setFont(font);
}

void draw() {
  if (done) {
    thread("lol");
  }
  background(200);
  Cx = 0;
  Cy = ((0.1 / abs(slider*1.76)) * 30000.0) - 34.0;
  if (slider<0) {
    Cy=-Cy;
  }
  cy = Cy/2;
  cx = Cx/2;
  if (Cy > 8000) {
    cy = 4000;
  } else if (Cy < -8000) {
    cy = -4000;
  }
  ray = sqrt(sq(cy) + sq(Cx));
  walkcirc = 2 * PI * (ray + spos[1][5]);
  ang = ((arc * 360.0) / walkcirc) * 2;
  bodytopos();
}

void lol() {
  done = false;

  for (int a=0; a<8; a++) {
    int fz, pz;
    fz=tarantula[a];
    if (a==0) {
      pz=tarantula[7];
    } else {
      pz=tarantula[a-1];
    }
    bpos[0][1] = bpos[0][0];//p
    bpos[1][1] = bpos[1][0];//p
    bpos[2][1] = bpos[2][0];//p
    bpos[0][0] = pos[0][fz];//f
    bpos[1][0] = pos[1][fz];//f
    bpos[2][0] = pos[2][fz];//f
    for (float b=0; b<1; b+=0.05) {
      for (int c=0; c<8; c++) {
        if (!first_step) {
          if (c!=fz && c!=pz) {
            float[] ar = RmatrixZ(pos[0][c], pos[1][c], pos[2][c], -(ang/120), cx, cy, 0);
            pos[0][c] = ar[0];
            pos[1][c] = ar[1];
          }
        } else {
          if (c!=fz) {
            float[] ar = RmatrixZ(pos[0][c], pos[1][c], pos[2][c], -(ang/120), cx, cy, 0);
            pos[0][c] = ar[0];
            pos[1][c] = ar[1];
          }
        }
      }
      if (!first_step) {
        float[] cr=RmatrixZ(spos[0][pz], spos[1][pz], spos[2][pz], ang/2.0, cx, cy, 0);//prev
        float[][] bm = {//prev
          {cr[0], cr[0], bpos[0][1], bpos[0][1]}, 
          {cr[1], cr[1], bpos[1][1], bpos[1][1]}, 
          {spos[2][pz], spos[2][pz]+step_h, spos[2][pz]+step_h, spos[2][pz]}, 
        };
        float[] dr = Spline3D(bm, (b/2.0)+0.5);//prev
        pos[0][pz] = dr[0];
        pos[1][pz] = dr[1];
        pos[2][pz] = dr[2];
        cbpos[0][1] = cr[0];
        cbpos[1][1] = cr[1];
        cbpos[2][1] = cr[2];
      }
      float[] br=RmatrixZ(spos[0][fz], spos[1][fz], spos[2][fz], ang/2.0, cx, cy, 0);//forw
      float[][] am = {//forw
        {br[0], br[0], bpos[0][0], bpos[0][0]}, 
        {br[1], br[1], bpos[1][0], bpos[1][0]}, 
        {spos[2][fz], spos[2][fz]+step_h, spos[2][fz]+step_h, spos[2][fz]}, 
      };
      float[] er = Spline3D(am, b/2.0);//forw
      pos[0][fz] = er[0];
      pos[1][fz] = er[1];
      pos[2][fz] = er[2];
      cbpos[0][0] = br[0];
      cbpos[1][0] = br[1];
      cbpos[2][0] = br[2];
      delay(2);
    }
    first_step = false;
  }
  
  done = true;
}

void bodytopos() {
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
    stroke(0);
    fill(255, 0, 255);
    for (int i=0; i<8; i++) {
      rect((cbpos[0][0]*12)+(1920/2)-8, (cbpos[1][0]*12)+(1080/2)-8, 16, 16);
      if (!first_step) {
        rect((cbpos[0][1]*12)+(1920/2)-8, (cbpos[1][1]*12)+(1080/2)-8, 16, 16);
      }
    }
  }
  stroke(255, 0, 0);
  noFill();
  int lol = 204;
  int kek = 300;
  int asd = 540;
  if (slider == 0) {
    line(0, asd, 1920, asd);
    line(0, asd+lol, 1920, asd+lol);
    line(0, asd+kek, 1920, asd+kek);
    line(0, asd-lol, 1920, asd-lol);
    line(0, asd-kek, 1920, asd-kek);
  } else {
    circle((Cx*6)+(1920/2), (Cy*6)+asd, Cy*12);
    circle((Cx*6)+(1920/2), (Cy*6)+asd, Cy*12-lol*2);
    circle((Cx*6)+(1920/2), (Cy*6)+asd, Cy*12+lol*2);
    circle((Cx*6)+(1920/2), (Cy*6)+asd, Cy*12-kek*2);
    circle((Cx*6)+(1920/2), (Cy*6)+asd, Cy*12+kek*2);
  }
  stroke(0);
  fill(0, 0, 255);
  rect((Cx*6)+(1920/2)-8, (Cy*6)+(1080/2)-8, 16, 16);
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
