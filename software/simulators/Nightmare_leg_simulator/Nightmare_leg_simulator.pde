float alpha= 90;
float beta = 90;
float gama = 90;
float cx = 6.5;
float fm = 13.0;
float tb = 17.0;
float x = 0;
float y = 20;
float z = 0;
float i;
int a;

void setup() {
  size(640, 320);
}

void draw() {
  class leg{
    public float CX_LENGHT = cx;
    public float FM_LENGHT = fm;
    public float TB_LENGHT = tb;
  };
  
  leg dim = new leg();
  
  background(226);
  line(y*10, 160, y*10, 160+100);
  if (a == 1) {
    i+=0.1;
  } else if (a == 0) {
    i-=0.1;
  }
  if (i>15) {
    a=0;
  }
  if (i<5) {
    a=1;
  }
  z = i;
  float L, L1, alpha1, alpha2;

  L1 = sqrt(sq(x) + sq(y));
  gama = - (atan(x / y) / PI * 180);
  L = sqrt(sq(L1 - dim.CX_LENGHT) + sq(z));
  beta = - (acos((sq(dim.TB_LENGHT) + sq(dim.FM_LENGHT) - sq(L)) / (2 * dim.TB_LENGHT * dim.FM_LENGHT)) / PI * 180) + 180;
  alpha1 = acos(z / L) / PI * 180;
  alpha2 = acos((sq(dim.FM_LENGHT) + sq(L) - sq(dim.TB_LENGHT)) / (2 * dim.FM_LENGHT * L)) / PI * 180;
  alpha = - (alpha1 + alpha2) + 90;
  drawleg(alpha, beta);
}

void drawleg(float alpha, float beta) {
  print(alpha); 
  print("  "); 
  println(beta);
  alpha = (180-alpha) + 180;
  beta  = (180-beta);
  stroke(255, 0, 0);
  line(0, 160, cx*10, 160);
  stroke(0, 255, 0);
  float a = cx*10+cos(radians(alpha))*fm*10;
  float b = 160-sin(radians(alpha))*fm*10;
  line(cx*10, 160, a, b);
  stroke(0, 0, 255);
  line(a, b, a+cos(radians((180-beta-alpha)))*tb*10, b+sin(radians((180-beta-alpha)))*tb*10);
  point(a+cos(radians((180-beta-alpha)))*tb*10, b+sin(radians((180-beta-alpha)))*tb*10);
}
