float p[][] = {
  {320,297,273,250,200,320,440,390,367,343,320},
  {250,250,250,250,250,  0,250,250,250,250,250},
};

float t;

void setup(){
  size(640, 360);
}

void draw(){
  t-=0.002;
  if(t<0){
    t=1;
  }
  stroke(0);
  fill(255,0,0);
  stroke(255,0,255);
  
  stroke(255,0,0);
  
  float calc[] = {};
  float calc_[] = {};
  calc = new float[11];
  calc_ = new float[11];
  for(int i=0;i<11;i++){
    calc[i]=p[0][i];
    calc_[i]=p[1][i];
  }
  for(int i=0;i<11;i++){
    for(int j=0;j<10-i;j++){
      calc[j]=timeVal(t,calc[j],calc[j+1]);
      calc_[j]=timeVal(t,calc_[j],calc_[j+1]);
    }
  }
  rect(calc[0],calc_[0],1,1);
}

float timeVal(float t, float higher, float lower){
  return t*(higher-lower)+lower;
}
