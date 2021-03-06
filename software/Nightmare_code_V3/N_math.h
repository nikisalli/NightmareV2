#ifndef N_MATH_H
#define N_MATH_H

void trigz(float &alpha, float &beta, float &gama, float x, float y, float z, struct Dleg dim);
void ftrigz(float &x, float &y, float &z, float alpha, float beta, float gama, struct Dleg dim);
void RmatrixX(float &Px, float &Py, float &Pz, float alfa , float Cx, float Cy, float Cz);
void RmatrixY(float &Px, float &Py, float &Pz, float beta , float Cx, float Cy, float Cz);
void RmatrixZ(float &Px, float &Py, float &Pz, float gamma, float Cx, float Cy, float Cz);
void Rmatrix(float &x_, float &y_, float &z_, float alfa, float beta, float gama, float centerX_, float centerY_, float centerZ_);
void Tmatrix(float &Px, float &Py, float &Pz, float Dx, float Dy, float Dz);
void Spline2D(float p[2][11], float ti, float &a, float &b);
void Spline3D(float p[3][4], float ti, float &a, float &b, float &c);
float timeVal(float t, float higher, float lower);
float toRad(float deg);
float toDeg(float rad);
float fmap(float x, float in_min, float in_max, float out_min, float out_max);
bool compareMatrix(float matrix[3][8], const float matrix1[3][8]);
float limit(float val, float downlimit, float uplimit);

class lp_filter{
  private:
    float setpoint;
    float rc_constant;
    float prev_time;
    float prev_val;
  public:
    lp_filter(double rc, double start_val);
    lp_filter(double rc);
    void set_input(double val);
    double get_val();
    void set_constant(double constant);
};

#endif //RHEX_MATH_H
