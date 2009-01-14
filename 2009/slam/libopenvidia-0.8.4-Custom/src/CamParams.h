#ifndef _CAMPARAMS_H
#define _CAMPARAMS_H
class CamParams {
private:
    float paramf1, paramf2, paramox, paramoy, paramk1, paramk2, paramk3, paramk4;
public:
    CamParams(float f1, float f2, float ox, float oy,
              float k1, float k2, float k3, float k4) {
        paramf1 = f1;
        paramf2 = f2;
        paramox = ox;
        paramoy = oy;
        paramk1 = k1;
        paramk2 = k2;
        paramk3 = k3;
        paramk4 = k4;
    }
    float f1() {
        return paramf1;
    }
    float f2() {
        return paramf2;
    }
    float ox() {
        return paramox;
    }
    float oy() {
        return paramoy;
    }
    float k1() {
        return paramk1;
    }
    float k2() {
        return paramk2;
    }
    float k3() {
        return paramk3;
    }
    float k4() {
        return paramk4;
    }
};
#endif
