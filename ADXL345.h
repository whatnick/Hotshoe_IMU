#ifndef ADXL345_h
#define ADXL345_h

void initADXL345(void);
void readAcc(void);


#define Register_ID 0
#define Register_2D 0x2D
#define Register_X0 0x32
#define Register_X1 0x33
#define Register_Y0 0x34
#define Register_Y1 0x35
#define Register_Z0 0x36
#define Register_Z1 0x37

#endif

