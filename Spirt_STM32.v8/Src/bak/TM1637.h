#ifndef TM1637_H_
#define TM1637_H_


void TN1637_init(void);
void TM1637SetBrightness(unsigned char brightness);
void TM1637Start(void);
void TM1637Stop(void);
void TM1637ReadAck(void);
void TM1637WriteByte(unsigned char b);
void TM1637DisplayDecimal(unsigned int v, unsigned char displaySeparator);


#endif 
