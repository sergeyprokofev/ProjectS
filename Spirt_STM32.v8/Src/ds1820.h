#ifndef	DS18B20
#define	DS18B20

uint8_t send_presence(uint8_t PinNumb);
void one_wire_write_bit(uint8_t bit,uint8_t PinNumb);
uint8_t one_wire_read_bit(uint8_t PinNumb);
void one_wire_write_byte(uint8_t data, uint8_t PinNumb);

void ds18d20_init(uint8_t PinNumb);
void ds18d20_start(uint8_t PinNumb);
int ds18d20_read(uint8_t PinNumb);
int ds1820_readtemp(void);



#endif

