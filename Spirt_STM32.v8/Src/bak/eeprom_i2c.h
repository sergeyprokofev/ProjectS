void EE_init(I2C_TypeDef* I2Cx, uint32_t speed);
void EE_single_write(I2C_TypeDef* I2Cx, uint8_t HW_address, uint8_t addr, uint8_t data);
void EE_burst_write(I2C_TypeDef* I2Cx, uint8_t HW_address, uint8_t addr, uint8_t n_data, uint8_t *data);
uint8_t EE_single_read(I2C_TypeDef* I2Cx, uint8_t HW_address, uint8_t addr);
void EE_burst_read(I2C_TypeDef* I2Cx, uint8_t HW_address, uint8_t addr, uint8_t n_data, uint8_t *data);

