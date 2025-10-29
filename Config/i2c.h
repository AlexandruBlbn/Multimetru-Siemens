/*
 * i2c.h
 *
 * Created: 10/21/2025 2:26:28 PM
 *  Author: Alexandru
 */ 


#ifndef I2C_H_
#define I2C_H_




#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

#define F_CPU 16000000UL
#define SCL 100000L


static inline void i2c_init(void) {
	TWBR = ((F_CPU / SCL) - 16) / 2;
	TWSR = (0 << TWPS0) | (0 << TWPS1);
	TWCR = (1 << TWEN);
}

static inline void i2c_start(void) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
}

static inline void i2c_stop(void) {
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

static inline uint8_t i2c_write(uint8_t data) {
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
	
	uint8_t status = TWSR & 0xF8;
	return status;
}

static inline uint8_t i2c_read_ack(void) {
	TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
	return TWDR;
}

static inline uint8_t i2c_read_nack(void) {
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
	return TWDR;
}

static inline void i2c_write_register(uint8_t addr, uint8_t reg, uint8_t value) {
	i2c_start();
	
	uint8_t stat = i2c_write((addr << 1) | 0x00);
	if ((stat & 0xF8) != 0x18) {
		i2c_stop();
		return;
	}
	
	stat = i2c_write(reg);
	if ((stat & 0xF8) != 0x28) {
		i2c_stop();
		return;
	}
	
	stat = i2c_write(value);
	if ((stat & 0xF8) != 0x28) {
		i2c_stop();
		return;
	}
	
	i2c_stop();
}

static inline uint8_t i2c_read_register(uint8_t addr, uint8_t reg) {
	i2c_start();
	
	uint8_t stat = i2c_write((addr << 1) | 0x00);
	if ((stat & 0xF8) != 0x18) {
		i2c_stop();
		return 0xFF;
	}
	
	stat = i2c_write(reg);
	if ((stat & 0xF8) != 0x28) {
		i2c_stop();
		return 0xFF;
	}
	
	i2c_start();
	
	stat = i2c_write((addr << 1) | 0x01);
	if ((stat & 0xF8) != 0x40) {
		i2c_stop();
		return 0xFF;
	}
	
	uint8_t data = i2c_read_nack();
	i2c_stop();
	return data;
}

static inline void i2c_read_bytes(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len) {
	i2c_start();
	i2c_write((addr << 1) | 0x00);
	i2c_write(reg);
	i2c_start();
	i2c_write((addr << 1) | 0x01);
	
	for (uint8_t i = 0; i < len; i++) {
		if (i < len - 1) {
			buffer[i] = i2c_read_ack();
			} else {
			buffer[i] = i2c_read_nack();
		}
	}
	
	i2c_stop();
}

static inline uint8_t i2c_device_present(uint8_t addr) {
	i2c_start();
	i2c_write(addr);
	
	uint8_t status = TWSR & 0xF8;
	
	i2c_stop();
	
	return (status == 0x18) ? 1 : 0;
}


#endif /* I2C_H_ */