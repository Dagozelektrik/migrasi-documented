/**
 * adopted from
 * CMPS03 by: Aarom Berk
 * 
 * Bismillahirahmanirrahim
 */

#ifndef CMPS_DAGOZ_H
#define CMPS_DAGOZ_H

/**
 * Includes
 */
#include "mbed.h"

#define CMPS_DEFAULT_I2C_ADDRESS 0xC0
//set register kompas
#define SOFTWARE_REVISION_REG    0x0
#define COMPASS_BEARING_WORD_REG 0x2

//Inisiasi I2C komunikasi dari kompas
class CMPS_DAGOZ {

    I2C* i2c;
    int  i2cAddress;

public:

    /**
     * Constructor.
     *
     * @param sda mbed pin to use for I2C SDA
     * @param scl mbed pin to use for I2C SCL
     * @param address I2C address of this device.
     */
    CMPS_DAGOZ(PinName sda, PinName scl, int address);

    /**
     * Membaca revisi dari kompas
     */
    char readSoftwareRevision(void);

    /**
     * Membaca nilai dari kompas
     * Range nilai 0 - 3599
     */
    int readBearing(void);
};

#endif /* CMPS_DAGOZ_H */
