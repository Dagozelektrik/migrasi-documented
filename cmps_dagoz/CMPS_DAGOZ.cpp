/**
 * adopted from
 * CMPS03 by: Aarom Berk
 * 
 * Bismillahirahmanirrahim
 */

/**
 * Includes
 */
#include "CMPS_DAGOZ.h"

CMPS_DAGOZ::CMPS_DAGOZ(PinName sda, PinName scl, int address) {
    i2c = new I2C(sda, scl);
    //CMPS11 maksimum 100kHz CMPS12 maksimum 400kHz
    i2c->frequency(400000);
    i2cAddress = address;

}

char CMPS_DAGOZ::readSoftwareRevision(void){
    char registerNumber   = SOFTWARE_REVISION_REG;
    char registerContents = 0;

    //First, send the number of register we wish to read,
    //in this case, command register, number 0.
    i2c->write(i2cAddress, &registerNumber, 1, false);
    
    //Now, read one byte, which will be the contents of the command register.
    i2c->read(i2cAddress, &registerContents, 1, false);
    
    return registerContents; 
}

int CMPS_DAGOZ::readBearing(void){
    char registerNumber = COMPASS_BEARING_WORD_REG;
    char registerContents[2] = {0x00, 0x00};
    
    //Mengirim register untuk address pertama dari i2c
    i2c->write(i2cAddress, &registerNumber, 1, false);
    
    //Mengambil data dari 2 address I2c
    i2c->read(i2cAddress, registerContents, 2, false);
    
    //Register 0 adalah 8 bit, harus di shift
    //Register 1 adalah 16 bit, bisa langsung dibaca
    int bearing = ((int)registerContents[0] << 8) | ((int)registerContents[1]);
    
    return bearing;   
}
