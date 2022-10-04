/*!
 * \file      sx1262mbxcas-board.c
 *
 * \brief     Target board SX1262MBXCAS shield driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include <stdlib.h>
#include <stdio.h>
#include "radio.h"
#include "sx126x-board.h"
#include "gpio-board.h"
#include <stdint.h>     /* C99 types */
#include <stdio.h>      /* printf fprintf */
#include <stdlib.h>     /* malloc free */
#include <unistd.h>     /* lseek, close */
#include <fcntl.h>      /* open */
#include <string.h>     /* memset */
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

static int Sx1268Fd;
extern int Sx126xBusyPin;
extern int Sx126xResetPin;

void SX126xSpiInit( char *path )
{
    int a=0, b=0;
    int i;
    
	Sx1268Fd = open(path, O_RDWR);
    if (Sx1268Fd < 0) {
        printf("Sx1268Fd Error\n");
    }
    /* setting SPI mode to 'mode 0' */
    i = SPI_MODE_0;
    a = ioctl(Sx1268Fd, SPI_IOC_WR_MODE, &i);
    b = ioctl(Sx1268Fd, SPI_IOC_RD_MODE, &i);
    if ((a < 0) || (b < 0)) {  
        printf("SPI_MODE_0 Error\n");
        close(Sx1268Fd);
    }

    /* setting SPI max clk (in Hz) */
    i = 1000000;
    a = ioctl(Sx1268Fd, SPI_IOC_WR_MAX_SPEED_HZ, &i);
    b = ioctl(Sx1268Fd, SPI_IOC_RD_MAX_SPEED_HZ, &i);
    if ((a < 0) || (b < 0)) {
        printf("SPEED Error\n");
        close(Sx1268Fd);
      
    }

    /* setting SPI to MSB first */
    i = 0;
    a = ioctl(Sx1268Fd, SPI_IOC_WR_LSB_FIRST, &i);
    b = ioctl(Sx1268Fd, SPI_IOC_RD_LSB_FIRST, &i);
    if ((a < 0) || (b < 0)) {
        
        printf("MSB Error\n");
        close(Sx1268Fd);
       
    }

    /* setting SPI to 8 bits per word */
    i = 0;
    a = ioctl(Sx1268Fd, SPI_IOC_WR_BITS_PER_WORD, &i);
    b = ioctl(Sx1268Fd, SPI_IOC_RD_BITS_PER_WORD, &i);
    if ((a < 0) || (b < 0)) {
        
        printf("BIT Error\n");
        close(Sx1268Fd);     
    }
	//printf("sx126x spi init success!\r\n");
}

void SX126xSpiDeInit( void )
{
    close(Sx1268Fd);
}

void SX126xIoTcxoInit( void )
{   
    // No TCXO component available on this board design.
    CalibrationParams_t calibParam;
    SX126xSetDio3AsTcxoCtrl( TCXO_CTRL_1_7V, SX126xGetBoardTcxoWakeupTime( ) << 6 ); // convert from ms to SX126x time base    
    calibParam.Value = 0x7F;
    SX126xCalibrate( calibParam );
}

uint32_t SX126xGetBoardTcxoWakeupTime( void )
{
    return 5;
}

void SX126xReset( void )
{
    if(Sx126xBusyPin != 0xff)
    {
        gpio_export(Sx126xResetPin);
        gpio_direction(Sx126xResetPin,OUT);
        gpio_write(Sx126xResetPin,0);
        usleep(200000);
        gpio_write(Sx126xResetPin,1);
    }
}

void SX126xWaitOnBusy( void )
{
    if(Sx126xBusyPin != 0xff)
    {
        gpio_export(Sx126xBusyPin);
        gpio_direction(Sx126xBusyPin,IN);
        while(gpio_read(Sx126xBusyPin) == 1);
        gpio_unexport(Sx126xBusyPin);
    }
}

void SX126xWakeup( void)
{
    uint8_t  out_buffer[2] = {0};
    struct   spi_ioc_transfer k;
    int      i = 0;
    int      a = 0;
    int      command_size;
    
    out_buffer[0] = (uint8_t)RADIO_GET_STATUS;
    out_buffer[1] = 0x00;
    command_size = 2;
    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k.tx_buf = (unsigned long) out_buffer;
    k.len = command_size;
    k.cs_change = 0;
    a = ioctl(Sx1268Fd, SPI_IOC_MESSAGE(1), &k);
    if (a != (int)k.len) 
    {
        printf("ERROR: SPI READ FAILURE\n");
    } 
    SX126xWaitOnBusy( );

    return 0;
}

void SX126xWriteCommand(RadioCommands_t command, uint8_t *buffer, uint16_t size )
{    
    uint8_t  out_buffer[256] = {0};
    struct   spi_ioc_transfer k;
    int      i = 0;
    int      a = 0;
    int      command_size;
    
    SX126xCheckDeviceReady( );
    
    out_buffer[0] = (uint8_t)command;
    for(i=0; i<size; i++)
    {
        out_buffer[1+i] = buffer[i];
    }
    command_size = 1+size;
    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k.tx_buf = (unsigned long) out_buffer;
    k.len = command_size;
    k.cs_change = 0;
    a = ioctl(Sx1268Fd, SPI_IOC_MESSAGE(1), &k);
    if (a != (int)k.len) 
    {
        printf("ERROR: SPI READ FAILURE\n");
    } 
    if( command != RADIO_SET_SLEEP )
    {
        SX126xWaitOnBusy( );
    }

    return 0;
    
}

uint8_t SX126xReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{    
    uint8_t  out_buffer[256] = {0};
    uint8_t  in_buffer[256] = {0};
    struct   spi_ioc_transfer k;
    int      i = 0;
    int      a = 0;
    int      command_size;
    
    SX126xCheckDeviceReady( );
    
    out_buffer[0] = (uint8_t)command;
    out_buffer[1] = 0x00;
    for(i=0; i<size; i++)
    {
        out_buffer[2+i] = 0x00;
    }
    command_size = 2 + size;
    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k.tx_buf = (unsigned long) out_buffer;
    k.rx_buf = (unsigned long) in_buffer;
    k.len = command_size;
    k.cs_change = 0;
    a = ioctl(Sx1268Fd, SPI_IOC_MESSAGE(1), &k);
    if (a != (int)k.len) 
    {
        printf("ERROR: SPI READ FAILURE\n");
    } 
    else 
    {
        for(i=0; i<size; i++)
        {
            buffer[i] = in_buffer[command_size - 1 - i];
        }
    }
    SX126xWaitOnBusy( );

    return 0;
}

void SX126xWriteRegisters(uint16_t address, uint8_t *buffer, uint16_t size )
{
    uint8_t  AddrHigh = (uint8_t)(( address & 0xFF00 ) >> 8 );
    uint8_t  AddrLow = (uint8_t)(address & 0x00FF);
    uint8_t  out_buffer[256] = {0};
    struct   spi_ioc_transfer k;
    int      i = 0;
    int      a = 0;
    int      command_size;
    
    SX126xCheckDeviceReady( );
    
    out_buffer[0] = (uint8_t)RADIO_WRITE_REGISTER;
    out_buffer[1] = AddrHigh;
    out_buffer[2] = AddrLow;
    for(i=0; i<size; i++)
    {
        out_buffer[3+i] = buffer[i];
    }
    command_size = 3 + size;
    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k.tx_buf = (unsigned long) out_buffer;
    k.len = command_size;
    k.cs_change = 0;
    a = ioctl(Sx1268Fd, SPI_IOC_MESSAGE(1), &k);
    if (a != (int)k.len) 
    {
        printf("ERROR: SPI READ FAILURE\n");
    } 
    SX126xWaitOnBusy( );
    
}

void SX126xWriteRegister( uint16_t address, uint8_t value )
{
    SX126xWriteRegisters(address, &value, 1 );
}

void SX126xReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    uint8_t  AddrHigh = (uint8_t)(( address & 0xFF00 ) >> 8 );
    uint8_t  AddrLow = (uint8_t)(address & 0x00FF);
    uint8_t  out_buffer[256] = {0};
    uint8_t  in_buffer[256] = {0};
    struct   spi_ioc_transfer k;
    int      i = 0;
    int      a = 0;
    int      command_size;
    
    SX126xCheckDeviceReady( );
    
    out_buffer[0] = (uint8_t)RADIO_READ_REGISTER;
    out_buffer[1] = AddrHigh;
    out_buffer[2] = AddrLow;
    out_buffer[3] = 0x00;
    for(i=0; i<size; i++)
    {
        out_buffer[4+i] = 0x00;
    }
    command_size = 4 + size;
    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k.tx_buf = (unsigned long) out_buffer;
    k.rx_buf = (unsigned long) in_buffer;
    k.len = command_size;
    k.cs_change = 0;
    a = ioctl(Sx1268Fd, SPI_IOC_MESSAGE(1), &k);
    if (a != (int)k.len) 
    {
        printf("ERROR: SPI READ FAILURE\n");
    } 
    else 
    {
        for(i=0; i<size; i++)
        {
            buffer[i] = in_buffer[command_size - 1 - i];
        }
    }
    SX126xWaitOnBusy( );
    
}

uint8_t SX126xReadRegister( uint16_t address )
{
    uint8_t data;
    SX126xReadRegisters( address, &data, 1 );
    return data;
}

void SX126xWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    uint8_t  out_buffer[256] = {0};
    struct   spi_ioc_transfer k;
    int      i = 0;
    int      a = 0;
    int      command_size;
    
    SX126xCheckDeviceReady( );
    
    out_buffer[0] = (uint8_t)RADIO_WRITE_BUFFER;
    out_buffer[1] = (uint8_t)offset;
    for(i=0; i<size; i++)
    {
        out_buffer[2+i] = buffer[i];
    }
    command_size = 2+size;
    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k.tx_buf = (unsigned long) out_buffer;
    k.len = command_size;
    k.cs_change = 0;
    a = ioctl(Sx1268Fd, SPI_IOC_MESSAGE(1), &k);
    if (a != (int)k.len) 
    {
        printf("ERROR: SPI READ FAILURE\n");
    } 
    SX126xWaitOnBusy( );

    return 0;
}

void SX126xReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    uint8_t  out_buffer[256] = {0};
    uint8_t  in_buffer[256] = {0};
    struct   spi_ioc_transfer k;
    int      i = 0;
    int      a = 0;
    int      command_size;
    
    SX126xCheckDeviceReady( );
    
    out_buffer[0] = (uint8_t)RADIO_READ_BUFFER;
    out_buffer[1] = offset;
    out_buffer[2] = 0x00;
    for(i=0; i<size; i++)
    {
        out_buffer[3+i] = 0x00;
    }
    command_size = 3 + size;
    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k.tx_buf = (unsigned long) out_buffer;
    k.rx_buf = (unsigned long) in_buffer;
    k.len = command_size;
    k.cs_change = 0;
    a = ioctl(Sx1268Fd, SPI_IOC_MESSAGE(1), &k);
    if (a != (int)k.len) 
    {
        printf("ERROR: SPI READ FAILURE\n");
    } 
    else 
    {
        for(i=0; i<size; i++)
        {
            buffer[i] = in_buffer[command_size - 1 - i];
        }
    }
    SX126xWaitOnBusy( );
}

void SX126xSetRfTxPower( int8_t power )
{
    SX126xSetTxParams( power, RADIO_RAMP_40_US );
}

uint8_t SX126xGetDeviceId( void )
{
    return SX1262;
}

void SX126xAntSwOn( void )
{
    
}

void SX126xAntSwOff( void )
{

}

bool SX126xCheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}

#if defined( USE_RADIO_DEBUG )
void SX126xDbgPinTxWrite( uint8_t state )
{
    
}

void SX126xDbgPinRxWrite( uint8_t state )
{
    
}
#endif
