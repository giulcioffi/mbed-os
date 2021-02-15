/******************************************************************************
 * INCLUDE
 ******************************************************************************/

#include "mbed_assert.h"
#include "mbed_critical.h"
#include "spi_api.h"
#include "pinmap.h"
#include "PeripheralPins.h"

/******************************************************************************
 * CONSTANT
 ******************************************************************************/

static unsigned int const SPI_MASTER_DEFAULT_BITRATE = 1000 * 1000; /* 1 MHz */

/******************************************************************************
 * FUNCTION DEFINITION
 ******************************************************************************/

void spi_init(spi_t *obj, PinName mosi, PinName miso, PinName sclk, PinName ssel)
{
    /* Obtain the pointer to the SPI hardware instance. */
    spi_inst_t * dev_mosi = (spi_inst_t *)pinmap_peripheral(mosi, PinMap_SPI_MOSI);
    spi_inst_t * dev_miso = (spi_inst_t *)pinmap_peripheral(miso, PinMap_SPI_MISO);
    spi_inst_t * dev_sclk = (spi_inst_t *)pinmap_peripheral(sclk, PinMap_SPI_SCLK);
    spi_inst_t * dev_ssel = (spi_inst_t *)pinmap_peripheral(ssel, PinMap_SPI_SSEL);

    /* Check if in fact all pins point to the same SPI hardware instance. */
    MBED_ASSERT(dev_mosi == dev_miso);
    MBED_ASSERT(dev_miso == dev_sclk);
    MBED_ASSERT(dev_sclk == dev_ssel);

    /* Now that we know that all pins use the same SPI module we can save it. */
    obj->dev = dev_mosi;

    /* Configure GPIOs for SPI usage. */
    gpio_set_function(mosi, GPIO_FUNC_SPI);
    gpio_set_function(sclk, GPIO_FUNC_SPI);
    gpio_set_function(miso, GPIO_FUNC_SPI);

    /* Initialize SPI at 1 MHz bitrate */
    _spi_init(obj->dev, SPI_MASTER_DEFAULT_BITRATE);
}

void spi_format(spi_t *obj, int bits, int mode, int slave)
{
    /* Doing some parameter sanity checks. */
    MBED_ASSERT((bits >= 4) && (bits <= 16));
    MBED_ASSERT((mode >= 0) && (mode <= 3));

    /* Determine parameters for CPOL, CPHA */
    spi_cpol_t cpol;
    spi_cpha_t cpha;
    if        (mode == 0) {
        cpol = SPI_CPOL_0;
        cpha = SPI_CPHA_0;
    } else if (mode == 1) {
        cpol = SPI_CPOL_0;
        cpha = SPI_CPHA_1;
    } else if (mode == 2) {
        cpol = SPI_CPOL_1;
        cpha = SPI_CPHA_0;
    } else {
        cpol = SPI_CPOL_1;
        cpha = SPI_CPHA_1;
    }
    /* Configure the SPI. */
    spi_set_format(obj->dev, bits, cpol, cpha, SPI_MSB_FIRST);
    /* Set's the SPI up as slave if the value of slave is different from 0, e.g. a value of 1 or -1 set's this SPI up as a slave. */
    spi_set_slave(obj->dev, slave != 0);
}

void spi_frequency(spi_t *obj, int hz)
{
    spi_set_baudrate(obj->dev, hz);
}

int spi_master_write(spi_t *obj, int value)
{
    uint8_t rx;
    uint8_t const tx = (uint8_t)value;
    spi_master_block_write(obj, (const char *)&tx, sizeof(tx), (char *)&rx, sizeof(rx), ' ');
    return rx;
}

int spi_master_block_write(spi_t *obj, const char *tx_buffer, int tx_length, char *rx_buffer, int rx_length, char write_fill)
{
    /* The pico-sdk API does not support different length SPI buffers. */
    MBED_ASSERT(tx_length == rx_length);
    /* Perform the SPI transfer. */
    return spi_write_read_blocking(obj->dev, (const uint8_t *)tx_buffer, (uint8_t *)rx_buffer, (size_t)tx_length);
}

const PinMap *spi_master_mosi_pinmap()
{
    return PinMap_SPI_MOSI;
}

const PinMap *spi_master_miso_pinmap()
{
    return PinMap_SPI_MISO;
}

const PinMap *spi_master_clk_pinmap()
{
    return PinMap_SPI_SCLK;
}

const PinMap *spi_master_cs_pinmap()
{
    return PinMap_SPI_SSEL;
}
