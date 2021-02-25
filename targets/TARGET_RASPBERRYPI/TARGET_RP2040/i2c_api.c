/******************************************************************************
 * INCLUDE
 ******************************************************************************/

#include "mbed_assert.h"
#include "i2c_api.h"
#include "pinmap.h"
#include "PeripheralPins.h"

/******************************************************************************
 * CONST
 ******************************************************************************/

static unsigned int const DEFAULT_I2C_BAUDRATE = 100 * 1000; /* 100 kHz */

/******************************************************************************
 * FUNCTION DEFINITION
 ******************************************************************************/

void i2c_init(i2c_t *obj, PinName sda, PinName scl)
{
    /* Verify if both pins belong to the same I2C peripheral. */
    I2CName const i2c_sda = (I2CName)pinmap_peripheral(sda, PinMap_I2C_SDA);
    I2CName const i2c_scl = (I2CName)pinmap_peripheral(scl, PinMap_I2C_SCL);
    MBED_ASSERT(i2c_sda == i2c_scl);

#if DEVICE_I2CSLAVE
    /** was_slave is used to decide which driver call we need
     * to use when uninitializing a given instance
     */
    obj->was_slave = false;
    obj->is_slave = false;
    obj->slave_addr = 0;
#endif

    /* Obtain the pointer to the I2C hardware instance. */
    obj->dev = (i2c_inst_t *)pinmap_function(sda, PinMap_I2C_SDA);
    //obj->baudrate = DEFAULT_I2C_BAUDRATE;
    //Call this function because if we are configuring a slave, we don't have to set the frequency
    i2c_frequency(obj->dev, DEFAULT_I2C_BAUDRATE);

    /* Initialize the I2C module. */
    _i2c_init(obj->dev, obj->baudrate);

    /* Configure GPIO for I2C as alternate function. */
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);

    /* Enable pull-ups for I2C pins. */
    gpio_pull_up(sda);
    gpio_pull_up(scl);
}

void i2c_frequency(i2c_t *obj, int hz)
{
#if DEVICE_I2CSLAVE
    /* Slaves automatically get frequency from master */
    if(obj->is_slave) {
    		return;
    }
#endif
    obj->baudrate = i2c_set_baudrate(obj->dev, hz);
}

int i2c_read(i2c_t *obj, int address, char *data, int length, int stop)
{
    int const bytes_read = i2c_read_blocking(obj->dev,
                                             (uint8_t)address,
                                             (uint8_t *)data,
                                             (size_t)length,
                                             /* nostop = */(stop == 0));
    if (bytes_read < 0)
        return I2C_ERROR_NO_SLAVE;
    else
        return bytes_read;
}

int i2c_write(i2c_t *obj, int address, const char *data, int length, int stop)
{
    int const bytes_written = i2c_write_blocking(obj->dev,
                                                 address,
                                                 (const uint8_t *)data,
                                                 (size_t)length,
                                                 /* nostop = */(stop == 0));
    if (bytes_written < 0)
        return I2C_ERROR_NO_SLAVE;
    else
        return bytes_written;
}

void i2c_reset(i2c_t *obj)
{
    i2c_deinit(obj->dev);
    _i2c_init(obj->dev, obj->baudrate);
}

const PinMap *i2c_master_sda_pinmap()
{
    return PinMap_I2C_SDA;
}

const PinMap *i2c_master_scl_pinmap()
{
    return PinMap_I2C_SCL;
}

const PinMap *i2c_slave_sda_pinmap()
{
    return PinMap_I2C_SDA;
}

const PinMap *i2c_slave_scl_pinmap()
{
    return PinMap_I2C_SCL;
}

int i2c_stop(i2c_t *obj)
{

}

#if DEVICE_I2CSLAVE

/** Configure I2C as slave or master.
 *  @param obj The I2C object
 *  @param enable_slave Enable i2c hardware so you can receive events with ::i2c_slave_receive
 *  @return non-zero if a value is available
 */
void i2c_slave_mode(i2c_t *obj, int enable_slave)
{
    /* Reconfigure this instance as an I2C slave */

    /* Not configured yet, so this instance is and was a slave */
    if(obj->dev == NULL) {
    		obj->was_slave = enable_slave;
    } else {
        obj->was_slave = obj->is_slave;
    }
    obj->is_slave = enable_slave;
}

/** Check to see if the I2C slave has been addressed.
 *  @param obj The I2C object
 *  @return The status - 1 - read addresses, 2 - write to all slaves,
 *         3 write addressed, 0 - the slave has not been addressed
 */
int i2c_slave_receive(i2c_t *obj)
{

}

/** Configure I2C as slave or master.
 *  @param obj The I2C object
 *  @param data    The buffer for receiving
 *  @param length  Number of bytes to read
 *  @return non-zero if a value is available
 */
int i2c_slave_read(i2c_t *obj, char *data, int length)
{
    i2c_read_raw_blocking(obj->dev, (uint8_t *)data, (size_t)length);

    return 1;
}

/** Configure I2C as slave or master.
 *  @param obj The I2C object
 *  @param data    The buffer for sending
 *  @param length  Number of bytes to write
 *  @return non-zero if a value is available
 */
int i2c_slave_write(i2c_t *obj, const char *data, int length)
{
    //Check if raw is okay or use write_blocking
    i2c_write_raw_blocking(obj->dev, (const uint8_t *)data, (size_t)length);

    return 1;
}

/** Configure I2C address.
 *  @param obj     The I2C object
 *  @param idx     Currently not used
 *  @param address The address to be set
 *  @param mask    Currently not used
 */
void i2c_slave_address(i2c_t *obj, int idx, uint32_t address, uint32_t mask)
{
    if (obj->is_slave) {
        i2c_set_slave_mode(obj->dev, true, address);
    }
}

#endif // DEVICE_I2CSLAVE
