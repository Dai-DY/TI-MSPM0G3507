#include "mpu6050_port.h"

#include <stdbool.h>
#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"

#include "ti_msp_dl_config.h"
#include "tim_delay.h"

#define MPU6050_I2C_TIMEOUT_MS   10U
#define MPU6050_BUS_CLEAR_PULSES 9U

#define MPU6050_I2C_ERROR_INTERRUPTS                                      \
    (DL_I2C_INTERRUPT_CONTROLLER_NACK |                                  \
        DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST)

static bool timeout_expired(TickType_t start_tick)
{
    const TickType_t timeout_ticks =
        pdMS_TO_TICKS(MPU6050_I2C_TIMEOUT_MS);

    return (TickType_t) (xTaskGetTickCount() - start_tick) >= timeout_ticks;
}

static bool controller_has_error(void)
{
    return ((DL_I2C_getControllerStatus(I2C_0_INST) &
                DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) ||
        (DL_I2C_getRawInterruptStatus(
             I2C_0_INST, MPU6050_I2C_ERROR_INTERRUPTS) != 0U);
}

static bool wait_until_idle(void)
{
    const TickType_t start_tick = xTaskGetTickCount();

    while ((DL_I2C_getControllerStatus(I2C_0_INST) &
               DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (controller_has_error() || timeout_expired(start_tick)) {
            return false;
        }
    }

    return true;
}

static void prepare_transfer(void)
{
    DL_I2C_clearInterruptStatus(I2C_0_INST,
        MPU6050_I2C_ERROR_INTERRUPTS |
            DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
            DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);
}

static int transmit(uint8_t address, const uint8_t *data, uint16_t length)
{
    TickType_t start_tick;
    uint16_t transferred;

    if ((data == NULL) || (length == 0U)) {
        return -1;
    }

    if (!wait_until_idle()) {
        mpu6050_port_recover_bus();
        return -1;
    }

    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    prepare_transfer();
    transferred = (uint16_t) DL_I2C_fillControllerTXFIFO(
        I2C_0_INST, data, length);

    DL_I2C_startControllerTransfer(I2C_0_INST, address,
        DL_I2C_CONTROLLER_DIRECTION_TX, length);
    start_tick = xTaskGetTickCount();

    while (DL_I2C_getRawInterruptStatus(I2C_0_INST,
               DL_I2C_INTERRUPT_CONTROLLER_TX_DONE) == 0U) {
        if (controller_has_error() || timeout_expired(start_tick)) {
            mpu6050_port_recover_bus();
            return -1;
        }

        if (transferred < length) {
            transferred += (uint16_t) DL_I2C_fillControllerTXFIFO(
                I2C_0_INST, &data[transferred], length - transferred);
        }
    }

    if (controller_has_error() || (transferred != length)) {
        mpu6050_port_recover_bus();
        return -1;
    }

    return 0;
}

static int receive(uint8_t address, uint8_t *data, uint16_t length)
{
    TickType_t start_tick;
    uint16_t received = 0U;

    if ((data == NULL) || (length == 0U)) {
        return -1;
    }

    if (!wait_until_idle()) {
        mpu6050_port_recover_bus();
        return -1;
    }

    DL_I2C_flushControllerRXFIFO(I2C_0_INST);
    prepare_transfer();
    DL_I2C_startControllerTransfer(I2C_0_INST, address,
        DL_I2C_CONTROLLER_DIRECTION_RX, length);
    start_tick = xTaskGetTickCount();

    while (received < length) {
        while (!DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST) &&
            (received < length)) {
            data[received] = DL_I2C_receiveControllerData(I2C_0_INST);
            received++;
        }

        if (controller_has_error() || timeout_expired(start_tick)) {
            mpu6050_port_recover_bus();
            return -1;
        }

        if ((DL_I2C_getRawInterruptStatus(I2C_0_INST,
                 DL_I2C_INTERRUPT_CONTROLLER_RX_DONE) != 0U) &&
            (received < length) &&
            DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST)) {
            mpu6050_port_recover_bus();
            return -1;
        }
    }

    return 0;
}

int mpu6050_port_init(void)
{
    if (DL_I2C_getSDAStatus(I2C_0_INST) == DL_I2C_CONTROLLER_SDA_LOW) {
        mpu6050_port_recover_bus();
    }

    return (DL_I2C_getSDAStatus(I2C_0_INST) ==
               DL_I2C_CONTROLLER_SDA_LOW)
        ? -1
        : 0;
}

int mpu6050_port_write(uint8_t address, uint8_t reg, uint8_t length,
    const uint8_t *data)
{
    uint8_t buffer[17];
    uint8_t index;

    if ((length == 0U) || (length > 16U) || (data == NULL)) {
        return -1;
    }

    buffer[0] = reg;
    for (index = 0U; index < length; index++) {
        buffer[index + 1U] = data[index];
    }

    return transmit(address, buffer, (uint16_t) length + 1U);
}

int mpu6050_port_read(uint8_t address, uint8_t reg, uint8_t length,
    uint8_t *data)
{
    if ((length == 0U) || (data == NULL)) {
        return -1;
    }

    if (transmit(address, &reg, 1U) != 0) {
        return -1;
    }

    return receive(address, data, length);
}

void mpu6050_port_delay_ms(uint32_t milliseconds)
{
    if (milliseconds == 0U) {
        return;
    }

    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        while (milliseconds > 0U) {
            delay_us(1000U);
            milliseconds--;
        }
    } else {
        vTaskDelay(pdMS_TO_TICKS(milliseconds));
    }
}

void mpu6050_port_recover_bus(void)
{
    uint8_t pulse;

    DL_I2C_reset(I2C_0_INST);
    DL_I2C_enablePower(I2C_0_INST);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_GPIO_initDigitalInputFeatures(GPIO_I2C_0_IOMUX_SDA,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalOutput(GPIO_I2C_0_IOMUX_SCL);
    DL_GPIO_setPins(GPIO_I2C_0_SCL_PORT, GPIO_I2C_0_SCL_PIN);
    DL_GPIO_enableOutput(GPIO_I2C_0_SCL_PORT, GPIO_I2C_0_SCL_PIN);

    for (pulse = 0U; pulse < MPU6050_BUS_CLEAR_PULSES; pulse++) {
        if ((DL_GPIO_readPins(GPIO_I2C_0_SDA_PORT,
                 GPIO_I2C_0_SDA_PIN) &
                GPIO_I2C_0_SDA_PIN) != 0U) {
            break;
        }

        DL_GPIO_clearPins(GPIO_I2C_0_SCL_PORT, GPIO_I2C_0_SCL_PIN);
        delay_us(5U);
        DL_GPIO_setPins(GPIO_I2C_0_SCL_PORT, GPIO_I2C_0_SCL_PIN);
        delay_us(5U);
    }

    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_0_IOMUX_SDA,
        GPIO_I2C_0_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_0_IOMUX_SCL,
        GPIO_I2C_0_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_I2C_0_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_I2C_0_IOMUX_SCL);

    SYSCFG_DL_I2C_0_init();
}
