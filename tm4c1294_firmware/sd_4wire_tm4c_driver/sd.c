/*
 * sd.c
 *
 * Implements a 4 wire (proprietary mode) SD card driver which uses the Quad SPI, SPI,
 * and GPIO signals to communicate with an FPGA. The FPGA in turn will provide glue logic to
 * the SD Card and accelerate key computations.
 *
 * Essentially, the FPGA will serve as a pass through device that
 * will maintain as little state as possible, with its main responsibility being to accelerate the
 * "per lane" CRC16 calculations which are performed on each data lane individually.
 *
 *  Created on: Jan 3, 2021
 *      Author: Andy
 */

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"

#include "delay.h"
#include "sd.h"

//=============================================================================
// Constants

//=============================================================================
// Macros
#define CS_HIGH()               GPIOPinWrite(CMD_PORT, CMD_CS_PIN, CMD_CS_PIN)
#define CS_LOW()                GPIOPinWrite(CMD_PORT, CMD_CS_PIN, ~CMD_CS_PIN)
#define SEND_CMD_HIGH()         GPIOPinWrite(SIGNAL_PORT, SEND_CMD_PIN, SEND_CMD_PIN)
#define SEND_CMD_LOW()          GPIOPinWrite(SIGNAL_PORT, SEND_CMD_PIN, ~SEND_CMD_PIN)
#define DONE()                  GPIOPinRead(SIGNAL_PORT, DONE_PIN)

//=============================================================================
// Private Definitions
Resp1       resp_1;
Resp2       resp_2;
RespACMD41  resp_acmd41;
Resp6       resp_6;

uint16_t    RCA;
CID         cid_reg;
CardStatus  card_stat;

InitState   init_state;

//=============================================================================
// External Declarations
extern uint32_t MAIN_sys_clock;

extern volatile uint16_t resp_timer;
extern volatile uint16_t timer_1;

//=============================================================================
// Public Definitions

//=============================================================================
// Private Function Prototypes
void send_initial_clock_train();
void send_cmd(uint8_t cmd_idx, uint32_t arg);
uint32_t receive_resp(uint8_t cmd_idx, bool calc_crc, bool check_busy, uint8_t *buff, int len_bytes);
int crc7(uint8_t *buffer, int count);

#ifndef DEBUG_H
void test_pin_mappings(void);
void test_pin(uint32_t port, uint8_t pin_mask);
#endif
//=============================================================================
// Public Function Definitions

// Initialize the peripherals and GPIO necessary to communicate with the FPGA
void SD_init()
{
    // Enable the peripherals corresponding to the SPI and GPIO peripherals
    SysCtlPeripheralEnable(CMD_PORT_PERIPH);
    SysCtlPeripheralEnable(CMD_SSI_PERIPH);
    SysCtlPeripheralEnable(SIGNAL_PORT_PERIPH);

    // Wait for all enabled peripherals to be ready:
    while(!SysCtlPeripheralReady(CMD_PORT_PERIPH)){}
    while(!SysCtlPeripheralReady(CMD_SSI_PERIPH)){}
    while(!SysCtlPeripheralReady(SIGNAL_PORT_PERIPH)){}

    // Configure the pin multiplexing for CMD SPI bus
    GPIOPinConfigure(CMD_CLK_CFG);
    GPIOPinConfigure(CMD_MOSI_CFG);
    GPIOPinConfigure(CMD_MISO_CFG);

    // Configure the pin types for the CMD SPI bus:
    // 1. CLK, MOSI, and MISO are controlled by the SPI peripheral
    GPIOPinTypeSSI (CMD_PORT, CMD_CLK_PIN | CMD_MOSI_PIN | CMD_MISO_PIN);
    // 2. CS is controlled by software, and we want to ensure it starts out high
    CS_HIGH();
    GPIOPinTypeGPIOOutput(CMD_PORT, CMD_CS_PIN);

    // Configure the CMD bus SPI peripheral
    SSIConfigSetExpClk(CMD_SSI, MAIN_sys_clock, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 200000, 8);
    SSIEnable(CMD_SSI);

    // Enable the pins used by the Send Command and Done signals
    SEND_CMD_LOW();
    GPIOPinTypeGPIOOutput(SIGNAL_PORT, SEND_CMD_PIN);
    GPIOPinTypeGPIOInput(SIGNAL_PORT, DONE_PIN);
}

uint32_t SD_card_init(void)
{
    uint32_t stat = STAT_SUCCESS;
    int i;
    uint32_t in_byte;
    uint32_t arg;

    // Send an initial clock train of pulses to allow the SD card to initialize.
    // (Since the chip select lines are high, this will also reset the FPGA)
    init_state = STATE_CLOCK_TRAIN;
    send_initial_clock_train();

    // Reset the card
    init_state = STATE_RESET;
    send_cmd(CMD0, 0);

    // Send a dummy byte to provide the card a chance to process the reset
    SSIDataPut(CMD_SSI, 0);
    while(SSIBusy(CMD_SSI)){};
    SSIDataGet(CMD_SSI, &in_byte);

    // Send Interface Condition Command to verify the interface operating condition
    init_state = STATE_VERIFY_IFCOND;
    send_cmd(CMD8, ARG_CMD8);
    stat |= receive_resp(CMD8, true, false, resp_1.bytes, sizeof(resp_1));
    if (STAT_SUCCESS != stat)
        return stat;

    // Verify the voltage supplied and check pattern are echoed back exactly in the CMD8
    // response. Note that technically CMD8 has an R7 response, but treating it as an
    // R1 response is more convenient
    if (resp_1.fields.card_status != ARG_CMD8)
    {
        stat |= STAT_INVALID_CMD8_ECHO;
        return stat;
    }

    // Verify the operating conditions and wait for the card to initialize (or a 1s timeout to occur)
    init_state = STATE_INIT_LOOP;
    timer_1 = 100;
    while (timer_1 > 0)
    {
        // Send CMD55 to start the APP Specific Command sequence
        send_cmd(CMD55, 0);
        stat |= receive_resp(CMD55, true, false, resp_1.bytes, sizeof(Resp1));
        if (STAT_SUCCESS != stat)
            return stat;

        // Send CMD41 to complete the ACMD41 command
        send_cmd(CMD41, ARG_CMD41);

        // Note that the ACMD41 response does not use the CRC7 field, and the
        // cmd_idx field actually contains 0x3f (not 41)
        stat |= receive_resp(0x3f, false, false, resp_acmd41.bytes, sizeof(RespACMD41));
        if (STAT_SUCCESS != stat)
            return stat;

        // Check if we are still initializing, or if we are done
        // (Although it is counter-intuitive, busy=1 actually indicates initialization is done)
        if (1 == resp_acmd41.fields.busy)
            break;
    }

    if (0 == timer_1 && 0 == resp_acmd41.fields.busy)
    {
        stat |= STAT_ACMD41_TIMEOUT;
        return stat;
    }

    // Get the card identification (CID) information
    init_state = STATE_CARD_IDENTIFY;
    send_cmd(CMD2, 0);
    // Note, CMD 2 does not send the CMD index back in the response, and although
    // it does use a CRC7, this is only calculated over the contents of the internal register.
    // The CRC only covers the register contents and not the cmd index which precedes it.
    stat |= receive_resp(0x3F, false, false, resp_2.bytes, sizeof(Resp2));
    if (STAT_SUCCESS != stat)
        return stat;

     // Copy the contents of the R2 response to the CID register for easier field access
     // Note: resp_2 actually has 17 bytes, but we don't care about the MSB, since it
     // contains the start bit, transmission bit, and CMD index, and is not part of the register
     for (i=0; i<16; i++)
     {
         cid_reg.bytes[i] = resp_2.bytes[i];
     }

     // Request the card publishes its RCA (relative card address)
     init_state = STATE_PUB_ADDRESS;
     send_cmd(CMD3, 0);
     stat |= receive_resp(CMD3, true, false, resp_6.bytes, sizeof(Resp6));
     if (STAT_SUCCESS != stat)
         return stat;

     // Store the RCA for convenience in accessing it later
     // TODO: extract status information
     RCA = resp_6.fields.new_pub_rca;

     // Using the new RCA, address the card using CMD7
     arg = RCA << 16;
     send_cmd(CMD7, arg);
     stat |= receive_resp(CMD7, true, true, resp_1.bytes, sizeof(Resp1));
     if (STAT_SUCCESS != stat)
         return stat;
     card_stat.u32 = resp_1.fields.card_status;
     // Todo: use status info

     // Now that the card is addressed, put in 4bit Bus mode using ACMD6
    arg = RCA << 16;
    send_cmd(CMD55, arg);
    stat |= receive_resp(CMD55, true, false, resp_1.bytes, sizeof(Resp1));
    if (STAT_SUCCESS != stat)
        return stat;

    send_cmd(CMD6, 0x00000002);
    stat |= receive_resp(CMD6, true, false, resp_1.bytes, sizeof(Resp1));
    if (STAT_SUCCESS != stat)
        return stat;
    card_stat.u32 = resp_1.fields.card_status;

    return stat;
}

//=============================================================================
// Private Function Definitions

void send_initial_clock_train()
{
    int i;
    uint32_t byte_in;

    // Ensure the CS is high since we don't want to send or receive any data and only
    // care about the clock generation here:
    CS_HIGH();

    // Send a total of 2048 transitions (since this is what was observed when analyzing
    // actual traffic from a windows SD host controller).  These transitions will be
    // used to initialize the SD card.

    for (i=0; i<256; i++)
    {
        // Send a series of dummy bytes
        SSIDataPut(CMD_SSI, 0);
        while(SSIBusy(CMD_SSI)){};
    }

    // Flush all bytes received from the receive FIFO
    while(SSIDataGetNonBlocking(CMD_SSI, &byte_in));
}

void send_cmd(uint8_t cmd_idx, uint32_t arg)
{
    int i;
    static Command cmd;
    uint32_t byte_in;

    // Pack the command index and argument into into the command structure
    // Note that the start bit, end bit, and CRC are controlled by the FPGA, so
    // they are "don't cares" and we can leave them at their default value of 0.
    cmd.fields.trans_bit = 1;
    cmd.fields.cmd_index = cmd_idx;
    cmd.fields.arg = arg;

    // Begin the command
    SEND_CMD_HIGH();
    CS_LOW();

    // Add the start bit, transmission bit, command index, and argument portion
    // of the command (the first 5 bytes) to the 8 level FIFO
    for (i=5; i>=1; i--)
    {
        SSIDataPut(CMD_SSI, cmd.bytes[i]);
    }

    // Add a 6th dummy byte to the transmission, during which time the FPGA will clock
    // out the calculated CRC7 value
    SSIDataPut(CMD_SSI, 0);

    // Wait for the command bytes to sent
    while(SSIBusy(CMD_SSI)){};

    // Flush all bytes received from the receive FIFO
    while(SSIDataGetNonBlocking(CMD_SSI, &byte_in));

    // End the command transaction
    CS_HIGH();
    SEND_CMD_LOW();
}

uint32_t receive_resp(uint8_t cmd_idx, bool calc_crc, bool check_busy, uint8_t *buff, int len_bytes)
{
    uint32_t stat = STAT_SUCCESS;
    int i;
    int byte_index;
    uint32_t byte_in;
    uint8_t MSB;
    uint8_t LSB;
    uint8_t crc_received;
    uint8_t crc_calc;

    // Ensure that the send command line remains low through the response reception
    SEND_CMD_LOW();

    byte_index = len_bytes - 1;

    // Zero the response buffer
    for (i=0; i<len_bytes; i++)
    {
        buff[i] = 0;
    }

    // Flush all bytes received from the receive FIFO
    while(SSIDataGetNonBlocking(CMD_SSI, &byte_in));

    // Switch the CMD CLK pin to a manually controlled GPIO output and switch the
    // CMD MISO pin to be a GPIO input
    GPIOPinWrite(CMD_PORT, CMD_CLK_PIN, ~CMD_CLK_PIN);
    GPIOPinTypeGPIOOutput(CMD_PORT, CMD_CLK_PIN);
    GPIOPinTypeGPIOInput(CMD_PORT, CMD_MISO_PIN);

    // Send clock pulses on the CMD CLK bin and observe the CMD MOSI pin until either:
    // 1) a Start bit (0) is observed
    // 2) a timeout occurs: TODO: Review timeout duration
    resp_timer = 10;
    while (resp_timer > 0)
    {
        // TODO: parameterize the timing so we can go faster
        DELAY_US(2);

        if (!GPIOPinRead(CMD_PORT, CMD_MISO_PIN))
        {
            break;
        }

        GPIOPinWrite(CMD_PORT, CMD_CLK_PIN, CMD_CLK_PIN);
        DELAY_US(2);
        GPIOPinWrite(CMD_PORT, CMD_CLK_PIN, ~CMD_CLK_PIN);
    }

    // Re-enable the SPI peripherals control over the CMD Clk and CMD MISO pins:
    GPIOPinTypeSSI(CMD_PORT, CMD_CLK_PIN | CMD_MISO_PIN);

    CS_LOW();

    // Exit if a timeout occurred while waiting
    if (0 == resp_timer)
    {
        return STAT_TIMEOUT;
    }

    // Read all bytes, starting with the start bit
    while (byte_index >= 0)
    {
        SSIDataPut(CMD_SSI, 0);
        SSIDataGet(CMD_SSI, &byte_in);
        buff[byte_index--] = (uint8_t) byte_in;
    }

    // End the SPI transaction
    CS_HIGH();

    // Send a dummy byte so that the SD card has 8 additional clock cycles to
    // finish its operations. During this time (whenever a clock is received by the FPGA and CS is HIGH)
    // the FPGA Send Command registers will also be reset in preparation for the next command or response.
    SSIDataPut(CMD_SSI, 0);

    // The SSIDataPut operation is non-blocking, so while it is sending the dummy byte, we
    // will proceed to perform validation of the response. We will then wait for the transmission to complete
    // after we are done with our validations.

    // Obtain the first and last bytes, used for validating the cmd_index,
    // crc, trans bit, and end bit.
    LSB = buff[0];
    MSB = buff[len_bytes - 1];

    // Verify the transmission bit indicates a "card to host" transaction
    if (0 != (MSB & 0x40))
    {
        stat |= STAT_INVALID_TRANS;
    }

    // Verify the End bit is 1
    if (LSB & 0x1 == 0)
    {
        stat |= STAT_INVALID_END_BIT;
    }

    // Verify the command index was received correctly in the response
    if ((MSB & 0x3f) != cmd_idx)
    {
        stat |= STAT_INVALID_CMDIDX;
    }

    // Verify the CRC7 is correct, if requested by the caller
    // TODO: Offload the CRC calculation to the FPGA
    if (calc_crc)
    {
        // Get the CRC from bits 1-7 of the LSB
        crc_received = LSB >> 1;

        // Calculate the CRC7 from all other bytes in the buffer
        crc_calc =crc7(buff+1, len_bytes-1);

        // Verify the calculated and received CRC's match
        if (crc_received != crc_calc)
        {
            stat |= STAT_INVALID_CRC;
        }
    }

    // Wait for the dummy byte clock cycles to be completed, and discard
    // the received byte from the FIFO. As noted, we
    // did some other calculations while waiting to make better use of this time.
    while(SSIBusy(CMD_SSI)){};
    SSIDataGet(CMD_SSI, &byte_in);

    // TODO: Add a wait on the busy signal (D0 held low) for R1b commands

    return stat;
}

int crc7(uint8_t *buffer, int count)
{
    int i, c, crc, idx;
    crc = 0;
    for (idx=count-1; idx >= 0; idx--)
    {
        c = buffer[idx];
        for (i = 0; i < 8; i++)
        {
            crc <<= 1;
            if ((c ^ crc) & 0x80) crc ^= 0x9;
            c <<= 1;
        }
    }
    return crc & 0x7f;
}

// Code to test the pin mappings:
#ifndef DEBUG_H

void test_pin(uint32_t port, uint8_t pin_mask)
{
    GPIOPinWrite(port, pin_mask, pin_mask);
    DELAY_US(2);
    GPIOPinWrite(port, pin_mask, ~pin_mask);
    DELAY_US(2);
    GPIOPinWrite(port, pin_mask, pin_mask);
    DELAY_US(2);
    GPIOPinWrite(port, pin_mask, ~pin_mask);
    DELAY_US(2);
}

void test_pin_mappings(void)
{
    SysCtlPeripheralEnable(CMD_PORT_PERIPH);
    SysCtlPeripheralEnable(SIGNAL_PORT_PERIPH);

    while(!SysCtlPeripheralReady(CMD_PORT_PERIPH)){}
    while(!SysCtlPeripheralReady(SIGNAL_PORT_PERIPH)){}

    GPIOPinTypeGPIOOutput(CMD_PORT, CMD_CLK_PIN | CMD_CS_PIN | CMD_MOSI_PIN | CMD_MISO_PIN);
    GPIOPinTypeGPIOOutput(SIGNAL_PORT, SEND_CMD_PIN | DONE_PIN);

    test_pin(CMD_PORT, CMD_CLK_PIN);
    DELAY_US(10);

    test_pin(CMD_PORT, CMD_CS_PIN);
    DELAY_US(10);

    test_pin(CMD_PORT, CMD_MOSI_PIN);
    DELAY_US(10);

    test_pin(CMD_PORT, CMD_MISO_PIN);
    DELAY_US(10);

    test_pin(SIGNAL_PORT, SEND_CMD_PIN);
    DELAY_US(10);

    test_pin(SIGNAL_PORT, DONE_PIN);
    DELAY_US(10);

    GPIOPinTypeGPIOInput(CMD_PORT, CMD_CLK_PIN | CMD_CS_PIN | CMD_MOSI_PIN | CMD_MISO_PIN);
    GPIOPinTypeGPIOInput(SIGNAL_PORT, SEND_CMD_PIN | DONE_PIN);
}
#endif // END #ifndef DEBUG_H
