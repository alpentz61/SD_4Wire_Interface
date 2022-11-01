
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "delay.h"
#include "led.h"
#include "inc/tm4c1294ncpdt.h"

// Global definitions
uint32_t MAIN_sys_clock;

volatile uint16_t resp_timer = 0;
volatile uint16_t timer_1 = 0;


// Command Indices
#define CMD0        0
#define CMD2        2
#define CMD3        3
#define CMD6        6
#define CMD7        7   // Select (address) a card so that data transfers can be performed
#define CMD8        8
#define CMD17       17  // Single block read
#define CMD41       41
#define CMD55       55

// Command arguments
#define ARG_CMD8     0x000001AA
#define ARG_CMD41    0x513C0000

// General Constants
#define BLOCK_LEN   512

// Enclose all data structures in this to ensure tight bit field packing
#pragma pack(push, 1)

// Command / Response data structures.  These are defined in the reverse order, since the TM4C processors
// are little endian, and the SD shifts data MSB first.  By putting them in reverse order and
// then shifting data in (or out) from the end of the structure backwards, we can easily
// ensure that adjacent fields are in the proper bit order

typedef union
{
    struct
    {
        unsigned int end_bit : 1;
        unsigned int crc7 : 7;
        uint32_t arg;
        unsigned int cmd_index : 6;
        unsigned int trans_bit : 1;
        unsigned int start_bit : 1;
    } fields;
    uint8_t bytes[6];
} Command;

// Response Format R1: Used by both R1 and R1b commands.
// 48 bits total length
// R1b commands have an identical field format, but also have a busy signal on the data line 0
typedef union
{
    struct
    {
        unsigned int end_bit : 1;
        unsigned int crc7 : 7;
        uint32_t card_status;
        unsigned int cmd_index : 6;
        unsigned int trans_bit : 1;
        unsigned int start_bit : 1;
    } fields;
    uint8_t bytes[6];
}Resp1;

// Response Format R2: Used by read CID and read CSD commands
// 136 bits total length
typedef union
{
    struct
    {
        unsigned int end_bit : 1;
        unsigned int crc7 : 7;
        uint8_t reg_data[15];
        unsigned int cmd_index : 6;
        unsigned int trans_bit : 1;
        unsigned int start_bit : 1;
    } fields;
    uint8_t bytes[17];
}Resp2;

// Response format for ACMD41 (Specialization of R3)
// 48 bits total length
typedef union
{
    struct
    {
        unsigned int end_bit : 1;
        unsigned int crc7 : 7;
        uint8_t reserved1;
        uint16_t ocr;
        unsigned s18A : 1;
        unsigned int reserved2 : 4;
        unsigned int uhs : 1;
        unsigned int ccs : 1;
        unsigned int busy : 1;
        unsigned int cmd_index : 6;
        unsigned int trans_bit : 1;
        unsigned int start_bit : 1;
    } fields;
    uint8_t bytes[6];
}RespACMD41;

// Response Format R6: Used for the published RCA address response to CMD 3
// 48 bits total length
typedef union
{
    struct
    {
        unsigned int end_bit : 1;
        unsigned int crc7 : 7;
        uint16_t card_stat_bits;
        uint16_t new_pub_rca;
        unsigned int cmd_index : 6;
        unsigned int trans_bit : 1;
        unsigned int start_bit : 1;
    } fields;
    uint8_t bytes[6];
}Resp6;

// CID register: This has the same field structure as what is decribed in 5.2
// of the Physical Simplified Specification Version 6.0, except that the CRC7
// field and "not used, always 1" bit (which is really the stop bit) are not
// included here, but instead are handled by the response
typedef union
{
    struct
    {
        unsigned int end_bit : 1;
        unsigned int crc7 : 7;
        unsigned int man_date : 12;
        unsigned int reserved : 4;
        uint32_t prod_ser;
        uint8_t prod_rev;
        uint8_t prod_name[5];
        uint16_t oem_app_id;
        uint8_t man_id;
    } fields;
    uint8_t bytes[16];
} CID;

typedef union
{
    struct CSDFields
    {
        unsigned always_1: 1;
        unsigned crc : 7;
        unsigned reserved_1 : 2;
        unsigned file_format : 2;
        unsigned tmp_write_protect : 1;
        unsigned perm_write_protect : 1;
        unsigned copy : 1;
        unsigned file_format_grp : 1;
        unsigned reserved_2 : 5;
        unsigned write_blk_partial : 1;
        unsigned write_blk_len : 4;
        unsigned r2w_factor : 3;
        unsigned reserved_3 : 2;
        unsigned wp_grp_enable : 1;
        unsigned wp_grp_size : 7;
        unsigned sector_size : 7;
        unsigned erase_blk_en : 1;
        unsigned reserved_4 : 1;
        unsigned c_size : 22;
        unsigned reserved_5 : 6;
        unsigned dsr_imp : 1;
        unsigned read_blk_misalign : 1;
        unsigned write_blk_misalign : 1;
        unsigned read_blk_partial : 1;
        unsigned read_blk_len : 4;
        unsigned ccc : 12;
        unsigned tran_speed : 8;
        unsigned nsac : 8;
        unsigned taac : 8;
        unsigned reserved_6 : 6;
        unsigned csd_structure : 2;
    } fields;
    uint8_t bytes[16];
} CSDData;

typedef union
{
    struct SDStatFields
    {
        uint8_t reserved_manufacturer[39];
        unsigned fule_support : 1;
        unsigned discard_support : 1;
        unsigned reserved_1 : 14;
        unsigned perfomance_enhance : 8;
        unsigned app_perf_class : 4;
        unsigned reserved_2 : 6;
        unsigned sus_addr : 22;
        unsigned vsc_au_size : 10;
        unsigned reserved_3 : 6;
        unsigned video_speed_class : 8;
        unsigned uhs_au_size : 4;
        unsigned uhs_speed_grade : 4;
        unsigned erase_offset : 2;
        unsigned erase_timeout : 6;
        unsigned erase_size : 16;
        unsigned reserved_4 : 4;
        unsigned au_size : 4;
        unsigned performance_move : 8;
        unsigned speed_class : 8;
        unsigned size_of_protected_area : 32;
        unsigned sd_card_type : 16;
        unsigned reserved_5 : 6;
        unsigned reserved_security : 7;
        unsigned secured_mode : 1;
        unsigned data_bus_width : 2;
    } fields;
    uint8_t bytes[64];
} SDStatus;


typedef union
{
    struct                                      // Bits:
    {
        unsigned int reserved_test_mode : 2;    // 1:0
        unsigned int reserved_app : 1;          // 2
        unsigned int ake_seq_error: 1;          // 3
        unsigned int reserved_sdio : 1;         // 4
        unsigned int app_cmd : 1;               // 5
        unsigned int fx_event : 1;              // 6
        unsigned int reserved_2 : 1;            // 7
        unsigned int ready_for_data : 1;        // 8
        unsigned int current_state : 4;         // 12:9
        unsigned int erase_reset : 1;           // 13
        unsigned int card_ecc_disabled : 1;     // 14
        unsigned int wp_erase_skip : 1;         // 15
        unsigned int csd_overwrite : 1;         // 16
        unsigned int reserved_for_deffered : 1; // 17
        unsigned int reserved_1 : 1;            // 18
        unsigned int error : 1;                 // 19
        unsigned int cc_error : 1;              // 20
        unsigned int card_ecc_failed : 1;       // 21
        unsigned int illegal_command : 1;       // 22
        unsigned int com_crc_failed : 1;        // 23
        unsigned int lock_unlock_failed : 1;    // 24
        unsigned int card_is_locked : 1;        // 25
        unsigned int wp_violation : 1;          // 26
        unsigned int erase_param : 1;           // 27
        unsigned int erase_seq_error : 1;       // 28
        unsigned int block_len_error : 1;       // 29
        unsigned int address_error : 1;         // 30
        unsigned int out_of_range : 1;          // 31
    } fields;
    uint32_t u32;
} CardStatus;


// End of tight bit field packing, which should enclose all structs
#pragma pack(pop)

// Status bits for describing results of the various response types:
#define STAT_SUCCESS            0
#define STAT_BUSY               0x00000001
#define STAT_TIMEOUT            0x00000002
#define STAT_INVALID_TRANS      0x00000004
#define STAT_INVALID_CMDIDX     0x00000008
#define STAT_INVALID_CRC        0x00000010
#define STAT_INVALID_END_BIT    0x00000020
#define STAT_ERROR              0x00000040
#define STAT_INVALID_CMD8_ECHO  0x00010000
#define STAT_ACMD41_TIMEOUT     0x00020000

// Initialization States
typedef enum
{
    STATE_CLOCK_TRAIN,      // Send a
    STATE_RESET,            // Resetting the card (CMD0)
    STATE_VERIFY_IFCOND,    // Verifying the interface operating condition (CMD8)
    STATE_INIT_LOOP,        // Initialize the card and negotiate the operating condition (ACMD55)
    STATE_CARD_IDENTIFY,    // SEND CID (Card Identification Register) - (CMD2)
    STATE_PUB_ADDRESS,      // Publish RCA (Relative Card Address) - (CMD3)
    STATE_SELECT_CARD,      // Select this card using the RCA for data transactions (CMD7)
    STATE_ENABLE_4WIRE,     // Enable the selected card for 4 wire transactions (ACMD6)
    STATE_READY_FOR_DATA    // Card is fully initialized and ready for 4 wire data transactions
} InitState;

CID cid_reg;

Resp1 resp_1;
Resp2 resp_2;
RespACMD41 resp_acmd41;
Resp6 resp_6;


CID cid_register;
uint16_t RCA = 0;

CardStatus card_stat;



uint32_t receive_resp(uint8_t cmd_idx, bool calc_crc, bool check_busy, uint8_t *buff, int len_bytes);



volatile uint32_t sys_timer = 0;

void SysTickHandler(void)
{
    if (sys_timer > 0)
    {
        sys_timer--;
    }

    if (timer_1 > 0)
    {
        timer_1--;
    }

    if (resp_timer > 0)
    {
        resp_timer--;
    }
}

// Port, Pin and Peripheral Mappings for Command (CMD) SPI Bus
// - SSI0CLK    - SCLK      -  PA2
// - SSI0FSS    - CS        -  PA3 (note: we control this directly with GPIO)
// - SSI0XDAT0  - TX (MOSI) -  PA4
// - SSI0XDAT1  - RX (MISO) -  PA5
#define CMD_PORT                GPIO_PORTA_BASE
#define CMD_PORT_PERIPH         SYSCTL_PERIPH_GPIOA
#define CMD_SSI                 SSI0_BASE
#define CMD_SSI_PERIPH          SYSCTL_PERIPH_SSI0

// Pin mappings:
#define CMD_CLK_PIN             GPIO_PIN_2
#define CMD_CS_PIN              GPIO_PIN_3
#define CMD_MOSI_PIN            GPIO_PIN_4
#define CMD_MISO_PIN            GPIO_PIN_5

// Alternate Function Configurations for the Clock, MOSI, and MISO pins:
#define CMD_CLK_CFG             GPIO_PA2_SSI0CLK
#define CMD_MOSI_CFG            GPIO_PA4_SSI0XDAT0
#define CMD_MISO_CFG            GPIO_PA5_SSI0XDAT1

// Port and Pin Mappings for the GPIO signals
#define SIGNAL_PORT             GPIO_PORTE_BASE
#define SIGNAL_PORT_PERIPH      SYSCTL_PERIPH_GPIOE
#define SIGNAL_PIN_0            GPIO_PIN_0
#define SIGNAL_PIN_1            GPIO_PIN_1
#define SIGNAL_PIN_2            GPIO_PIN_2
#define SIGNAL_PIN_3            GPIO_PIN_3
#define SIGNAL_PIN_4            GPIO_PIN_4
#define SIGNAL_PIN_5            GPIO_PIN_5

// Named Signals which will use the GPIO signals
#define SEND_CMD_PIN            SIGNAL_PIN_0
#define DONE_PIN                SIGNAL_PIN_1

// Macros which provide convenient access to each GPIO signal
#define CS_HIGH()               GPIOPinWrite(CMD_PORT, CMD_CS_PIN, CMD_CS_PIN)
#define CS_LOW()                GPIOPinWrite(CMD_PORT, CMD_CS_PIN, ~CMD_CS_PIN)
#define SEND_CMD_HIGH()         GPIOPinWrite(SIGNAL_PORT, SEND_CMD_PIN, SEND_CMD_PIN)
#define SEND_CMD_LOW()          GPIOPinWrite(SIGNAL_PORT, SEND_CMD_PIN, ~SEND_CMD_PIN)
#define DONE()                  GPIOPinRead(SIGNAL_PORT, DONE_PIN)


// Initialize the peripherals and GPIO necessary to communicate with the FPGA
void init()
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

// Test the pin mappings by toggling each pin in sequence and observing the response on the logic analyzer:
// Comment this out when not in use
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

// TODO:
// 1. Offload the CRC calculation to the fpga
// 2. Remove some of the error checking if necessary for performance (maybe use #define macros?)
uint32_t receive_resp(uint8_t cmd_idx, bool calc_crc, bool check_busy, uint8_t *buff, int len_bytes)
{
    uint32_t resp_stat = STAT_SUCCESS;
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
    // 2) a timeout occurs: TODO: What precise timeout should be used?
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
    // the FPGA send command registers will also be reset in preparation for the next command or response.
    SSIDataPut(CMD_SSI, 0);

    // The SSIDataPut operation is non-blocking, so while it is sending dummy the byte, we
    // will proceed to perform validation of the response. We will then wait for the transmission to complete
    // after we are done with our validations.

    // Obtain the first and last bytes, used for validating the cmd_index,
    // crc, trans bit, and end bit.
    LSB = buff[0];
    MSB = buff[len_bytes - 1];

    // Verify the transmission bit indicates a "card to host" transaction
    if (0 != (MSB & 0x40))
    {
        resp_stat |= STAT_INVALID_TRANS;
    }

    // Verify the End bit is 1
    if (LSB & 0x1 == 0)
    {
        resp_stat |= STAT_INVALID_END_BIT;
    }

    // Verify the command index was received correctly in the response
    if ((MSB & 0x3f) != cmd_idx)
    {
        resp_stat |= STAT_INVALID_CMDIDX;
    }

    // Verify the CRC7 is correct, if requested by the caller
    if (calc_crc)
    {
        // Get the CRC from bits 1-7 of the LSB
        crc_received = LSB >> 1;

        // Calculate the CRC7 from all other bytes in the buffer
        crc_calc =crc7(buff+1, len_bytes-1);

        // Verify the calculated and received CRC's match
        if (crc_received != crc_calc)
        {
            resp_stat |= STAT_INVALID_CRC;
        }
    }

    // Wait for the dummy byte clock cycles to be completed, and discard
    // the received byte from the FIFO. As noted, we
    // did some other calculations while waiting to make better use of this time.
    while(SSIBusy(CMD_SSI)){};
    SSIDataGet(CMD_SSI, &byte_in);

    // TODO: Add a wait on the busy signal (D0 held low) for R1b commands

    return resp_stat;
}

inline void send_initial_clock_train()
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

void read_single_block(uint32_t block_address)
{


}

uint32_t stat;
InitState init_state;

void card_init(void)
{
    int i;
    stat = STAT_SUCCESS;
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

    // Verify the voltage supplied and check pattern are echoed back exactly in the CMD8
    // response. Note that technically CMD8 has an R7 response, but treating it as an
    // R1 response is more convenient
    if (resp_1.fields.card_status != ARG_CMD8)
        stat |= STAT_INVALID_CMD8_ECHO;

    if (STAT_SUCCESS != stat)
        return;

    // Verify the operating conditions and wait for the card to initialize (or a 1s timeout to occur)
    init_state = STATE_INIT_LOOP;
    timer_1 = 100;
    while (timer_1 > 0)
    {
        // Send CMD55 to start the APP Specific Command sequence
        send_cmd(CMD55, 0);
        stat |= receive_resp(CMD55, true, false, resp_1.bytes, sizeof(Resp1));
        if (STAT_SUCCESS != stat)
            return;

        // Send CMD41 to complete the ACMD41 command
        send_cmd(CMD41, ARG_CMD41);

        // Note that the ACMD41 response does not use the CRC7 fild, and the
        // cmd_idx field actually contains 0x3f (not 41)
        stat |= receive_resp(0x3f, false, false, resp_acmd41.bytes, sizeof(RespACMD41));
        if (STAT_SUCCESS != stat)
            return;

        // Check if we are still initializing, or if we are done
        // (Although it is counter-intuitive, busy=1 actually indicates initialization is done)
        if (1 == resp_acmd41.fields.busy)
            break;
    }

    if (0 == timer_1 && 0 == resp_acmd41.fields.busy)
    {
        stat |= STAT_ACMD41_TIMEOUT;
        return;
    }

    // Get the card identification (CID) information
    init_state = STATE_CARD_IDENTIFY;
    send_cmd(CMD2, 0);
    // Note, CMD 2 does not send the CMD index back in the response, and although
    // it does use a CRC7, this is only calculated over the contents of the internal register.
    // The CRC only covers the register contents and not the cmd index which preceeds it.
    stat |= receive_resp(0x3F, false, false, resp_2.bytes, sizeof(Resp2));
    if (STAT_SUCCESS != stat)
        return;

     // Copy the contents of the R2 response to the CID register for easier field access
     // Note: resp_2 actually has 17 bytes, but we don't care about the MSB, since it
     // contains the start bit, transmission bit, and cmd index, and is not part of the register
     for (i=0; i<16; i++)
     {
         cid_reg.bytes[i] = resp_2.bytes[i];
     }

     // Request the card publishes its RCA (relative card address)
     init_state = STATE_PUB_ADDRESS;
     send_cmd(CMD3, 0);
     stat |= receive_resp(CMD3, true, false, resp_6.bytes, sizeof(Resp6));
     if (STAT_SUCCESS != stat)
         return;

     // Store the RCA for convenience in accessing it later
     // TODO: extract status information
     RCA = resp_6.fields.new_pub_rca;

     // Using the new RCA, address the card using CMD7
     arg = RCA << 16;
     send_cmd(CMD7, arg);
     stat |= receive_resp(CMD7, true, true, resp_1.bytes, sizeof(Resp1));
     if (STAT_SUCCESS != stat)
         return;
     card_stat.u32 = resp_1.fields.card_status;
     // Todo: use status info

     // Now that the card is addressed, put in 4bit Bus mode using ACMD6
    arg = RCA << 16;
    send_cmd(CMD55, arg);
    stat |= receive_resp(CMD55, true, false, resp_1.bytes, sizeof(Resp1));
    if (STAT_SUCCESS != stat)
        return;

    send_cmd(CMD6, 0x00000002);
    stat |= receive_resp(CMD6, true, false, resp_1.bytes, sizeof(Resp1));
    if (STAT_SUCCESS != stat)
        return;
    card_stat.u32 = resp_1.fields.card_status;

    // TODO: verify from the status bits that we are ready
    read_single_block(0);
}


int main(void)
{
    int led_half_period = 1000;

    //Set clock frequency to 120 MHz:
    MAIN_sys_clock = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ |
                       SYSCTL_OSC_MAIN |
                       SYSCTL_USE_PLL |
                       SYSCTL_CFG_VCO_480,
                       120000000);

    // Configure SysTick for a 100Hz interrupt.
    SysTickPeriodSet(MAIN_sys_clock / 100);
    SysTickEnable();
    SysTickIntEnable();

    // Initialize the LED
    led_init();

    test_pin_mappings();

    // Initialize the peripheral hardware necessary to communicate with the SD card
    // init();

    // Initialize the card.
    // card_init();

    // After completing the initialization, we will either blink the LED slowly on success
    // or quickly if failure occurred
    //  if (STAT_SUCCESS != stat)
    //     led_half_period = 100;

    while(1)
    {
        volatile stat_local = stat;
        volatile init_state_local = init_state;

        DELAY_MS(led_half_period);

        led_on();

        DELAY_MS(led_half_period);

        led_off();
    }
}




