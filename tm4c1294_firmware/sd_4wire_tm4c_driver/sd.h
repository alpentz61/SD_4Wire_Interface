/*
 * sh.h
 *
 *  Created on: Jan 3, 2021
 *      Author: Andy
 */

#ifndef SD_H_
#define SD_H_

#include <stdint.h>
#include <stdbool.h>
//=============================================================================
// Constants

// General Constants
#define BLOCK_LEN   512

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

// Command Indices
#define CMD0                    0
#define CMD2                    2
#define CMD3                    3
#define CMD6                    6
#define CMD7                    7   // Select (address) a card so that data transfers can be performed
#define CMD8                    8
#define CMD17                   17  // Single block read
#define CMD41                   41
#define CMD55                   55

// Constant command arguments
#define ARG_CMD8                0x000001AA
#define ARG_CMD41               0x513C0000

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

//=============================================================================
// Type Definitions

// Enclose all SD protocol data structures in this to ensure tight bit field packing.
#pragma pack(push, 1)

// Command / Response data structures.  These are defined in the reverse order, since the TM4C processors
// are Little-Endian, and the SD shifts data MSB first.  By putting them in reverse order and
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
} Resp1;

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
} Resp2;

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
} RespACMD41;

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
} Resp6;

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

// End of tight bit field packing, which should enclose all SD protocol structs
#pragma pack(pop)

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

//=============================================================================
// Public Function Prototypes

void SD_init();
uint32_t SD_card_init(void);

// TODO:
// SD_read_single_block()
// SD_write_single_block()
// SD_read_multi_block()
// SD_write_multi_block()
// SD_erase()


#endif /* SD_H_ */
