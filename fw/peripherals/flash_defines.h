#ifndef FLASH_DEFINES_H
#define FLASH_DEFINES_H

#define FLASH_PAGE_SIZE   (0x100) //256 may depend on flash chip, can be increased to 512
#define FLASH_MAX_MESSAGE (0xFF) //255
#define FLASH_BLOCK_SIZE  (0x10000)  // 64K
#define FLASH_SIZE        (0x1000000) // 16M

#define STATUS_BUSY     (1<<0)
#define STATUS_WEL      (1<<1) // Write-Enable Latch status
#define STATUS_WSE      (1<<2) // Write Suspend-Erase status
#define STATUS_WSP      (1<<3) // Write Suspend-Program status
#define STATUS_WPLD     (1<<4) // Write Protection Lock-Down status
#define STATUS_SEC      (1<<5) // Security ID status

#define CONFIG_IOC      (1<<1) // I/O Configuration for SPI Mode
#define CONFIG_BPNV     (1<<3) // Block-Protection Volatility State
#define CONFIG_WPEN     (1<<7) // Write-Protection Pin (WP#) Enable

// Flash instructions

// Configuration
#define FLASH_NOP             0x00 // No Operation
#define FLASH_RSTEN           0x66 // Reset Enable
#define FLASH_RST             0x99 // Reset Memory
#define FLASH_EQIO            0x38 // Enable Quad I/O
#define FLASH_RSTQIO          0xff // Reset Quad I/O
#define FLASH_RDSR1           0x05 // Read Status Register 1
#define FLASH_RDSR2           0x07 // Read Status Register 2
#define FLASH_WRR             0x01 // Write Registers (SR1 CR SR2)
#define FLASH_RDCR            0x35 // Read Configuration Register

// Read
#define FLASH_READ            0x03 // Read Memory
#define FLASH_READ_HS         0x0b // Read Memory at Higher Speed
#define FLASH_SQOR            0x6b // SPI Quad Output Read
#define FLASH_SQIOR           0xeb // SPI Quad I/O Read
#define FLASH_SDOR            0x3b // SPI Dual Output Read
#define FLASH_SDIOR           0xbb // SPI Dual I/O Read
#define FLASH_SB              0xc0 // Set Burst Length
#define FLASH_RBSQI           0x0c // SQI Read Burst with Wrap
#define FLASH_RBSPI           0xec // SPI Read Burst with Wrap

// Identification
#define FLASH_RDID            0x9f // JEDEC-ID Read
#define FLASH_QUAD_J_ID       0xaf // Quad I/O J-ID Read
#define FLASH_SFDP            0x5a // Serial Flash Discoverable Parameters

// Write
#define FLASH_WREN            0x06 // Write Enable
#define FLASH_WRDI            0x04 // Write Disable
#define FLASH_P4E             0x20 // Erase 4 KBytes of Memory Array
#define FLASH_SE              0xd8 // Erase 64, 32 or 8 KBytes of Memory Array
#define FLASH_BE              0xc7 // Erase Full Array
#define FLASH_PP              0x02 // Page Program
#define FLASH_SPI_QUAD_PP     0x32 // SQI Quad Page Program
#define FLASH_WRSU            0xb0 // Suspends Program/Erase
#define FLASH_WRRE            0x30 // Resumes Program/Erase

// Protection
#define FLASH_RBPR            0x72 // Read Block-Protection Register
#define FLASH_WBPR            0x42 // Write Block-Protection Register
#define FLASH_LBPR            0x8d // Lock Down Block-Protection Register
#define FLASH_nVWLDR          0xe8 // non-Volatile Write Lock- Down Register
#define FLASH_ULBPR           0x98 // Global Block Protection Unlock
#define FLASH_RSID            0x88 // Read Security ID
#define FLASH_PSID            0xa5 // Program User Security ID area
#define FLASH_LSID            0x85 // Lockout Security ID Pro- gramming

#endif /* FLASH_DEFINES_H */
