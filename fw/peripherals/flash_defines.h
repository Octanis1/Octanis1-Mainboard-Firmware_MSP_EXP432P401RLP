#ifndef FLASH_DEFINES_H
#define FLASH_DEFINES_H

#define STATUS_BUSY     ((1<<0) | (1<<7))
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
#define NOP             0x00 // No Operation
#define RSTEN           0x66 // Reset Enable
#define RST             0x99 // Reset Memory
#define EQIO            0x38 // Enable Quad I/O
#define RSTQIO5         0xff // Reset Quad I/O
#define RDSR            0x05 // Read Status Register
#define WRSR            0x01 // Write Status Register
#define RDCR            0x35 // Read Configuration Register

// Read
#define READ            0x03 // Read Memory
#define READ_HS         0x0b // Read Memory at Higher Speed
#define SQOR6           0x6b // SPI Quad Output Read
#define SQIOR7          0xeb // SPI Quad I/O Read
#define SDOR8           0x3b // SPI Dual Output Read
#define SDIOR9          0xbb // SPI Dual I/O Read
#define SB              0xc0 // Set Burst Length
#define RBSQI           0x0c // SQI Read Burst with Wrap
#define RBSPI7          0xec // SPI Read Burst with Wrap

// Identification
#define JEDEC_ID        0x9f // JEDEC-ID Read
#define QUAD_J_ID       0xaf // Quad I/O J-ID Read
#define SFDP            0x5a // Serial Flash Discoverable Parameters

// Write
#define WREN            0x06 // Write Enable
#define WRDI            0x04 // Write Disable
#define SE10            0x20 // Erase 4 KBytes of Memory Array
#define BE11            0xd8 // Erase 64, 32 or 8 KBytes of Memory Array
#define CE              0xc7 // Erase Full Array
#define PP              0x02 // Page Program
#define SPI_QUAD_PP6    0x32 // SQI Quad Page Program
#define WRSU            0xb0 // Suspends Program/Erase
#define WRRE            0x30 // Resumes Program/Erase

// Protection
#define RBPR            0x72 // Read Block-Protection Register
#define WBPR            0x42 // Write Block-Protection Register
#define LBPR            0x8d // Lock Down Block-Protection Register
#define nVWLDR          0xe8 // non-Volatile Write Lock- Down Register
#define ULBPR           0x98 // Global Block Protection Unlock
#define RSID            0x88 // Read Security ID
#define PSID            0xa5 // Program User Security ID area
#define LSID            0x85 // Lockout Security ID Pro- gramming

#endif /* FLASH_DEFINES_H */
