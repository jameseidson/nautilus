#include "nautilus/spinlock.h"
#include <nautilus/nautilus.h>
#include <nautilus/cpu.h>
#include <nautilus/mm.h>
#include <nautilus/irq.h>
#include <nautilus/dev.h>
#include <nautilus/blkdev.h>
#include <nautilus/naut_string.h>
#include <nautilus/timer.h>
#include <nautilus/condvar.h>

#include <dev/pci.h>
#include <dev/ahci.h>

#ifndef NAUT_CONFIG_DEBUG_AHCI
#undef  DEBUG_PRINT
#define DEBUG_PRINT(fmt, args...) 
#endif

#define ERROR(fmt, args...) ERROR_PRINT("ahci: " fmt, ##args)
#define DEBUG(fmt, args...) DEBUG_PRINT("ahci: " fmt, ##args)
#define INFO(fmt, args...)  INFO_PRINT("ahci: " fmt, ##args)

#define LOCK_CONF          uint8_t _state_lock_flags
#define LOCK_ACQUIRE(lock) _state_lock_flags = spin_lock_irq_save(&(lock))
#define LOCK_RELEASE(lock) spin_unlock_irq_restore(&(lock), _state_lock_flags)

#define ATOMIC_LOAD(srcptr)          __atomic_load_n(srcptr, __ATOMIC_SEQ_CST)
#define ATOMIC_STORE(destptr, value) __atomic_store_n(destptr, value, __ATOMIC_SEQ_CST)

#define PCI_INTEL_VENDOR_ID     0x8086  // intel
#define PCI_ICH9_DEVICE_ID      0x2922  // ICH9
#define PCI_CONTROLLER_CLASS    0x01    // mass storage device
#define PCI_CONTROLLER_SUBCLASS 0x06    // serial ATA
#define PCI_HEADER_TYPE         0x0     // endpoint

#define ATA_BLKSIZE 512

#define HBA_MAX_PORTS 32

#define FIS_MAX_NBLKS 65536

#define CMD_LIST_NSLOTS  32
#define CMD_TBL_MAX_NPRD 65535

#define PRDT_MAX_LEN   (8 * 1024)
#define PRDT_MAX_NBLKS (PRDT_MAX_LEN / ATA_BLKSIZE)

#define ATA_CFG_CMD_IRQ_DISABLE (1 << 10)
#define ATA_CFG_CMD_BUSMASTER   (1 << 2)
#define ATA_CFG_CMD_MEMSPACE    (1 << 1)
#define ATA_CFG_CMD_IOSPACE      1

#define HBA_GHC_AHCI_ENABLE (1 << 31)
#define HBA_GHC_IRQ_ENABLE  (1 << 1)
#define HBA_GHC_RESET        1

#define PORT_CMD_RUNNING (1 << 15)
#define PORT_CMD_FIS     (1 << 4)
#define PORT_CMD_START    1

#define PORT_IS_TF_ERR    (1 << 30)
#define PORT_IS_PRD       (1 << 5)
#define PORT_IS_DMA_SETUP (1 << 2)
#define PORT_IS_PIO_SETUP (1 << 1)
#define PORT_IS_D2H        1

#define PORT_IE_TF_ERR        (1 << 30)
#define PORT_IE_OVERFLOW      (1 << 24)
#define PORT_IE_PRD_PROCESSED (1 << 5)
#define PORT_IE_DMA_SETUP     (1 << 2)
#define PORT_IE_PIO_SETUP     (1 << 1)
#define PORT_IE_D2H            1

#define PORT_TFD_BUSY 0x80
#define PORT_TFD_DRQ  0x08

#define PORT_SSTS_DET_DETECTED 3
#define PORT_SSTS_IPM_ACTIVE   1

#define FIS_DEVICE_LBAMODE (1 << 6)

typedef enum {
  ATA_CMD_IDENTIFY_DMA  = 0xEE,
  ATA_CMD_IDENTIFY_PIO  = 0xEC,
  ATA_CMD_READ_DMA_EXT  = 0x25,
  ATA_CMD_WRITE_DMA_EXT = 0x35,
} ata_cmd_t;

typedef enum {
  SATA_SIG_ATA   = 0x00000101,  // SATA drive
  SATA_SIG_ATAPI = 0xEB140101,  // SATAPI drive
  SATA_SIG_SEMB  = 0xC33C0101,  // Enclosure management bridge
  SATA_SIG_PM    = 0x96690101,  // Port multiplier
} sata_sig_t;

typedef enum {
  FIS_TYPE_REG_H2D   = 0x27,  // Register FIS - host to device
  FIS_TYPE_REG_D2H   = 0x34,  // Register FIS - device to host
  FIS_TYPE_DMA_ACT   = 0x39,  // DMA activate FIS - device to host
  FIS_TYPE_DMA_SETUP = 0x41,  // DMA setup FIS - bidirectional
  FIS_TYPE_DATA      = 0x46,  // Data FIS - bidirectional
  FIS_TYPE_BIST      = 0x58,  // BIST activate FIS - bidirectional
  FIS_TYPE_PIO_SETUP = 0x5F,  // PIO setup FIS - device to host
  FIS_TYPE_DEV_BITS	 = 0xA1,	// Set device bits FIS - device to host
} fis_type_t;

typedef enum {
  AHCI_DEV_NULL   = 0,
  AHCI_DEV_SATA   = 1,
  AHCI_DEV_SEMB   = 2,
  AHCI_DEV_PM     = 3,
  AHCI_DEV_SATAPI = 4,
} ahci_dev_t;

typedef struct {
  // DWORD 0
  uint8_t fis_type;	// FIS_TYPE_REG_H2D
 
  uint8_t pmport:4;	// Port multiplier
  uint8_t rsv0:3;		// Reserved
  uint8_t c:1;		  // 1: Command, 0: Control
 
  uint8_t command;	// Command register
  uint8_t featurel;	// Feature register, 7:0
 
  // DWORD 1
  uint8_t lba0;	 	  // LBA low register, 7:0
  uint8_t lba1;	 	  // LBA mid register, 15:8
  uint8_t lba2;	 	  // LBA high register, 23:16
  uint8_t device;   // Device register
 
  // DWORD 2
  uint8_t lba3;		  // LBA register, 31:24
  uint8_t lba4;		  // LBA register, 39:32
  uint8_t lba5;		  // LBA register, 47:40
  uint8_t featureh; // Feature register, 15:8
 
  // DWORD 3
  union {
    uint16_t count; // Count register, 7:15
    struct {
      uint8_t count_l; // Count low
      uint8_t count_h; // Count high
    } __attribute__((packed));
  };
  uint8_t icc;		  // Isochronous command completion
  uint8_t control;	// Control register
 
  // DWORD 4
  uint8_t rsv1[4];	// Reserved
} __attribute__((packed)) fis_h2d_t;

typedef volatile struct {
  // DWORD 0
  uint8_t fis_type;    // FIS_TYPE_REG_D2H
 
  uint8_t pmport:4;    // Port multiplier
  uint8_t rsv0:2;      // Reserved
  uint8_t i:1;         // Interrupt bit
  uint8_t rsv1:1;      // Reserved
 
  union {
    uint8_t val;
    struct {
      uint8_t err:1;  // error occurred
      uint8_t rsv:2; 
      uint8_t drq:1;  // drive has data ready or can accept data (PIO)
      uint8_t srv:1;  // overlapped mode?
      uint8_t df:1;   // drive fault
      uint8_t rdy:1;  // drive ready
      uint8_t bsy:1;  // drive busy
    } __attribute__((packed));
  } status;            // Status register

  uint8_t error;       // Error register
 
  // DWORD 1
  uint8_t lba0;        // LBA low register, 7:0
  uint8_t lba1;        // LBA mid register, 15:8
  uint8_t lba2;        // LBA high register, 23:16
  uint8_t device;      // Device register
 
  // DWORD 2
  uint8_t lba3;        // LBA register, 31:24
  uint8_t lba4;        // LBA register, 39:32
  uint8_t lba5;        // LBA register, 47:40
  uint8_t rsv2;        // Reserved
 
  // DWORD 3
  uint8_t countl;      // Count register, 7:0
  uint8_t counth;      // Count register, 15:8
  uint8_t rsv3[2];     // Reserved
 
  // DWORD 4
  uint8_t  rsv4[4];     // Reserved
} __attribute__((packed)) fis_d2h_t;

typedef volatile struct {
  // DWORD 0
  uint8_t fis_type;	// FIS_TYPE_PIO_SETUP
 
  uint8_t pmport:4;	// Port multiplier
  uint8_t rsv0:1;		// Reserved
  uint8_t d:1;		// Data transfer direction, 1 - device to host
  uint8_t i:1;		// Interrupt bit
  uint8_t rsv1:1;
 
  uint8_t status;		// Status register
  uint8_t error;		// Error register
 
  // DWORD 1
  uint8_t lba0;		// LBA low register, 7:0
  uint8_t lba1;		// LBA mid register, 15:8
  uint8_t lba2;		// LBA high register, 23:16
  uint8_t device;		// Device register
 
  // DWORD 2
  uint8_t lba3;		// LBA register, 31:24
  uint8_t lba4;		// LBA register, 39:32
  uint8_t lba5;		// LBA register, 47:40
  uint8_t rsv2;		// Reserved
 
  // DWORD 3
  uint8_t countl;		// Count register, 7:0
  uint8_t counth;		// Count register, 15:8
  uint8_t rsv3;		// Reserved
  uint8_t e_status;	// New value of status register
 
  // DWORD 4
  uint16_t tc;		// Transfer count
  uint8_t rsv4[2];	// Reserved
} __attribute__((packed)) fis_pio_setup_t;;

typedef volatile struct {
  // DWORD 0
  uint8_t fis_type;	      // FIS_TYPE_DMA_SETUP
 
  uint8_t pmport:4;	      // Port multiplier
  uint8_t rsv0:1;		      // Reserved
  uint8_t d:1;		        // Data transfer direction, 1 - device to host
  uint8_t i:1;		        // Interrupt bit
  uint8_t a:1;            // Auto-activate. Specifies if DMA Activate FIS is needed
 
  uint8_t rsv[2];         // Reserved
 
  //DWORD 1&2
  uint64_t dma_buf_id;    // DMA Buffer Identifier. Used to Identify DMA buffer in host memory.
                          // SATA Spec says host specific and not in Spec. Trying AHCI spec might work.

  //DWORD 3
  uint32_t rsv1;          // More reserved

  //DWORD 4
  uint32_t dma_buf_offset;  // Byte offset into buffer. First 2 bits must be 0

  //DWORD 5
  uint32_t tc;              // Number of bytes to transfer. Bit 0 must be 0

  //DWORD 6
  uint32_t rsv2;          // Reserved
} __attribute__((packed)) fis_dma_setup_t;

typedef volatile struct{
  // 0x00
  fis_dma_setup_t dsfis; // DMA setup FIS
  uint8_t pad0[4];
 
  // 0x20
  fis_pio_setup_t psfis; // PIO setup FIS
  uint8_t pad1[12];
 
  // 0x40
  fis_d2h_t rfis;        // D2H register FIS
  uint8_t pad2[4];
 
  // 0x58
  uint8_t sdbfis[8];		 // Set device bit FIS
 
  // 0x60
  uint8_t ufis[64];      // Unknown FIS
 
  // 0xA0
  uint8_t rsv[0x100-0xA0];
} __attribute__((packed, aligned(256))) fis_hba_t;

typedef struct {
  union {
    volatile void *dba;
    struct {
      uint32_t dba_l;		   // Data base address lower 32 bits
      uint32_t dba_u;		   // Data base address upper 32 bits
    } __attribute__((packed));
  };                 // Data base address
  uint32_t rsv0;		 // Reserved

  uint32_t dbc:22;	 // Byte count, 4M max
  uint32_t rsv1:9;	 // Reserved
  uint32_t i:1;		   // Interrupt on completion
} __attribute__((packed)) prd_t;

typedef struct {
  // 0x00
  uint8_t cfis[64];    // Command FIS

  // 0x40
  uint8_t acmd[16];    // ATAPI command
  
  // 0x50
  uint8_t rsv[48];	   // Reserved

  // 0x80
  prd_t prdt[CMD_TBL_MAX_NPRD];  // Physical region descriptor table entries, 0 ~ 65535
} __attribute__((packed, aligned(128))) cmd_table_t;

typedef struct {
  // DW0
  uint8_t  cfl:5;	  // Command FIS length in DWORDS, 2 ~ 16
  uint8_t  a:1;		  // ATAPI
  uint8_t  w:1;		  // Write, 1: H2D, 0: D2H
  uint8_t  p:1;		  // Prefetchable
  uint8_t  r:1;		  // Reset
  uint8_t  b:1;		  // BIST
  uint8_t  c:1;		  // Clear busy upon R_OK
  uint8_t  rsv0:1;  // Reserved
  uint8_t  pmp:4;		// Port multiplier port

  uint16_t prdtl;		// Physical region descriptor table length in entries

  // DW1
  volatile uint32_t prdbc;		// Physical region descriptor byte count transferred

  // DW2, 3
  union {
    cmd_table_t *ctba;
    struct {
      uint32_t ctba_l;		// Command table descriptor base address
      uint32_t ctba_u;		// Command table descriptor base address upper 32 bits
    } __attribute__((packed));
  };

  // DW4 - 7
  uint32_t rsv1[4];	// Reserved
} __attribute__((packed)) cmd_hdr_t;

typedef struct {
  cmd_hdr_t cmds[CMD_LIST_NSLOTS];
} __attribute__((packed, aligned(1024))) cmd_list_t;

typedef volatile struct {
    union {
      cmd_list_t *clb;
      struct {
        uint32_t clb_l;  // 0x00, command list base address lower 32 bits, 1K-byte aligned
        uint32_t clb_u;  // 0x04, command list base address upper 32 bits
      } __attribute__((packed));
    };                         // command list base address 1 ~ 32

    union {
      fis_hba_t *fb;
      volatile struct {
        uint32_t fb_l; // 0x08, FIS base address lower 32 bits, 256-byte aligned
        uint32_t fb_u; // 0x0C, FIS base address upper 32 bits
      } __attribute__((packed));
    };                         // fis base address 

    volatile uint32_t is;      // 0x10, interrupt status  
    uint32_t ie;               // 0x14, interrupt enable
    
    union {
      uint32_t val;
      struct {
        uint8_t st    : 1;  // start
        uint8_t sud   : 1;  // spin-up device
        uint8_t pod   : 1;  // power on device
        uint8_t clo   : 1;  // command list override
        uint8_t fre   : 1;  // fis receive enable
        uint8_t rsvd  : 3;  // reserved
        uint8_t ccs   : 5;  // current command slot
        uint8_t mpss  : 1;  // mechanical presence switch state
        uint8_t fr    : 1;  // fis receive running
        uint8_t cr    : 1;  // command list running
        uint8_t cps   : 1;  // cold presence state
        uint8_t pma   : 1;  // port multiplier attached
        uint8_t hpcp  : 1;  // hot plug capable port
        uint8_t mpsp  : 1;  // mechanical presence switch attached to port
        uint8_t cpd   : 1;  // cold presence detection
        uint8_t esp   : 1;  // external sata port
        uint8_t fbscp : 1;  // fis-based switching capable port
        uint8_t apste : 1;  // automatic partial to slumber transitions enabled
        uint8_t atapi : 1;  // device is atapi
        uint8_t dlae  : 1;  // drive led on atapi enable
        uint8_t alpe  : 1;  // aggressive link power management enable
        uint8_t asp   : 1;  // aggressive slumber/partial 
        uint8_t icc   : 4;  // interface communication control 
      } __attribute__((packed));
    } cmd;		                 // 0x18, command and status
    
    uint32_t rsv0;		         // 0x1C, Reserved
    uint32_t tfd;		           // 0x20, task file data
    uint32_t sig;		           // 0x24, signature
    
    union {
      uint32_t val;
      struct {
        uint8_t det   : 4;    // Device detection
        uint8_t spd   : 4;    // Current interface speed
        uint8_t ipm   : 4;    // Interface power management
        uint32_t rsv  : 20;
      } __attribute__((packed));
    } ssts;                    // 0x28, SATA status (SCR0:SStatus)

    union {
      uint32_t val;
      struct {
        uint8_t det  : 4;   // device detection
        uint8_t spd  : 4;   // speed allowed
        uint8_t ipm  : 4;   // power management transitions allowed
        uint32_t rsv : 20;
      } __attribute__((packed));
    } sctl;                    // 0x2C, SATA control (SCR2:SControl)

    uint32_t serr;		         // 0x30, SATA error (SCR1:SError)
    uint32_t sact;		         // 0x34, SATA active (SCR3:SActive)
    uint32_t ci;		           // 0x38, command issue
    uint32_t sntf;		         // 0x3C, SATA notification (SCR4:SNotification)
    uint32_t fbs;		           // 0x40, FIS-based switch control
    uint32_t devslp;            // 0x44, Device Sleep
    uint32_t rsv1[10];	       // 0x47 - 0x6F, Reserved
    uint32_t vendor[4];	       // 0x70 ~ 0x7F, vendor specific
} __attribute__((packed)) hba_port_reg_t;

typedef volatile struct {
  // 0x00 - 0x2B, Generic Host Control
  union {
    uint32_t val;
    struct {
      uint8_t np    : 5;  // number of ports 
      uint8_t sxs   : 1;  // supports external SATA
      uint8_t ems   : 1;  // enclosure management supported
      uint8_t cccs  : 1;  // command completion coalescing supported
      uint8_t ncs   : 5;  // number of command slots
      uint8_t psc   : 1;  // partial state capable
      uint8_t ssc   : 1;  // slumber state capable
      uint8_t pmd   : 1;  // pio multiple drq block
      uint8_t fbss  : 1;  // fis-based switching supported
      uint8_t spm   : 1;  // supports port multiplier
      uint8_t sam   : 1;  // supports ahci mode only
      uint8_t rsv   : 1;  // reserved
      uint8_t iss   : 4;  // interface speed support
      uint8_t sclo  : 1;  // supports command list override
      uint8_t sal   : 1;  // supports activity led
      uint8_t salp  : 1;  // supports aggressive link power management
      uint8_t sss   : 1;  // supports staggered spin up
      uint8_t smps  : 1;  // supports mechanical presence switch
      uint8_t ssntf : 1;  // supports snotification register
      uint8_t sncq  : 1;  // supports native command queuing
      uint8_t s64a  : 1;  // supports 64 bit addressing
    } __attribute__((packed));
  } cap;                       // 0x00, Host Capability

  uint32_t ghc;                // 0x04, Global host control
  uint32_t is;                 // 0x08, Interrupt status
  uint32_t pi;		             // 0x0C, Port implemented
  uint32_t vs;		             // 0x10, Version
  uint32_t ccc_ctl;            // 0x14, Command completion coalescing control
  uint32_t ccc_pts;	           // 0x18, Command completion coalescing ports
  uint32_t em_loc;             // 0x1C, Enclosure management location
  uint32_t em_ctl;	           // 0x20, Enclosure management control
  uint32_t cap2;	             // 0x24, Host capabilities extended
  uint32_t bohc;		           // 0x28, BIOS/OS handoff control and status

  // 0x2C - 0x9F, Reserved
  uint8_t rsv[0xA0-0x2C];   
  
  // 0xA0 - 0xFF, Vendor specific registers 
  uint8_t vendor[0x100-0xA0];

  // 0x100 - 0x10FF, Port control registers 1 ~ 32 
  hba_port_reg_t ports[HBA_MAX_PORTS];
} __attribute__((packed)) hba_reg_t;

_Static_assert(sizeof(fis_hba_t)           == 0x100,  "bad fis_hba_t length");
_Static_assert(offsetof(fis_hba_t, dsfis)  == 0x00,   "bad dsfis offset in fis_hba_t");
_Static_assert(offsetof(fis_hba_t, psfis)  == 0x20,   "bad psfis offset in fis_hba_t");
_Static_assert(offsetof(fis_hba_t, rfis)   == 0x40,   "bad rfis offset in fis_hba_t");
_Static_assert(offsetof(fis_hba_t, sdbfis) == 0x58,   "bad sdbfis offset in fis_hba_t");
_Static_assert(offsetof(fis_hba_t, ufis)   == 0x60,   "bad ufis offset in fis_hba_t");
_Static_assert(sizeof(prd_t)               == 0x10,   "bad prd_t length\n");
_Static_assert(offsetof(cmd_table_t, prdt) == 0x80,   "bad prdt offset in cmd_table_t");
_Static_assert(sizeof(cmd_hdr_t)           == 0x20,   "bad cmd_hdr_t length");
_Static_assert(offsetof(cmd_hdr_t, ctba_l) == 0x8,    "bad ctba offset in cmd_hdr_t");
_Static_assert(sizeof(cmd_list_t)          == (0x20 * CMD_LIST_NSLOTS), "bad cmd_list_t length");
_Static_assert(sizeof(hba_port_reg_t)      == 0x80,   "bad hba_port_reg_t length");
_Static_assert(sizeof(hba_reg_t)           == 0x1100, "bad hba_reg_t length");
_Static_assert(offsetof(hba_reg_t, ports)  == 0x100,  "bad ports offset in hba_reg_t");

typedef void (ahci_cmd_callback_t)(nk_block_dev_status_t, void *);

typedef struct ahci_cmd_continuation {
  ahci_cmd_callback_t *cb;
  void *context;
} ahci_cmd_continuation_t;

typedef struct ahci_cmd_state {
  uint8_t nslots;
  // lock for `running`
  spinlock_t lock;
  // threads wait on this for an available slot
  nk_condvar_t slot_avail;
  // 1-bit indicates a command is currently running; this is a shared resource
  uint32_t running;

  // continuation to call when a command finishes 
  struct ahci_cmd_continuation conts[CMD_LIST_NSLOTS];
} ahci_cmd_state_t;

typedef struct ahci_port_state {
  struct ahci_controller_state *parent;

  // response from ata identify, stored here to ensure it doesn't leak
  uint16_t __attribute__((aligned(2))) *ata_identity;
  volatile int identified;

  // the inherited interface
  struct nk_block_dev *blkdev;
  struct nk_block_dev_characteristics chars;

  uint8_t num;
  ahci_dev_t type;
  // points to port registers within abar
  hba_port_reg_t *regs;
  // state of the commands currently being processed by this port
  struct ahci_cmd_state cmds;
} ahci_port_state_t;

typedef struct ahci_controller_state {
  // ahci base address register
  hba_reg_t *abar;

  struct pci_dev *pdev;

  struct ahci_port_state ports[HBA_MAX_PORTS];
} ahci_controller_state_t;

static int ahci_handle_irq(excp_entry_t *excp, excp_vec_t vec, void *state);

static int get_characteristics(void *state, struct nk_block_dev_characteristics *c);
static int read_blocks(void *state, uint64_t blocknum, uint64_t count, uint8_t *dest, void (*callback)(nk_block_dev_status_t status, void *context), void *context);
static int write_blocks(void *state, uint64_t blocknum, uint64_t count, uint8_t *src, void (*callback)(nk_block_dev_status_t status, void *context), void *context);

static struct nk_block_dev_int interface = {
    .get_characteristics = get_characteristics,
    .read_blocks         = read_blocks,
    .write_blocks        = write_blocks,
};

static inline void port_stop_cmd(ahci_port_state_t *p) {
  p->regs->cmd.val &= ~(PORT_CMD_START | PORT_CMD_FIS);

  while(p->regs->cmd.val & (PORT_CMD_START | PORT_CMD_FIS));
}

static inline void port_start_cmd(ahci_port_state_t *p) {
  while (p->regs->cmd.val & PORT_CMD_RUNNING);

  p->regs->cmd.val |= (PORT_CMD_START | PORT_CMD_FIS);
}

static inline void fis_readwrite_set_lba(fis_h2d_t *cmdfis, uint64_t blocknum, uint64_t count) {\
  cmdfis->lba0 =  blocknum        & 0xff;
  cmdfis->lba1 = (blocknum >>  8) & 0xff;
  cmdfis->lba2 = (blocknum >> 16) & 0xff;
  cmdfis->lba3 = (blocknum >> 24) & 0xff;
  cmdfis->lba4 = (blocknum >> 32) & 0xff;
  cmdfis->lba5 = (blocknum >> 40) & 0xff;

  cmdfis->device = FIS_DEVICE_LBAMODE;
  cmdfis->count = (count == FIS_MAX_NBLKS ? 0 : count) & 0xffff;
}

static inline ahci_dev_t ata_get_dev_type(hba_port_reg_t *regs) {
  if (regs->ssts.det != PORT_SSTS_DET_DETECTED) return AHCI_DEV_NULL;
  if (regs->ssts.ipm != PORT_SSTS_IPM_ACTIVE)   return AHCI_DEV_NULL;

  switch (regs->sig) {
    case SATA_SIG_ATAPI: return AHCI_DEV_SATAPI;
    case SATA_SIG_SEMB:  return AHCI_DEV_SEMB;
    case SATA_SIG_PM:    return AHCI_DEV_PM;
    default:             return AHCI_DEV_SATA;
  }
}

static inline int cmdlist_get_completed(ahci_port_state_t *p) {
  LOCK_CONF;
  LOCK_ACQUIRE(p->cmds.lock);
  // device clears ci when it's done with command
  uint32_t completed = p->cmds.running & ~p->regs->ci;
  for (int slot = 0; slot < p->cmds.nslots; slot++) {
    if (completed & 1) {
      LOCK_RELEASE(p->cmds.lock);
      return slot;
    }
    completed >>= 1;
  }
  LOCK_RELEASE(p->cmds.lock);
  return -1;
}

static inline void cmdlist_mark_slot_avail(ahci_port_state_t *p, int slot) {
  // allow future commands to use this slot
  LOCK_CONF;
  LOCK_ACQUIRE(p->cmds.lock);
  uint32_t mask = ~(1 << slot);
  p->cmds.running &= mask;
  p->regs->ci &= mask;
  // signal that a slot is now available
  nk_condvar_signal(&p->cmds.slot_avail);
  LOCK_RELEASE(p->cmds.lock);
}

static int inline cmdlist_claim_slot(ahci_port_state_t *p) {
  LOCK_CONF;
  LOCK_ACQUIRE(p->cmds.lock);
  // wait until a command slot is available
  while (p->cmds.running | p->regs->ci) {
    nk_condvar_wait(&p->cmds.slot_avail, &p->cmds.lock);
  }

  uint32_t available = ~(p->cmds.running | p->regs->ci);
  for (int slot = 0; slot < p->cmds.nslots; slot++) {
    if (available & 1) {
      // claim the slot to prevent other threads from doing so 
      p->cmds.running |= (1 << slot);
      LOCK_RELEASE(p->cmds.lock);
      return slot;
    }
    available >>= 1;
  }
  
  // should never get here because we waited for available slot
  LOCK_RELEASE(p->cmds.lock);
  return -1;
}

static inline void port_reset(ahci_port_state_t *p) {
  DEBUG("[port %d] resetting\n", p->num);
  p->regs->sctl.det = 0x1;
  // sleep for 1 ms before clearing to let device do its thing
  nk_delay(1000000);
  p->regs->sctl.det = 0x0;

  // wait for communication to be re-established
  while (p->regs->ssts.det != 0x3);
  p->regs->serr = -1;
}

static inline void controller_reset(ahci_controller_state_t *c) {
  DEBUG("controller resetting\n");
  c->abar->ghc |= HBA_GHC_RESET;
  while (c->abar->ghc & HBA_GHC_RESET);
}

static inline int controller_irq_setup(ahci_controller_state_t *c) {
  if (c->pdev->msi.type == PCI_MSI_NONE) {
    ERROR("controller does not support msi\n");
    return -1;
  }
  uint64_t nvecs = c->pdev->msi.num_vectors_needed;
  uint64_t base_vec = 0;

  if (idt_find_and_reserve_range(nvecs, 1, &base_vec)) {
      ERROR("could not find %d interrupt vectors for controller\n", nvecs);
      return -1;
  }

  DEBUG("irq vectors are %d to %d\n", base_vec, base_vec + nvecs - 1);

  if (pci_dev_enable_msi(c->pdev, base_vec, nvecs, 0)) {
      ERROR("could not enable msi for controller\n");
      return -1;
  }

  for (int i = base_vec; i < (base_vec + nvecs); i++) {
    if (register_int_handler(i, ahci_handle_irq, c)) {
      ERROR("could not register handler for interrupt vector %d\n", i);
      return -1;
    }
  }

  for (int i = base_vec; i < (base_vec + nvecs); i++) {
    if (pci_dev_unmask_msi(c->pdev, i)) {
      ERROR("could not unmask interrupt vector %d\n", i);
      return -1;
    }
  }

  return 0;
}

static inline int port_issue_cmd(ahci_port_state_t *p, fis_h2d_t cmdfis, ahci_cmd_continuation_t cont, uint8_t *data, uint64_t nblks) {
  // clear interrupt status
  p->regs->is = -1;

  if ((uint64_t)data & 1) {
    ERROR("[port %d] data buffer must be word-aligned\n");
    return -1;
  }

  int slot = cmdlist_claim_slot(p);
  if (slot == -1) {
    ERROR("[port %d] could not claim command slot\n", p->num);
  }
  DEBUG("[port %d] issuing command 0x%x to device on slot %d\n", p->num, cmdfis.command, slot);

  cmd_hdr_t *cmdhdr = &p->regs->clb->cmds[slot];
  memset(cmdhdr, 0, sizeof(cmd_hdr_t));
  // cfl holds length in dwords 
  cmdhdr->cfl = sizeof(fis_h2d_t) / sizeof(uint32_t);
  // need enough prds to hold all the blocks being requested
  cmdhdr->prdtl = ((nblks - 1) / PRDT_MAX_NBLKS) + 1;
  cmdhdr->c = 1;
  cmdhdr->w = cmdfis.command == ATA_CMD_WRITE_DMA_EXT;
  cmdhdr->a = cmdfis.command == (ATA_CMD_IDENTIFY_PIO | ATA_CMD_IDENTIFY_DMA);

  cmd_table_t *cmdtbl = cmdhdr->ctba;
  memset(cmdtbl, 0, sizeof(cmd_table_t));
  uint16_t i = 0;
  for (i = 0; i < cmdhdr->prdtl - 1; i++) {
    cmdtbl->prdt[i].dba = data;
    // dbc is 0-based
    cmdtbl->prdt[i].dbc = PRDT_MAX_LEN - 1;
    data += PRDT_MAX_LEN;
    nblks -= PRDT_MAX_NBLKS;
  }
  cmdtbl->prdt[i].dba = data;
  cmdtbl->prdt[i].dbc = (nblks * ATA_BLKSIZE) - 1;
  // trigger an interrupt when the last prd is done processing
  cmdtbl->prdt[i].i = 1;

  memcpy(cmdtbl->cfis, &cmdfis, sizeof(fis_h2d_t));

  size_t spin = 0;
  while ((p->regs->tfd & (PORT_TFD_BUSY | PORT_TFD_DRQ)) && spin < 1000000) {
    spin++;
  }
  if (spin == 1000000) {
    ERROR("[port %d] hung\n", p->num);
    return -1;
  }
  
  p->cmds.conts[slot] = cont;

  // issue the command
  p->regs->ci |= (1 << slot);

  return 0;
}

static int port_handle_irq(ahci_port_state_t *p) {
  uint32_t is = p->regs->is;
  p->regs->is = is;
  if (is & PORT_IS_TF_ERR) {
    ERROR("[port: %d] device signaled error\n", p->num);
  }

  int slot = cmdlist_get_completed(p);
  if (slot == -1) {
    DEBUG("[port %d] no commands have been completed so irq was unexpected, ignoring\n", p->num);
    return 0;
  }

  // let's be optimistic and assume we succeed :)
  nk_block_dev_status_t status = NK_BLOCK_DEV_STATUS_SUCCESS;

  if (is & PORT_IS_PIO_SETUP) {
    DEBUG("[port %d] irq was for PIO setup command\n", p->num);
  } else if (is & PORT_IS_D2H) {
    DEBUG("[port %d] irq was for D2H command\n", p->num);

    fis_d2h_t *fis = &p->regs->fb->rfis;
    if (fis->status.err || fis->status.bsy || !fis->status.rdy) {
      // :(
      INFO("[port %d] command for received irq was unsuccessful\n", p->num);
      status = NK_BLOCK_DEV_STATUS_ERROR;
    }
  }

  // call the continuation if it exists
  ahci_cmd_continuation_t *cont = &p->cmds.conts[slot];
  if (cont->cb) cont->cb(status, cont->context);

  cmdlist_mark_slot_avail(p, slot);

  return 0;
}

static void port_identify_callback(nk_block_dev_status_t status, void *context) {
  ahci_port_state_t *p = context;
  fis_pio_setup_t *fis = &p->regs->fb->psfis;

  if (!((p->ata_identity[83] >> 10) & 1)) { 
    ERROR("[port %d] could not identify, LBA48 not supported on this drive\n", p->num);
    return;
  }

  p->chars.block_size = ATA_BLKSIZE;
  p->chars.num_blocks = 
    (((uint64_t) p->ata_identity[103]) << 48) +
    (((uint64_t) p->ata_identity[102]) << 32) +
    (((uint64_t) p->ata_identity[101]) << 16) +
    (((uint64_t) p->ata_identity[100]) <<  0);

  DEBUG("[port %d] lba48 supported, device has %d blocks\n", p->num, p->chars.num_blocks);

  ATOMIC_STORE(&p->identified, 1);
}

static int port_identify(ahci_port_state_t *p) {
  fis_h2d_t cmdfis;
  memset(&cmdfis, 0, sizeof(fis_h2d_t));
  cmdfis.fis_type = FIS_TYPE_REG_H2D;
  cmdfis.command = ATA_CMD_IDENTIFY_PIO;
  cmdfis.c = 1;

  ahci_cmd_continuation_t cont;
  cont.cb = port_identify_callback;
  cont.context = p;

  return port_issue_cmd(p, cmdfis, cont, (uint8_t *)p->ata_identity, 1);
}

static int port_init(ahci_controller_state_t *c, ahci_dev_t type, int num) {
  hba_port_reg_t *regs = &c->abar->ports[num];

  // allocate command list
  regs->clb = malloc(sizeof(cmd_list_t));
  if (!regs->clb) {
    ERROR("[port %d] could not allocate command list\n", num);
    return -1;
  }
  if ((uint64_t)regs->clb & 0x3FF) {
    ERROR("[port %d] command list was not 1K-byte aligned\n", num);
    return -1;
  }
  memset(regs->clb, 0, sizeof(cmd_list_t));

  // allocate a command table in each header within the list 
  uint8_t nslots = c->abar->cap.ncs;
  for (int i = 0; i < nslots; i++) {
    regs->clb->cmds[i].ctba = malloc(sizeof(cmd_table_t));
    if (!regs->clb->cmds[i].ctba) {
      ERROR("[port %d] could not allocate command table %d\n", num, i);
      return -1;
    }
    if ((uint64_t)regs->clb->cmds[i].ctba & 0x7F) {
      ERROR("[port %d] command table %d was not 128-byte aligned\n", num, i);
      return -1;
    }
    memset(regs->clb->cmds[i].ctba, 0, sizeof(cmd_table_t));
  }

  // allocate device-to-host fis
  regs->fb = malloc(sizeof(fis_hba_t));
  if (!regs->fb) {
    ERROR("[port %d] could not allocate frame information structure\n", num);
    return -1;
  }
  if ((uint64_t)regs->fb & 0xFF) {
    ERROR("[port %d] frame information structure was not 256-byte aligned\n", num);
    return -1;
  }

  // set up device state, at this point it has not yet been registered
  ahci_port_state_t *p = &(c->ports[num]);
  p->ata_identity = malloc(ATA_BLKSIZE);
  if (!p->ata_identity) {
    ERROR("[port %d] could not allocate ata identify block\n", num);
    return -1;
  }
  if ((uint64_t)p->ata_identity & 1) {
    ERROR("[port %d] ata identify block was not word-aligned\n", num);
    return -1;
  }
  memset(p->ata_identity, 0, ATA_BLKSIZE);
  p->identified = 0;
  p->type = type;
  p->num = num;
  p->regs = regs;
  p->parent = c;
  p->blkdev = NULL;
  p->chars.block_size = 0;
  p->chars.num_blocks = 0;
  // add 1 because cap.ncs is 0-based
  p->cmds.nslots = nslots + 1;
  spinlock_init(&p->cmds.lock);
  nk_condvar_init(&p->cmds.slot_avail);
  p->cmds.running = 0;

  return 0;
}

static int probe(ahci_controller_state_t *c) {
  uint32_t pi = c->abar->pi;
  DEBUG("probing ports for devices, implemented mask: %x\n", pi);

  for (int i = 0; pi & (i < HBA_MAX_PORTS); i++) {
    // only support SATA devices
    if ((pi & 1)) {
      ahci_dev_t type = ata_get_dev_type(&c->abar->ports[i]);
      switch (type) {
        case AHCI_DEV_SATA: {
          if (port_init(c, type, i)) {
            ERROR("[port %d] failed to initialize\n", i);
            return -1;
          }
          
          ahci_port_state_t *p = &c->ports[i];
          port_reset(p);
          port_start_cmd(p);

          // enable interrupts
          p->regs->ie = (PORT_IE_D2H | PORT_IE_PIO_SETUP | PORT_IE_TF_ERR);
          
          if (port_identify(p)) {
            ERROR("[port %d] failed to identify\n", i);
            return -1;
          }

          // block until identified, then register the device
          DEBUG("[port %d], waiting for identification...\n", p->num);
          while (!ATOMIC_LOAD(&p->identified));

          char name[32];
          sprintf(name, "ahci-%d-%d", p->num, p->type);
          INFO("[port %d] initializing block device %s with type %d\n", p->num, name, p->type);
          p->blkdev = nk_block_dev_register(name, 0, &interface, p);
          break;
        } default: {
          // only support SATA devices
          break;
        }
      }
    }

    pi >>= 1;
  }

  DEBUG("finished probing\n");
  return 0;
}

static int controller_init(ahci_controller_state_t *c, struct naut_info *naut) {
  struct pci_info *pci = naut->sys.pci;
  struct list_head *curbus, *curdev;
  uint32_t num = 0;

  if (!pci) {
    ERROR("no PCI info\n");
    return -1;
  }

  DEBUG("finding ahci controller\n");

  list_for_each(curbus, &(pci->bus_list)) {
    struct pci_bus *bus = list_entry(curbus, struct pci_bus, bus_node);

    DEBUG("searching PCI bus %u for ahci controller\n", bus->num);

    list_for_each(curdev, &(bus->dev_list)) {
      struct pci_dev *pdev = list_entry(curdev, struct pci_dev, dev_node);
      struct pci_cfg_space *cfg = &pdev->cfg;
      
      DEBUG("device %u is a 0x%x:0x%x\n", pdev->num, cfg->vendor_id, cfg->device_id);
  
      if (cfg->vendor_id == PCI_INTEL_VENDOR_ID && cfg->device_id == PCI_ICH9_DEVICE_ID) {
        DEBUG("found an ahci controller\n");
          
        if (cfg->hdr_type != PCI_HEADER_TYPE) {
          ERROR("unexpected device type\n");
          return -1;
        }

        c->abar = (hba_reg_t *)pci_dev_get_bar_addr(pdev, 5);
        c->pdev = pdev;

        // allow device to use DMA + make sure interrupts are enabled
        uint16_t cmdreg = pci_cfg_readw(bus->num, pdev->num, 0, 0x4);
        cmdreg |= ATA_CFG_CMD_IOSPACE | ATA_CFG_CMD_MEMSPACE | ATA_CFG_CMD_BUSMASTER;
        cmdreg &= ~ATA_CFG_CMD_IRQ_DISABLE;
        pci_cfg_writew(bus->num, pdev->num, 0, 0x4, cmdreg);
      }
    }
  }
  
  return 0;
}

static int ahci_handle_irq(excp_entry_t *excp, excp_vec_t vec, void *state) {
  ahci_controller_state_t *c = state;

  DEBUG("handling irq: vector 0x%x rip: 0x%p s: 0x%p\n", vec, excp->rip, state);

  uint32_t is = c->abar->is;
  c->abar->is = is;
  for (int i = 0; (i < HBA_MAX_PORTS) && is; i++) {
    if (is & 1) {
      DEBUG("irq routed to port %d\n", i);

      if (port_handle_irq(&c->ports[i])) {
        ERROR("[port %d] failed to handle irq\n", i);
        IRQ_HANDLER_END();
        return -1;
      }
      is <<= 1;
    }
  }

  IRQ_HANDLER_END();
  return 0;
}

int nk_ahci_deinit() {
  INFO("deinited and leaking\n");
  return 0;
}

int nk_ahci_init(struct naut_info* naut) {
  INFO("init\n");

  // allocate and initialize the ahci controller
  ahci_controller_state_t *c = malloc(sizeof(ahci_controller_state_t));
  if (!c) {
    ERROR("could not allocate controller\n");
    return -1;
  }
  memset(c, 0, sizeof(ahci_controller_state_t));

  if (controller_init(c, naut)) {
    INFO("could not find an ahci controller\n");
    return -1;
  }

  // currently only support 64 bit addressing
  if (!c->abar->cap.s64a) {
    ERROR("controller does not support 64 bit addressing\n");
    return -1;
  }

  controller_reset(c);
  
  // set up interrupts and enable them, also enable ahci while we're at it
  if (controller_irq_setup(c)) {
    ERROR("could not setup interrupts for controller\n");
    return -1;
  }
  c->abar->ghc |= (HBA_GHC_AHCI_ENABLE | HBA_GHC_IRQ_ENABLE);

  // probe implemented ports for devices and initialize/identify any we find 
  if (probe(c)) {
    ERROR("error probing ports\n");
    return -1;
  }

  return 0;
}

static int get_characteristics(void *state, struct nk_block_dev_characteristics *c) {
  ahci_port_state_t *p = (ahci_port_state_t *)state;

  DEBUG("[port %d] get characteristics\n", p->num);

  if (!p->blkdev) {
    ERROR("[port %d] could not get characteristics, device was never identified\n", p->num);
    return -1;
  }

  *c = ((ahci_port_state_t *)state)->chars;
  return 0;
}

static int read_blocks(void *state, uint64_t blocknum, uint64_t count, uint8_t *dest, void (*callback)(nk_block_dev_status_t status, void *context), void *context) {
  ahci_port_state_t *p = (ahci_port_state_t *)state;

  DEBUG("[port %d] read %llu blocks starting at 0x%x\n", p->num, count, blocknum);

  if (!p->blkdev) {
    ERROR("[port %d] could not read blocks, device was never identified\n", p->num);
    return -1;
  }
  
  fis_h2d_t cmdfis;
  memset(&cmdfis, 0, sizeof(fis_h2d_t));
  cmdfis.fis_type = FIS_TYPE_REG_H2D;
  cmdfis.c = 1;
  cmdfis.command = ATA_CMD_READ_DMA_EXT;

  fis_readwrite_set_lba(&cmdfis, blocknum, count);

  ahci_cmd_continuation_t cont;
  cont.cb = callback;
  cont.context = context;

  return port_issue_cmd(p, cmdfis, cont, dest, count);
}

static int write_blocks(void *state, uint64_t blocknum, uint64_t count, uint8_t *src, void (*callback)(nk_block_dev_status_t status, void *context), void *context) {
  ahci_port_state_t *p = (ahci_port_state_t *)state;
  uint64_t src_uint = (uint64_t)src;

  DEBUG("[port %d] write %llu blocks starting at 0x%x\n", p->num, count, blocknum);

  if (!p->blkdev) {
    ERROR("[port %d] could not write blocks, device was never identified\n", p->num);
    return -1;
  }
  
  fis_h2d_t cmdfis;
  memset(&cmdfis, 0, sizeof(fis_h2d_t));
  cmdfis.fis_type = FIS_TYPE_REG_H2D;
  cmdfis.c = 1;
  cmdfis.command = ATA_CMD_WRITE_DMA_EXT;

  fis_readwrite_set_lba(&cmdfis, blocknum, count);

  ahci_cmd_continuation_t cont;
  cont.cb = callback;
  cont.context = context;

  return port_issue_cmd(p, cmdfis, cont, src, count);
}
