#include <nautilus/nautilus.h>
#include <nautilus/cpu.h>
#include <nautilus/timer.h>
#include <nautilus/dev.h>
#include <nautilus/mm.h>
#include <nautilus/irq.h>
#include <nautilus/naut_string.h>
#include <nautilus/blkdev.h>

#include <dev/pci.h>
#include <dev/ahci.h>

#ifndef NAUT_CONFIG_DEBUG_AHCI
#undef DEBUG_PRINT
#define DEBUG_PRINT(fmt, args...) 
#endif

#define ERROR(fmt, args...) ERROR_PRINT("ahci: " fmt, ##args)
#define DEBUG(fmt, args...) DEBUG_PRINT("ahci: " fmt, ##args)
#define INFO(fmt, args...) INFO_PRINT("ahci: " fmt, ##args)

// atomic load from pointer with sequential consistency
#define atomic_load(srcptr)         __atomic_load_n(srcptr,__ATOMIC_SEQ_CST)

// atomic store to pointer with sequenial consistency
#define atomic_store(destptr,value) __atomic_store_n(destptr,value,__ATOMIC_SEQ_CST)

// AHCI Controller
#define INTEL_VENDOR_ID     0x8086  // intel
#define ICH9_DEVICE_ID      0x2922  // ICH9
#define CONTROLLER_CLASS    0x01    // mass storage device
#define CONTROLLER_SUBCLASS 0x06    // serial ATA

#define CMD_LIST_SIZE 32
#define CMD_TBL_NUM_PRDT 1

#define HBA_MAX_PORTS 32
#define HBA_PORT_IPM_ACTIVE 1
#define HBA_PORT_DET_DETECTED 3

typedef enum {
  SATA_SIG_ATA   = 0x00000101,  // SATA drive
  SATA_SIG_ATAPI = 0xEB140101,  // SATAPI drive
  SATA_SIG_SEMB  = 0xC33C0101,  // Encolsure management bridge
  SATA_SIG_PM    = 0x96690101,  // Port multiplier
} sata_sig_t;

typedef enum {
  AHCI_DEV_NULL   = 0,
  AHCI_DEV_SATA   = 1,
  AHCI_DEV_SEMB   = 2,
  AHCI_DEV_PM     = 3,
  AHCI_DEV_SATAPI = 4,
} ahci_dev_t;

// FRAME INFORMATION STRUCTURE DECLARATIONS

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
  uint8_t countl;		// Count register, 7:0
  uint8_t counth;		// Count register, 15:8
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
	uint8_t  fis_type;	// FIS_TYPE_PIO_SETUP
 
	uint8_t  pmport:4;	// Port multiplier
	uint8_t  rsv0:1;		// Reserved
	uint8_t  d:1;		// Data transfer direction, 1 - device to host
	uint8_t  i:1;		// Interrupt bit
	uint8_t  rsv1:1;
 
	uint8_t  status;		// Status register
	uint8_t  error;		// Error register
 
	// DWORD 1
	uint8_t  lba0;		// LBA low register, 7:0
	uint8_t  lba1;		// LBA mid register, 15:8
	uint8_t  lba2;		// LBA high register, 23:16
	uint8_t  device;		// Device register
 
	// DWORD 2
	uint8_t  lba3;		// LBA register, 31:24
	uint8_t  lba4;		// LBA register, 39:32
	uint8_t  lba5;		// LBA register, 47:40
	uint8_t  rsv2;		// Reserved
 
	// DWORD 3
	uint8_t  countl;		// Count register, 7:0
	uint8_t  counth;		// Count register, 15:8
	uint8_t  rsv3;		// Reserved
	uint8_t  e_status;	// New value of status register
 
	// DWORD 4
	uint16_t tc;		// Transfer count
	uint8_t  rsv4[2];	// Reserved
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

typedef struct {
  // DWORD 0
  uint8_t fis_type;	// FIS_TYPE_DATA
 
  uint8_t pmport:4;	// Port multiplier
  uint8_t rsv0:4;		// Reserved
 
  uint8_t rsv1[2];	// Reserved
 
  // DWORD 1 ~ N
  uint32_t data[1];	// Payload
} __attribute__((packed)) fis_data_t;

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
  uint8_t sdbfis;		     // Set device bit FIS
 
  // 0x60
  uint8_t ufis[64];      // Unknown FIS
 
  // 0xA0
  uint8_t rsv[0x100-0xA0];
} __attribute__((packed)) fis_hba_t;

// COMMAND LIST DECLARATIONS

typedef enum {
  ATA_CMD_IDENTIFY_DMA = 0xEE,
  ATA_CMD_IDENTIFY_PIO = 0xEC,
} ata_cmd_t;

typedef struct {
  // 0x00
  fis_h2d_t cfis;      // Command FIS

  // 0x40
  uint8_t acmd[16];    // ATAPI command
  
  // 0x50
  uint8_t rsv[48];	   // Reserved

  // 0x80
  struct {
    union {
      void *dba;
      struct {
        uint32_t dba_l;		   // Data base address lower 32 bits
        uint32_t dba_u;		   // Data base address upper 32 bits
      } __attribute__((packed));
    };                 // Data base address

    uint32_t rsv0;		 // Reserved
    uint32_t dbc:22;	 // Byte count, 4M max
    uint32_t rsv1:9;	 // Reserved
    uint32_t i:1;		   // Interrupt on completion
  } __attribute__((packed)) prdt_entry[CMD_TBL_NUM_PRDT];  // Physical region descriptor table entries, 0 ~ 65535
} __attribute__((packed)) cmd_table_t;


// #define CMDHDR_CLR_BUSY = (1 << 10)
// #define CMDHDR_BIST = (1 << 9)
// #define CMDHDR_RESET (1 << 8)
// #define CMDHDR_PREFETCH = (1 << 7)
// #define CMDHDR_WRITE = (1 << 6)
// #define CMDHDR_ATAPI = (1 << 5)

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

// AHCI REGISTER DECLARATIONS

#define PORT_CMD_RUNNING (1 << 15)
#define PORT_CMD_FIS (1 << 4)
#define PORT_CMD_START 1

#define PORT_IS_TF_ERR    (1 << 30)
#define PORT_IS_PRD       (1 << 5)
#define PORT_IS_DMA_SETUP (1 << 2)
#define PORT_IS_PIO_SETUP (1 << 1)
#define PORT_IS_D2H        1

#define PORT_IE_TF_ERR (1 << 30)
#define PORT_IE_OVERFLOW (1 << 24)
#define PORT_IE_PRD_PROCESSED (1 << 5)
#define PORT_IE_DMA_SETUP (1 << 2)
#define PORT_IE_PIO_SETUP (1 << 1)
#define PORT_IE_D2H 1

#define PORT_TFD_BUSY 0x80
#define PORT_TFD_DATA_REQ 0x08

typedef volatile struct {
    union {
      cmd_hdr_t *clb;
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

    volatile uint32_t is;               // 0x10, interrupt status  
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
    } cmd;		    // 0x18, command and status
    
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
    uint32_t rsv1[11];	       // 0x44 ~ 0x6F, Reserved
    uint32_t vendor[4];	       // 0x70 ~ 0x7F, vendor specific
} __attribute__((packed)) hba_port_reg_t;

#define HBA_GHC_RESET        1
#define HBA_GHC_IRQ_ENABLE  (1 << 1)
#define HBA_GHC_AHCI_ENABLE (1 << 31)

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

// BLOCK DEVICE INTERFACE

static int get_characteristics(void *state, struct nk_block_dev_characteristics *c);
static int read_blocks(void *state, uint64_t blocknum, uint64_t count, uint8_t *dest,void (*callback)(nk_block_dev_status_t, void *), void *context);
static int write_blocks(void *state, uint64_t blocknum, uint64_t count, uint8_t *src,void (*callback)(nk_block_dev_status_t, void *), void *context);

static struct nk_block_dev_int interface = 
{
    .get_characteristics = get_characteristics,
    .read_blocks = read_blocks,
    .write_blocks = write_blocks,
};

// LOCAL STATE DECLARATIONS

typedef void (*cmd_callback_t)(nk_block_dev_status_t, void *);

typedef struct ahci_cmd_state {
  // marks whether the command is currently running on the device
  uint8_t running;
  // to call when the command finishes
  cmd_callback_t callback;
  // context for the callback
  void *context;
} ahci_cmd_state_t;

typedef struct ahci_port_state {
  struct nk_block_dev *blkdev;
  struct nk_block_dev_characteristics chars;
  ahci_dev_t type;

  uint8_t portn;
  hba_port_reg_t *regs;

  spinlock_t cmdlock;
  // stores state for each command in regs->clb
  struct ahci_cmd_state cmd_state[CMD_LIST_SIZE];
  
  struct ahci_controller_state *controller;
} ahci_port_state_t;

typedef struct ahci_controller_state {
  hba_reg_t *abar;  // ahci base address register

  struct pci_dev *pdev;

  struct ahci_port_state devs[HBA_MAX_PORTS];
} ahci_controller_state_t;

// static struct ahci_controller_state controller;

// FUNCTION DEFINITIONS

static int controller_init(struct naut_info* naut, ahci_controller_state_t *s) {
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
  
      if (cfg->vendor_id == INTEL_VENDOR_ID && cfg->device_id == ICH9_DEVICE_ID) {
        DEBUG("found an ahci controller\n");
          
        if (cfg->hdr_type != 0x0) {
          ERROR("unexpected device type\n");
          return -1;
        }

        s->abar = (hba_reg_t *)pci_dev_get_bar_addr(pdev, 5);
        s->pdev = pdev;

        // allow device to use DMA
        uint16_t cmdreg = pci_cfg_readw(bus->num, pdev->num, 0, 0x4);
        cmdreg |= 0x7; // make sure bus master is enabled
        pci_cfg_writew(bus->num, pdev->num, 0, 0x4, cmdreg);

        uint16_t stat = pci_cfg_readw(bus->num, pdev->num, 0, 0x6);
      }
    }
  }
  
  return 0;
}

static int get_completed_cmd(ahci_port_state_t *s) {
  uint32_t ci = s->regs->ci;

  int slot;
  for (slot = 0; slot < CMD_LIST_SIZE; slot++) {
    // DEBUG("command list %d is running?: %d\n", slot, s->cmd_state[slot].running);
    if (s->cmd_state[slot].running && !(ci & 1)) return slot; 
    ci >>= 1;
  }

  return -1;
}

static int port_handle_irq(ahci_port_state_t *s) {

  uint32_t is = s->regs->is;
  s->regs->is = is;

  // check for task file errors
  if (is & PORT_IS_TF_ERR) {
    ERROR("task file error\n");
    return -1;
  }

  // prdt finished processing
  if (is & PORT_IS_PRD) {
    DEBUG("finished transferring prdt to controller\n");
    return 0;
  }

  int slot = get_completed_cmd(s);
  if (slot == -1) {
      DEBUG("no commands have been completed so this interrupt is unexpected, ignoring\n");
  } 

  // PIO setup interrupt 
  else if (is & PORT_IS_PIO_SETUP) {
    DEBUG("interrupt was for a PIO setup command\n");
    uint8_t flags = spin_lock_irq_save(&s->cmdlock);
    s->cmd_state[slot].running = 0;
    spin_unlock_irq_restore(&s->cmdlock, flags);
    DEBUG("slot: %d\n", slot);

    if (s->cmd_state[slot].callback) {
      s->cmd_state[slot].callback(NK_BLOCK_DEV_STATUS_SUCCESS, s->cmd_state[slot].context);
      s->regs->ci &= ~(1U << slot);
    }
  }

  // device register update
  else if (is & PORT_IS_D2H) {
    DEBUG("interrupt was for a d2h command\n");
    // mark the command as no longer running
    uint8_t flags = spin_lock_irq_save(&s->cmdlock);
    s->cmd_state[slot].running = 0;
    spin_unlock_irq_restore(&s->cmdlock, flags);

    // determine command status
    fis_d2h_t *fis = &s->regs->fb->rfis;
    nk_block_dev_status_t status;
    if (!fis->status.err && !fis->status.bsy && fis->status.rdy) {
      status = NK_BLOCK_DEV_STATUS_SUCCESS;
    } else {
      INFO("command for received irq was unsuccessful\n");
      status = NK_BLOCK_DEV_STATUS_ERROR;
    }

    // call the saved callback 
    if (s->cmd_state[slot].callback) {
      s->cmd_state[slot].callback(status, s->cmd_state[slot].context);
      // allow future commands to use this slot
      s->regs->ci &= ~(1U << slot);
    }
  }

  return 0;
}

static ahci_dev_t get_dev_type(hba_port_reg_t *regs) {
  if (regs->ssts.det != HBA_PORT_DET_DETECTED) return AHCI_DEV_NULL;
  if (regs->ssts.ipm != HBA_PORT_IPM_ACTIVE)   return AHCI_DEV_NULL;

  switch (regs->sig) {
    case SATA_SIG_ATAPI: return AHCI_DEV_SATAPI;
    case SATA_SIG_SEMB:  return AHCI_DEV_SEMB;
    case SATA_SIG_PM:    return AHCI_DEV_PM;
    default:             return AHCI_DEV_SATA;
  }
}

static int find_cmd_slot(ahci_port_state_t *s) {
  uint32_t slots = s->regs->ci | s->regs->sact;
  for (int slot = 0; slot < s->controller->abar->cap.ncs; slot++) {
    if ((slots & 1) == 0) {
      return slot;
    }
    slots >>= 1;
  }

  return -1;
}


// issue an ATA command
// buf: data buffer
// len: length of data buffer in bytes
static int port_issue_cmd(ahci_port_state_t *s, fis_h2d_t *cmd, void *buf, size_t len, cmd_callback_t callback, void *cb_context) {
  if (len > CMD_TBL_NUM_PRDT * (1LLU << 22)) {
    // we can only store 4 MiB of data per prdt entry and currently only 1 prdt entry is used
    ERROR("command result buffer length must be <= 4 MiB");
    return -1;
  }

  int slot = find_cmd_slot(s);
  if (slot == -1) {
    ERROR("could not find unclaimed slot for new command\n");
    return -1;
  }

  DEBUG("port %d: issuing command in slot %d\n", s->portn, slot);

  cmd_hdr_t *cl_entry = &s->regs->clb[slot];
  // copy the command into the command table
  memcpy(&cl_entry->ctba->cfis, cmd, sizeof(fis_h2d_t));

  cl_entry->ctba->prdt_entry[0].dba = buf;
  // this field is 0 indexed so we subtract 1
  cl_entry->ctba->prdt_entry[0].dbc = len == 0 ? len - 1 : 0;
  // interrupt when command completes
  cl_entry->ctba->prdt_entry[0].i = 1;

  cl_entry->cfl = sizeof(cmd) / 4;
  cl_entry->a = 0;
  cl_entry->w = 0;
  cl_entry->p = 0;
  cl_entry->r = 0;
  cl_entry->b = 0;
  cl_entry->c = 1;
  cl_entry->prdtl = CMD_TBL_NUM_PRDT;
  cl_entry->prdbc = 0;

  // issue the command and save state for when it finishes
  uint8_t flags = spin_lock_irq_save(&s->cmdlock);
  s->cmd_state[slot].running = 1;
  s->cmd_state[slot].callback = callback;
  s->cmd_state[slot].context = cb_context;
  s->regs->ci |= (1U << slot);
  spin_unlock_irq_restore(&s->cmdlock, flags);

  // while (1) {
  //   uint16_t stat = pci_dev_cfg_readw(s->controller->pdev, 0x6);
  //   DEBUG("PCI status: 0x%04x\n", stat);
  //   DEBUG("GHC interrupt status: %x\n", s->controller->abar->is);
  //   DEBUG("Port interrupt status: %x\n", s->regs->is.val);
  //   DEBUG("GHC interrupt status: %x\n", atomic_load(&s->controller->abar->is));
  //   DEBUG("Port interrupt status: %x\n", atomic_load(&s->regs->is.val));
  // }

  return 0;
}

static int ahci_irq_handler(excp_entry_t *excp, excp_vec_t vec, void *state) {
  ahci_controller_state_t *s = state;

  DEBUG("handling irq: vector 0x%x rip: 0x%p s: 0x%p\n", vec, excp->rip, state);
  DEBUG("controller interrupt status: %x\n", s->abar->is);

  uint32_t is = s->abar->is;

  for (int i = 0; i < HBA_MAX_PORTS; i++) {
    uint32_t mask = 1 << i;
    if (is & mask) {
      DEBUG("port %d: interrupt status: %x\n", i, s->devs[i].regs->is);

      if (port_handle_irq(&s->devs[i])) {
        return -1;
      }
    }
  }

  s->abar->is = is;
  IRQ_HANDLER_END();
  return 0;
} 

void identify_callback(nk_block_dev_status_t status, void *context) {
  ahci_port_state_t *s = context;
  fis_pio_setup_t *fis = &s->regs->fb->psfis;

  DEBUG("WE CALLED THE CALLBACK AND OUR TC IS %d\n", fis->tc);

  uint16_t *buf = s->regs->clb[0].ctba->prdt_entry[0].dba;

  // for (int i = 0; i < 256; i++) {
  //   DEBUG("field %d: %x\n", i, buf[i]);
  // }
}

static int port_identify(ahci_port_state_t *s) {
  DEBUG("port %d: identifying sata device\n", s->portn);

  uint16_t *buf = malloc(256 * sizeof(uint16_t));
  memset(buf, 0, 256 * sizeof(uint16_t));

  s->regs->is = -1;

  uint8_t flags = spin_lock_irq_save(&s->cmdlock);
  
  int slot = find_cmd_slot(s);
  if (slot == -1) {
    ERROR("could not claim command slot\n");
  }

  cmd_hdr_t *cmdhdr = &s->regs->clb[slot];
  cmdhdr->cfl = sizeof(fis_h2d_t) / sizeof(uint32_t);
  cmdhdr->w = 0;
  cmdhdr->prdtl = 1;

  cmd_table_t *cmdtbl = cmdhdr->ctba;
  memset(cmdtbl, 0, sizeof(cmd_table_t));

  cmdtbl->prdt_entry[0].dba = buf;
  cmdtbl->prdt_entry[0].dbc = (256 * sizeof(uint16_t)) - 1;
  cmdtbl->prdt_entry[0].i = 0;

  fis_h2d_t *cmdfis = &cmdtbl->cfis;
  memset(cmdfis, 0, sizeof(fis_h2d_t));
  cmdfis->fis_type = FIS_TYPE_REG_H2D;
  cmdfis->command = ATA_CMD_IDENTIFY_PIO;
  cmdfis->c = 1;

  size_t spin = 0;
  while ((s->regs->tfd & (PORT_TFD_BUSY | PORT_TFD_DATA_REQ)) && spin < 1000000) {
    spin++;
  }
  if (spin == 1000000) {
    ERROR("port is hung\n");
    return -1;
  }

  DEBUG("command list entry %d is now running\n", slot);
  s->cmd_state[slot].running = 1;
  s->cmd_state[slot].callback = identify_callback;
  s->cmd_state[slot].context = s;

  s->regs->ci |= (1 << slot);

  spin_unlock_irq_restore(&s->cmdlock, flags);

  return 0;
}

static int port_identify_blocking(ahci_port_state_t *s) {
  // currently only support SATA devices

  DEBUG("port %d: identifying sata device\n", s->portn);

  uint16_t *buf = malloc(256 * sizeof(uint16_t));
  memset(buf, 0, 256 * sizeof(uint16_t));

  s->regs->is = -1;

  uint8_t flags = spin_lock_irq_save(&s->cmdlock);
  
  int slot = find_cmd_slot(s);
  if (slot == -1) {
    ERROR("could not claim command slot\n");
  }

  cmd_hdr_t *cmdhdr = &s->regs->clb[slot];
  cmdhdr->cfl = sizeof(fis_h2d_t) / sizeof(uint32_t);
  cmdhdr->w = 0;
  cmdhdr->prdtl = 1;

  cmd_table_t *cmdtbl = cmdhdr->ctba;
  memset(cmdtbl, 0, sizeof(cmd_table_t));

  cmdtbl->prdt_entry[0].dba = buf;
  cmdtbl->prdt_entry[0].dbc = (256 * sizeof(uint16_t)) - 1;
  cmdtbl->prdt_entry[0].i = 0;

  fis_h2d_t *cmdfis = &cmdtbl->cfis;
  memset(cmdfis, 0, sizeof(fis_h2d_t));
  cmdfis->fis_type = FIS_TYPE_REG_H2D;
  cmdfis->command = ATA_CMD_IDENTIFY_PIO;
  cmdfis->c = 1;

  size_t spin = 0;
  while ((s->regs->tfd & (PORT_TFD_BUSY | PORT_TFD_DATA_REQ)) && spin < 1000000) {
    spin++;
  }
  if (spin == 1000000) {
    ERROR("port is hung\n");
    return -1;
  }

  s->regs->ci = 1 << slot;

  spin_unlock_irq_restore(&s->cmdlock, flags);

  while (1) {
    if ((s->regs->ci & (1 << slot)) == 0) break;

    if (s->regs->is & PORT_IS_TF_ERR) {
      ERROR("task file error\n");
      return -1;
    }
  }

  if (s->regs->is & PORT_IS_TF_ERR) {
    ERROR("task file error\n");
    return -1;
  }

  fis_pio_setup_t *rfis = &s->regs->fb->psfis;
  
  DEBUG("received FIS of type: %x\n", rfis->fis_type);
  DEBUG("transfer count: %d\n", rfis->tc);

  DEBUG("status register: %x\n", rfis->status);
  DEBUG("error register: %x\n", rfis->error);

  // nk_delay(1000000);

  // for (int i = 0; i < 256; i++) {
  //   DEBUG("field %d: %x\n", i, buf[i]);
  // }

  // if (!((buf[83] >> 10) & 0x1)) { 
  //   ERROR("LBA48 not supported on this drive\n");
  //   return -1;
  // } else {
  //   size_t num_blocks = 
  //     (((uint64_t) buf[103]) << 48) +
	//     (((uint64_t) buf[102]) << 32) +
	//     (((uint64_t) buf[101]) << 16) +
	//     (((uint64_t) buf[100]) <<  0) ;

  //   DEBUG("LBA48 supported, block data is %x %x %x %x\n", buf[103], buf[102], buf[101], buf[100]);
  //   DEBUG("Interpretted as numblocks=0x%x\n", num_blocks);
  // }


  return 0;
}

static void port_start_cmd(ahci_port_state_t *s) {
  while (s->regs->cmd.val & PORT_CMD_RUNNING);

  s->regs->cmd.val |= (PORT_CMD_START | PORT_CMD_FIS);
}

static void port_stop_cmd(ahci_port_state_t *s) {
  s->regs->cmd.val &= ~(PORT_CMD_START | PORT_CMD_FIS);

  while(s->regs->cmd.val & (PORT_CMD_START | PORT_CMD_FIS));
}

static void port_reset(ahci_port_state_t *s) {
  s->regs->sctl.det = 0x1;

  // sleep for 1 ms before clearing to ensure that device has time to read
  nk_delay(1000000);
  
  s->regs->sctl.det = 0x0;

  // wait for communication to be re-established
  while (s->regs->ssts.det != 0x3);

  s->regs->serr = -1;

  DEBUG("successfully reset port %d\n", s->portn);
}

static int port_init(ahci_controller_state_t *controller, int portn) {
  hba_port_reg_t *regs = &controller->abar->ports[portn];
  ahci_dev_t type = get_dev_type(regs);

  char name[32];
  sprintf(name, "ahci-%d-%d", portn, type);
  INFO("initializing block device: %s\n", name);

  // allocate command list
  uint8_t n_slots = controller->abar->cap.ncs;
  regs->clb = malloc(n_slots * sizeof(cmd_hdr_t));
  if (!regs->clb) {
    ERROR("could not allocate command list\n");
    return -1;
  }
  memset(regs->clb, 0, n_slots * sizeof(cmd_hdr_t));

  // allocate a command table in each header within the list 
  for (int i = 0; i < n_slots; i++) {
    regs->clb[i].ctba = malloc(sizeof(cmd_table_t));
    if (!regs->clb[i].ctba) {
      ERROR("could not allocate command table\n");
      return -1;
    }
    if((uint64_t)regs->clb[i].ctba & 0b1111111) {
      ERROR("command table was not 128 byte aligned\n");
      return -1;
    }
    memset(regs->clb[i].ctba, 0, sizeof(cmd_table_t));
  }

  // allocate device-to-host fis
  regs->fb = malloc(sizeof(fis_hba_t));
  if (!regs->fb) {
    ERROR("could not allocate frame information structure\n");
    return -1;
  }

  // set up device state
  ahci_port_state_t *s = &(controller->devs[portn]);
  s->blkdev = nk_block_dev_register(name, 0, &interface, s);
  s->type = type;
  s->portn = portn;
  s->regs = regs;
  s->controller = controller;

  return 0;
}

static int probe(ahci_controller_state_t *controller) {
  uint32_t pi = controller->abar->pi;
  DEBUG("probing ports for devices, implemented mask: %x\n", pi);

  for (int i = 0; pi & (i < HBA_MAX_PORTS); i++) {
    // only support SATA devices
    if ((pi & 1)) {
      switch (get_dev_type(&controller->abar->ports[i])) {
        case AHCI_DEV_SATA: {
          if (port_init(controller, i)) {
            ERROR("port %d: failed to initialize sata device\n", i);
            return -1;
          }
          
          ahci_port_state_t *dev = &controller->devs[i];
          port_reset(dev);
          port_start_cmd(dev);

          // enable interrupts
          // dev->regs->ie = (PORT_IE_D2H | PORT_IE_PIO_SETUP | PORT_IE_TF_ERR);
          dev->regs->ie = -1;
          
          if (port_identify(dev)) {
            ERROR("port %d: failed to identify sata device\n", i);
            return -1;
          }

          // if (port_identify_blocking(dev)) {
          //   ERROR("port %d: failed to identify sata device\n", i);
          //   return -1;
          // }

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

int nk_ahci_init(struct naut_info* naut) {
  INFO("init\n");

  // allocate and initialize the ahci controller
  ahci_controller_state_t *controller = malloc(sizeof(ahci_controller_state_t));
  if (!controller) {
    ERROR("could not allocate controller\n");
    return -1;
  }
  memset(controller, 0, sizeof(ahci_controller_state_t));

  if (controller_init(naut, controller)) {
    INFO("could not find an ahci controller\n");
    return -1;
  }

  // reset controller
  INFO("resetting controller\n");
  controller->abar->ghc |= HBA_GHC_RESET;
  while (controller->abar->ghc & HBA_GHC_RESET);

  // register controller's interrupt handler
  if (register_int_handler(0xe4, ahci_irq_handler, controller)) {
    ERROR("could not register interrupt handler for controller\n");
    return -1;
  }

  // enable ahci mode and interrupts
  controller->abar->ghc |= (HBA_GHC_IRQ_ENABLE | HBA_GHC_AHCI_ENABLE);
  for (int irq = 8; irq < 16; irq++) {
    nk_unmask_irq(irq);
  }

  // currently only support 64 bit addressing
  if (!controller->abar->cap.s64a) {
    ERROR("controller does not support 64 bit addressing\n");
    return -1;
  }

  // probe implemented ports for devices and initialize/identify any we find 
  if (probe(controller)) {
    ERROR("could not probe ports\n");
  }

  return 0;
}

int nk_ahci_deinit() {
  INFO("deinited and leaking\n");
  return 0;
}

static int get_characteristics(void *state, struct nk_block_dev_characteristics *c) {
  return 0;
}

static int read_blocks(void *state, uint64_t blocknum, uint64_t count, uint8_t *dest, void (*callback)(nk_block_dev_status_t, void *), void *context) {
  ahci_port_state_t *s = (ahci_port_state_t *)state;
  
  DEBUG("read_blocks on device %s starting at %lu for %lu blocks\n", s->blkdev->dev.name, blocknum, count);

  return 0;
}

static int write_blocks(void *state, uint64_t blocknum, uint64_t count, uint8_t *src, void (*callback)(nk_block_dev_status_t, void *), void *context) {
  return 0;
}