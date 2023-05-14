#include <nautilus/nautilus.h>
#include <nautilus/cpu.h>
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

// AHCI Controller
#define INTEL_VENDOR_ID     0x8086  // intel
#define ICH9_DEVICE_ID      0x2922  // ICH9
#define CONTROLLER_CLASS    0x01    // mass storage device
#define CONTROLLER_SUBCLASS 0x06    // serial ATA

#define PORT_MASK(p, field) (((field) & (1u << (p))) >> (p))

#define HBA_PORT_IPM_ACTIVE 1
#define HBA_PORT_DET_DETECTED 3

typedef volatile struct {
    union {
      uint64_t val;
      volatile struct {
        uint32_t clbl;  // 0x00, command list base address lower 32 bits, 1K-byte aligned
        uint32_t clbu;  // 0x04, command list base address upper 32 bits
      } __attribute__((packed));
    } clb;                     // command list base address

    union {
      uint64_t val;
      volatile struct {
        uint32_t fbl; // 0x08, FIS base address lower 32 bits, 256-byte aligned
        uint32_t fbu; // 0x0C, FIS base address upper 32 bits
      } __attribute__((packed));
    } fb;                      // fis base address 

    uint32_t is;		           // 0x10, interrupt status
    uint32_t ie;		           // 0x14, interrupt enable
    uint32_t cmd;		           // 0x18, command and status
    uint32_t rsvd0;		         // 0x1C, Reserved
    uint32_t tfd;		           // 0x20, task file data
    uint32_t sig;		           // 0x24, signature
    union {
      uint32_t val;
      volatile struct {
        uint8_t det   : 4;    // Device detection
        uint8_t spd   : 4;    // Current interface speed
        uint8_t ipm   : 4;    // Interface power management
        uint32_t rsvd : 20;
      } __attribute__((packed));
    } ssts;                    // 0x28, SATA status (SCR0:SStatus)
    uint32_t sctl;		         // 0x2C, SATA control (SCR2:SControl)
    uint32_t serr;		         // 0x30, SATA error (SCR1:SError)
    uint32_t sact;		         // 0x34, SATA active (SCR3:SActive)
    uint32_t ci;		           // 0x38, command issue
    uint32_t sntf;		         // 0x3C, SATA notification (SCR4:SNotification)
    uint32_t fbs;		           // 0x40, FIS-based switch control
    uint32_t rsvd1[11];	       // 0x44 ~ 0x6F, Reserved
    uint32_t vendor[4];	       // 0x70 ~ 0x7F, vendor specific
} __attribute__((packed)) hba_port_reg_t;

typedef volatile struct {
  // 0x00 - 0x2B, Generic Host Control
  union {
    uint32_t val;
    volatile struct {
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
      uint8_t rsvd  : 1;  // reserved
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
	uint8_t rsvd[0xA0-0x2C];   
  
  // 0xA0 - 0xFF, Vendor specific registers 
	uint8_t vendor[0x100-0xA0];

  // 0x100 - 0x10FF, Port control registers 1 ~ 32 
  hba_port_reg_t ports[32];
} __attribute__((packed)) hba_reg_t;


typedef enum {
  SATA_SIG_ATA   = 0x00000101, // SATA drive
  SATA_SIG_ATAPI = 0xEB140101, // SATAPI drive
  SATA_SIG_SEMB  = 0xC33C0101, // Encolsure management bridge
  SATA_SIG_PM    = 0x96690101,  // Port multiplier
} sata_sig_t;

typedef enum {
  AHCI_DEV_NULL   = 0,
  AHCI_DEV_SATA   = 1,
  AHCI_DEV_SEMB   = 2,
  AHCI_DEV_PM     = 3,
  AHCI_DEV_SATAPI = 4,
} ahci_dev_t;

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
  uint8_t lba0[3];  // LBA low 7:0, mid 15:8, high 23:16
	uint8_t device;		// Device register
 
	// DWORD 2
  uint8_t lba1[3];  // LBA low 31:24, LBA mid 39:32, LBA high47:40
	uint8_t featureh;	// Feature register, 15:8
 
	// DWORD 3
	uint8_t countl;		// Count register, 7:0
	uint8_t counth;		// Count register, 15:8
	uint8_t icc;		  // Isochronous command completion
	uint8_t control;	// Control register
 
	// DWORD 4
	uint8_t rsv1[4];	// Reserved
} __attribute__((packed)) fis_h2d_t;

typedef struct {
  // DWORD 0
	uint8_t fis_type;    // FIS_TYPE_REG_D2H
 
	uint8_t pmport:4;    // Port multiplier
	uint8_t rsv0:2;      // Reserved
	uint8_t i:1;         // Interrupt bit
	uint8_t rsv1:1;      // Reserved
 
	uint8_t status;      // Status register
	uint8_t error;       // Error register
 
	// DWORD 1
  uint8_t lba0[3];      // LBA low 7:0, mid 15:8, high 23:16
	uint8_t device;      // Device register
 
	// DWORD 2
  uint8_t lba1[3];      // LBA low 31:24, LBA mid 39:32, LBA high47:40
	uint8_t rsv2;        // Reserved
 
	// DWORD 3
	uint8_t countl;      // Count register, 7:0
	uint8_t counth;      // Count register, 15:8
	uint8_t rsv3[2];     // Reserved
 
	// DWORD 4
	uint8_t  rsv4[4];     // Reserved
} __attribute__((packed)) fis_d2h_t;

typedef struct {
	// DWORD 0
	uint8_t fis_type;	// FIS_TYPE_DATA
 
	uint8_t pmport:4;	// Port multiplier
	uint8_t rsv0:4;		// Reserved
 
	uint8_t rsv1[2];	// Reserved
 
	// DWORD 1 ~ N
	uint32_t data[1];	// Payload
} __attribute__((packed)) fis_data_t;

typedef struct {
  // DWORD 0
	uint8_t fis_type;	      // FIS_TYPE_DMA_SETUP
 
	uint8_t pmport:4;	      // Port multiplier
	uint8_t rsv0:1;		      // Reserved
	uint8_t d:1;		        // Data transfer direction, 1 - device to host
	uint8_t i:1;		        // Interrupt bit
	uint8_t a:1;            // Auto-activate. Specifies if DMA Activate FIS is needed
 
  uint8_t rsv[2];       // Reserved
 
	//DWORD 1&2
  uint64_t DMAbufferID;   // DMA Buffer Identifier. Used to Identify DMA buffer in host memory.
                          // SATA Spec says host specific and not in Spec. Trying AHCI spec might work.

  //DWORD 3
  uint32_t rsv1;          // More reserved

  //DWORD 4
  uint32_t DMAbufOffset;  // Byte offset into buffer. First 2 bits must be 0

  //DWORD 5
  uint32_t TransferCount; // Number of bytes to transfer. Bit 0 must be 0

  //DWORD 6
  uint32_t rsv2;          // Reserved
} __attribute__((packed)) fis_dma_setup_t;

struct ahci_blkdev_state {
  struct nk_blk_dev *blkdev;
};

struct ahci_controller_state {
  hba_reg_t *abar;   // ahci base address register
  
  // struct pci_dev *pci_dev;   // pci interrupt and interrupt vector
  // struct list_head node;     // device list

  // uint64_t  mem_start;       // Where registers are mapped into the physical memory address space
  // uint64_t  mem_end;
};

static struct list_head dev_list;
static struct ahci_controller_state controller;

int get_dev_type(hba_port_reg_t *port) {
  if (port->ssts.det != HBA_PORT_DET_DETECTED) return AHCI_DEV_NULL;
  if (port->ssts.ipm != HBA_PORT_IPM_ACTIVE)   return AHCI_DEV_NULL;

  switch (port->sig) {
    case SATA_SIG_ATAPI: return AHCI_DEV_SATAPI;
    case SATA_SIG_SEMB:  return AHCI_DEV_SEMB;
    case SATA_SIG_PM:    return AHCI_DEV_PM;
    default:             return AHCI_DEV_SATA;
  }
}

int controller_init(struct naut_info* naut) {
  struct pci_info *pci = naut->sys.pci;
  struct list_head *curbus, *curdev;
  uint32_t num = 0;

  INIT_LIST_HEAD(&dev_list);

  if (!pci) {
    ERROR("no PCI info\n");
    return -1;
  }

  DEBUG("finding ahci controller\n");

  list_for_each(curbus,&(pci->bus_list)) {
    struct pci_bus *bus = list_entry(curbus, struct pci_bus, bus_node);

    DEBUG("searching PCI bus %u for ahci controller\n", bus->num);

    list_for_each(curdev, &(bus->dev_list)) {
      struct pci_dev *pdev = list_entry(curdev, struct pci_dev, dev_node);
      struct pci_cfg_space *cfg = &pdev->cfg;
      
      DEBUG("device %u is a 0x%x:0x%x\n", pdev->num, cfg->vendor_id, cfg->device_id);
  
      if (cfg->class_code == CONTROLLER_CLASS && cfg->subclass  == CONTROLLER_SUBCLASS && cfg->vendor_id  == INTEL_VENDOR_ID  && cfg->device_id == ICH9_DEVICE_ID) {
        DEBUG("found an achi controller\n");
          
        if (cfg->hdr_type != 0x0) {
          ERROR("unexpected device type\n");
          return -1;
        }

        controller.abar = (hba_reg_t *)pci_dev_get_bar_addr(pdev, 5);
        
        DEBUG("device has %u ports and implements ports (one-hot): %x\n", controller.abar->cap.np, controller.abar->pi);
      }
    }
  }
  
  return 0;
}

int probe_ports() {
  DEBUG("probing ahci ports\n");

  uint32_t pi = controller.abar->pi;
  for (int i = 0; i < 32; i++) {
    if (pi & 1) { 
      int devty = get_dev_type(&controller.abar->ports[i]);
      switch (devty) {
        case AHCI_DEV_SATA: {
          INFO("SATA drive found at port %d\n", i);
          break;
        } case AHCI_DEV_SATAPI: {
          INFO("SATAPI drive found at port %d\n", i);
          break;
        } case AHCI_DEV_SEMB: {
          INFO("Enclosure management bridge found at port %d\n", i);
          break;
        } case AHCI_DEV_PM: {
          INFO("Port multiplier found at port %d\n", i);
          break;
        } default: break;
      }
      pi >>= 1;
    }
  }

  return 0;
}

int nk_ahci_init(struct naut_info* naut) {
  INFO("init\n");

  if (controller_init(naut)) return -1;
  if (probe_ports())         return -1;

  return 0;
}