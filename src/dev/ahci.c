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
  volatile struct {
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
    uint32_t ssts;		         // 0x28, SATA status (SCR0:SStatus)
    uint32_t sctl;		         // 0x2C, SATA control (SCR2:SControl)
    uint32_t serr;		         // 0x30, SATA error (SCR1:SError)
    uint32_t sact;		         // 0x34, SATA active (SCR3:SActive)
    uint32_t ci;		           // 0x38, command issue
    uint32_t sntf;		         // 0x3C, SATA notification (SCR4:SNotification)
    uint32_t fbs;		           // 0x40, FIS-based switch control
    uint32_t rsvd1[11];	       // 0x44 ~ 0x6F, Reserved
    uint32_t vendor[4];	       // 0x70 ~ 0x7F, vendor specific
  } __attribute__((packed)) ports[32];  
} __attribute__((packed)) hba_reg_t;


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

int nk_ahci_init(struct naut_info* naut) {
  struct pci_info *pci = naut->sys.pci;
  struct list_head *curbus, *curdev;
  uint32_t num = 0;

  INFO("init\n");

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