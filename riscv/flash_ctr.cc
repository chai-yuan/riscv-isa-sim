#include "devices.h"
#include "dts.h"
#include "mmu.h"
#include "processor.h"
#include "sim.h"
#include <sstream>
#include <sys/time.h>

flash_ctr_t::flash_ctr_t(mem_t *initrd) : initrd(initrd) {
    this->flashcmd  = 0;
    this->flashaddr = 0;
    this->data_idx  = 0;
}

bool flash_ctr_t::load(reg_t addr, size_t len, uint8_t *bytes) {
    uint8_t val;
    bool    ret = true, update = false;

    if (addr + len > PGSIZE) {
        return false;
    }
    addr = addr & 0x4;

    switch (addr) {
    case 0x0:
        if (data_idx > 3) {
            initrd->load(flashaddr, 1, &val);
            flashaddr++;
        } else {
            val = 0;
        }
        break;
    case 0x4:
        val = 0;
        break;
    default:
        ret = false;
        break;
    };

    if (ret) {
        bytes[0] = val;
    }

    return ret;
}

bool flash_ctr_t::store(reg_t addr, size_t len, const uint8_t *bytes) {
    uint8_t val;
    bool    ret = true, update = false;

    if (addr + len > PGSIZE) {
        return false;
    }
    addr = addr & 0x4;
    val  = bytes[0];

    switch (addr) {
    case 0x0:
        if (data_idx == 0) {
            flashcmd = val;
        } else if (data_idx > 0 && data_idx <= 3) {
            flashaddr = flashaddr << 8;
            flashaddr |= val;
        } else {
            initrd->store(flashaddr, 1, &val);
            flashaddr++;
        }
        data_idx++;
        break;
    case 0x4:
        if (val == 0) {
        } else {
            this->flashcmd  = 0;
            this->flashaddr = 0;
            this->data_idx  = 0;
        }
        break;
    default:
        ret = false;
        break;
    };

    return ret;
}

std::string flash_generate_dts(const sim_t *sim, const std::vector<std::string> &UNUSED sargs) {
    return std::string("");
}

flash_ctr_t *flash_parse_from_fdt(const void *fdt, const sim_t *sim, reg_t *base,
                                  const std::vector<std::string> &UNUSED sargs) {
    printf("hello from flash_ctr_t!\n");
    return nullptr;
}

REGISTER_DEVICE(flash_ctr, flash_parse_from_fdt, flash_generate_dts)
