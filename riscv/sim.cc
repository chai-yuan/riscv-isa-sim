// See LICENSE for license details.

#include "sim.h"
#include "byteorder.h"
#include "config.h"
#include "dlfcn.h"
#include "dts.h"
#include "libfdt.h"
#include "mmu.h"
#include "platform.h"
#include "remote_bitbang.h"
#include "socketif.h"
#include "string"
#include <cassert>
#include <climits>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <signal.h>
#include <sstream>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

// --defftest--
DifftestRiscvCore *difftestCore, simCore;

struct DifftestRiscvCore *(*difftest_init)(const uint8_t *data, uint64_t data_size) = NULL;
void (*difftest_step)(void)                                                         = NULL;
void (*difftest_interrupt)(const uint64_t)                                          = NULL;

void difftestInit(const std::string &soFileName, const std::string &binFileName) {
    std::ifstream binFile(binFileName, std::ios::binary | std::ios::ate);
    if (!binFile) {
        std::cerr << "Error opening binary file: " << binFileName << std::endl;
        exit(1);
    }

    std::streamsize binFileSize = binFile.tellg();
    binFile.seekg(0, std::ios::beg);

    std::vector<uint8_t> binData(binFileSize);
    if (!binFile.read(reinterpret_cast<char *>(binData.data()), binFileSize)) {
        std::cerr << "Error reading binary file: " << binFileName << std::endl;
        exit(1);
    }

    void *handle = dlopen(soFileName.c_str(), RTLD_LAZY);
    if (!handle) {
        std::cerr << "Error loading library: " << dlerror() << std::endl;
        exit(1);
    }

    difftest_init = reinterpret_cast<DifftestRiscvCore *(*)(const uint8_t *, uint64_t)>(dlsym(handle, "difftest_init"));
    difftest_step = reinterpret_cast<void (*)()>(dlsym(handle, "difftest_step"));
    difftest_interrupt = reinterpret_cast<void (*)(uint64_t)>(dlsym(handle, "difftest_interrupt"));

    if (!difftest_init || !difftest_step || !difftest_interrupt) {
        std::cerr << "Error finding symbol: " << dlerror() << std::endl;
        dlclose(handle);
        exit(1);
    }

    difftestCore = difftest_init(binData.data(), binData.size());

    binFile.close();
}

/*#define CHECK_REG(reg)                                                                                                 \*/
/*    {                                                                                                                  \*/
/*        if (difftestCore->reg != simCore.reg) {                                                                        \*/
/*            printf("ERROR: %s mismatch\n", #reg);                                                                      \*/
/*            printf("  difftest_core->%s = 0x%lx\n", #reg, difftestCore->reg);                                          \*/
/*            printf("  cpu->%s = 0x%lx\n", #reg, simCore.reg);                                                          \*/
/*            diff = 1;                                                                                                  \*/
/*        }                                                                                                              \*/
/*    }*/

#define CHECK_REG(reg)                                                                                                 \
    {                                                                                                                  \
        if (difftestCore->reg != simCore.reg) {                                                                        \
            printf("ERROR: %s mismatch\n", #reg);                                                                      \
            printf("  difftest_core->%s = 0x%x\n", #reg, difftestCore->reg);                                          \
            printf("  cpu->%s = 0x%x\n", #reg, simCore.reg);                                                          \
            diff = 1;                                                                                                  \
        }                                                                                                              \
    }

void checkDiff() {
    int diff = 0;

    for (int i = 0; i < 32; i++) {
        CHECK_REG(regs[i]);
    }
    CHECK_REG(pc);
    CHECK_REG(mode);
    CHECK_REG(csrs[MSTATUS]);
    CHECK_REG(csrs[MEDELEG]);
    CHECK_REG(csrs[MIDELEG]);
    CHECK_REG(csrs[MIE]);

    CHECK_REG(csrs[MEPC]);
    CHECK_REG(csrs[MCAUSE]);
    CHECK_REG(csrs[MTVAL]);
    CHECK_REG(csrs[MIP]);

    CHECK_REG(csrs[SEPC]);
    CHECK_REG(csrs[SCAUSE]);
    CHECK_REG(csrs[STVAL]);

    /*if (diff == 1) {*/
    /*    printf("pc : %lx\n", difftestCore->pc);*/
    /*    printf("mode : %x\n", difftestCore->mode);*/
    /*    for (int i = 0; i < 32; i += 2) {*/
    /*        printf("reg %d : %lx reg %d : %lx\n", i, difftestCore->regs[i], i + 1, difftestCore->regs[i + 1]);*/
    /*    }*/
    /**/
    /*    printf("pc : %lx\n", simCore.pc);*/
    /*    printf("mode : %x\n", simCore.mode);*/
    /*    for (int i = 0; i < 32; i += 2) {*/
    /*        printf("reg %d : %lx reg %d : %lx\n", i, simCore.regs[i], i + 1, simCore.regs[i + 1]);*/
    /*    }*/
    /*    exit(1);*/
    /*}*/

    if (diff == 1) {
        printf("pc : %x\n", difftestCore->pc);
        printf("mode : %x\n", difftestCore->mode);
        for (int i = 0; i < 32; i += 2) {
            printf("reg %d : %x reg %d : %x\n", i, difftestCore->regs[i], i + 1, difftestCore->regs[i + 1]);
        }

        printf("pc : %x\n", simCore.pc);
        printf("mode : %x\n", simCore.mode);
        for (int i = 0; i < 32; i += 2) {
            printf("reg %d : %x reg %d : %x\n", i, simCore.regs[i], i + 1, simCore.regs[i + 1]);
        }
        exit(1);
    }
}
// ------

volatile bool ctrlc_pressed = false;
static void   handle_signal(int sig) {
    if (ctrlc_pressed)
        exit(-1);
    ctrlc_pressed = true;
    signal(sig, &handle_signal);
}

const size_t sim_t::INTERLEAVE;

extern device_factory_t *clint_factory;
extern device_factory_t *plic_factory;
extern device_factory_t *ns16550_factory;

sim_t::sim_t(const cfg_t *cfg, bool halted, std::vector<std::pair<reg_t, abstract_mem_t *>> mems,
             const std::vector<device_factory_sargs_t> &plugin_device_factories, const std::vector<std::string> &args,
             const debug_module_config_t &dm_config, const char *log_path, bool dtb_enabled, const char *dtb_file,
             bool  socket_enabled,
             FILE *cmd_file) // needed for command line option --cmd
    : htif_t(args), cfg(cfg), mems(mems), dtb_enabled(dtb_enabled), log_file(log_path), cmd_file(cmd_file),
      sout_(nullptr), current_step(0), current_proc(0), debug(false), histogram_enabled(false), log(false),
      remote_bitbang(NULL), debug_module(this, dm_config) {
    signal(SIGINT, &handle_signal);

    // difftest
    printf("sim_t::sim_t enable difftest\n");
    difftestInit("/home/charain/Project/RISCV-Emulator/libcremu.so", "/home/charain/Project/ysyx-workbench/am-kernels/riscv-tests-am/build/rvc-riscv32-cremu.bin");

    sout_.rdbuf(std::cerr.rdbuf()); // debug output goes to stderr by default

    for (auto &x : mems)
        bus.add_device(x.first, x.second);

    bus.add_device(DEBUG_START, &debug_module);

    socketif = NULL;
#ifdef HAVE_BOOST_ASIO
    if (socket_enabled) {
        socketif = new socketif_t();
    }
#else
    if (socket_enabled) {
        fputs("Socket support requires compilation with boost asio; "
              "please rebuild the riscv-isa-sim project using "
              "\"configure --with-boost-asio\".\n",
              stderr);
        abort();
    }
#endif

#ifndef RISCV_ENABLE_DUAL_ENDIAN
    if (cfg->endianness != endianness_little) {
        fputs("Big-endian support has not been prroperly enabled; "
              "please rebuild the riscv-isa-sim project using "
              "\"configure --enable-dual-endian\".\n",
              stderr);
        abort();
    }
#endif

    debug_mmu = new mmu_t(this, cfg->endianness, NULL);

    // When running without using a dtb, skip the fdt-based configuration steps
    if (!dtb_enabled) {
        for (size_t i = 0; i < cfg->nprocs(); i++) {
            procs.push_back(
                new processor_t(cfg->isa, cfg->priv, cfg, this, cfg->hartids[i], halted, log_file.get(), sout_));
            harts[cfg->hartids[i]] = procs[i];
        }
        return;
    } // otherwise, generate the procs by parsing the DTS

    // Only make a CLINT (Core-Local INTerrupt controller) and PLIC (Platform-
    // Level-Interrupt-Controller) if they are specified in the device tree
    // configuration.
    //
    // This isn't *quite* as general as we could get (because you might have one
    // that's not bus-accessible), but it should handle the normal use cases. In
    // particular, the default device tree configuration that you get without
    // setting the dtb_file argument has one.
    std::vector<device_factory_sargs_t> device_factories = {{clint_factory, {}}, // clint must be element 0
                                                            {plic_factory, {}},  // plic must be element 1
                                                            {ns16550_factory, {}}};
    device_factories.insert(device_factories.end(), plugin_device_factories.begin(), plugin_device_factories.end());

    // Load dtb_file if provided, otherwise self-generate a dts/dtb
    if (dtb_file) {
        std::ifstream fin(dtb_file, std::ios::binary);
        if (!fin.good()) {
            std::cerr << "can't find dtb file: " << dtb_file << std::endl;
            exit(-1);
        }
        std::stringstream strstream;
        strstream << fin.rdbuf();
        dtb = strstream.str();
        dts = dtb_to_dts(dtb);
    } else {
        std::pair<reg_t, reg_t> initrd_bounds = cfg->initrd_bounds;
        std::string             device_nodes;
        for (const device_factory_sargs_t &factory_sargs : device_factories) {
            const device_factory_t         *factory = factory_sargs.first;
            const std::vector<std::string> &sargs   = factory_sargs.second;
            device_nodes.append(factory->generate_dts(this, sargs));
        }
        dts = make_dts(INSNS_PER_RTC_TICK, CPU_HZ, cfg, mems, device_nodes);
        dtb = dts_to_dtb(dts);
    }

    int fdt_code = fdt_check_header(dtb.c_str());
    if (fdt_code) {
        std::cerr << "Failed to read DTB from ";
        if (!dtb_file) {
            std::cerr << "auto-generated DTS string";
        } else {
            std::cerr << "`" << dtb_file << "'";
        }
        std::cerr << ": " << fdt_strerror(fdt_code) << ".\n";
        exit(-1);
    }

    void *fdt = (void *)dtb.c_str();

    // per core attribute
    int    cpu_offset = 0, cpu_map_offset, rc;
    size_t cpu_idx    = 0;
    cpu_offset        = fdt_get_offset(fdt, "/cpus");
    cpu_map_offset    = fdt_get_offset(fdt, "/cpus/cpu-map");
    if (cpu_offset < 0)
        return;

    for (cpu_offset = fdt_get_first_subnode(fdt, cpu_offset); cpu_offset >= 0;
         cpu_offset = fdt_get_next_subnode(fdt, cpu_offset)) {

        if (!(cpu_map_offset < 0) && cpu_offset == cpu_map_offset)
            continue;

        if (cpu_idx != procs.size()) {
            std::cerr << "Spike only supports contiguous CPU IDs in the DTS" << std::endl;
            exit(1);
        }

        // handle isa string
        const char *isa_str;
        rc = fdt_parse_isa(fdt, cpu_offset, &isa_str);
        if (rc != 0) {
            std::cerr << "core (" << cpu_idx << ") has an invalid or missing 'riscv,isa'\n";
            exit(1);
        }

        // handle hartid
        uint32_t hartid;
        rc = fdt_parse_hartid(fdt, cpu_offset, &hartid);
        if (rc != 0) {
            std::cerr << "core (" << cpu_idx << ") has an invalid or missing `reg` (hartid)\n";
            exit(1);
        }

        procs.push_back(new processor_t(isa_str, cfg->priv, cfg, this, hartid, halted, log_file.get(), sout_));
        harts[hartid] = procs[cpu_idx];

        // handle pmp
        reg_t pmp_num, pmp_granularity;
        if (fdt_parse_pmp_num(fdt, cpu_offset, &pmp_num) != 0)
            pmp_num = 0;
        procs[cpu_idx]->set_pmp_num(pmp_num);

        if (fdt_parse_pmp_alignment(fdt, cpu_offset, &pmp_granularity) == 0) {
            procs[cpu_idx]->set_pmp_granularity(pmp_granularity);
        }

        // handle mmu-type
        const char *mmu_type;
        rc = fdt_parse_mmu_type(fdt, cpu_offset, &mmu_type);
        if (rc == 0) {
            procs[cpu_idx]->set_mmu_capability(IMPL_MMU_SBARE);
            if (strncmp(mmu_type, "riscv,sv32", strlen("riscv,sv32")) == 0) {
                procs[cpu_idx]->set_mmu_capability(IMPL_MMU_SV32);
            } else if (strncmp(mmu_type, "riscv,sv39", strlen("riscv,sv39")) == 0) {
                procs[cpu_idx]->set_mmu_capability(IMPL_MMU_SV39);
            } else if (strncmp(mmu_type, "riscv,sv48", strlen("riscv,sv48")) == 0) {
                procs[cpu_idx]->set_mmu_capability(IMPL_MMU_SV48);
            } else if (strncmp(mmu_type, "riscv,sv57", strlen("riscv,sv57")) == 0) {
                procs[cpu_idx]->set_mmu_capability(IMPL_MMU_SV57);
            } else if (strncmp(mmu_type, "riscv,sbare", strlen("riscv,sbare")) == 0) {
                // has been set in the beginning
            } else {
                std::cerr << "core (" << cpu_idx << ") has an invalid 'mmu-type': " << mmu_type << ").\n";
                exit(1);
            }
        } else {
            procs[cpu_idx]->set_mmu_capability(IMPL_MMU_SBARE);
        }

        cpu_idx++;
    }

    // must be located after procs/harts are set (devices might use sim_t get_* member functions)
    for (size_t i = 0; i < device_factories.size(); i++) {
        const device_factory_t         *factory     = device_factories[i].first;
        const std::vector<std::string> &sargs       = device_factories[i].second;
        reg_t                           device_base = 0;
        abstract_device_t              *device      = factory->parse_from_fdt(fdt, this, &device_base, sargs);
        if (device) {
            assert(device_base);
            std::shared_ptr<abstract_device_t> dev_ptr(device);
            add_device(device_base, dev_ptr);

            if (i == 0) // clint_factory
                clint = std::static_pointer_cast<clint_t>(dev_ptr);
            else if (i == 1) // plic_factory
                plic = std::static_pointer_cast<plic_t>(dev_ptr);
        }
    }
}

sim_t::~sim_t() {
    for (size_t i = 0; i < procs.size(); i++)
        delete procs[i];
    delete debug_mmu;
}

int sim_t::run() {
    if (!debug && log)
        set_procs_debug(true);

    htif_t::set_expected_xlen(harts[0]->get_isa().get_max_xlen());

    // htif_t::run() will repeatedly call back into sim_t::idle(), each
    // invocation of which will advance target time
    return htif_t::run();
}

void sim_t::step(size_t n) {
    for (size_t i = 0, steps = 0; i < n; i += steps) {
        steps = 1;
        procs[current_proc]->step(steps);

        current_step += steps;
        if (current_step == INTERLEAVE) {
            current_step = 0;
            //     procs[current_proc]->get_mmu()->yield_load_reservation();
            if (++current_proc == procs.size()) {
                current_proc    = 0;
                reg_t rtc_ticks = INTERLEAVE / INSNS_PER_RTC_TICK;
                for (auto &dev : devices)
                    dev->tick(rtc_ticks);
            }
        }

        // difftest
        auto cpuState = procs[current_proc]->get_state();
        auto mmu      = procs[current_proc]->get_mmu();
        simCore.pc    = cpuState->pc;
        for (int i = 0; i < 32; i++)
            simCore.regs[i] = cpuState->XPR[i];
        simCore.mode          = (enum mode)cpuState->prv;
        simCore.csrs[MSTATUS] = cpuState->mstatus->read();
        simCore.csrs[MEDELEG] = cpuState->medeleg->read();
        simCore.csrs[MIDELEG] = cpuState->mideleg->read();
        simCore.csrs[MIE]     = cpuState->mie->read();

        simCore.csrs[MEPC]   = cpuState->mepc->read();
        simCore.csrs[MCAUSE] = cpuState->mcause->read();
        simCore.csrs[MTVAL]  = cpuState->mtval->read();

        simCore.csrs[SEPC]   = cpuState->sepc->read();
        simCore.csrs[SCAUSE] = cpuState->scause->read();
        simCore.csrs[STVAL]  = cpuState->stval->read();

        // 同步 csr
        difftestCore->csrs[0x7a0] = cpuState->tselect->read();
        difftestCore->csrs[0x7a4] = 0x100807c;

        difftest_step();

        simCore.csrs[MIP]       = cpuState->mip->read();
        difftest_interrupt(cpuState->mip->read());

        cpuState->satp->write(difftestCore->csrs[SATP]);

        checkDiff();
    }
}

void sim_t::add_device(reg_t addr, std::shared_ptr<abstract_device_t> dev) {
    bus.add_device(addr, dev.get());
    devices.push_back(dev);
}

void sim_t::set_debug(bool value) { debug = value; }

void sim_t::set_histogram(bool value) {
    histogram_enabled = value;
    for (size_t i = 0; i < procs.size(); i++) {
        procs[i]->set_histogram(histogram_enabled);
    }
}

void sim_t::configure_log(bool enable_log, bool enable_commitlog) {
    log = enable_log;

    if (!enable_commitlog)
        return;

    for (processor_t *proc : procs) {
        proc->enable_log_commits();
    }
}

void sim_t::set_procs_debug(bool value) {
    for (size_t i = 0; i < procs.size(); i++)
        procs[i]->set_debug(value);
}

static bool paddr_ok(reg_t addr) {
    static_assert(MAX_PADDR_BITS == 8 * sizeof(addr));
    return true;
}

bool sim_t::mmio_load(reg_t paddr, size_t len, uint8_t *bytes) {
    if (paddr + len < paddr || !paddr_ok(paddr + len - 1))
        return false;
    return bus.load(paddr, len, bytes);
}

bool sim_t::mmio_store(reg_t paddr, size_t len, const uint8_t *bytes) {
    if (paddr + len < paddr || !paddr_ok(paddr + len - 1))
        return false;
    return bus.store(paddr, len, bytes);
}

void sim_t::set_rom() {
    const int reset_vec_size = 8;

    reg_t start_pc = cfg->start_pc.value_or(get_entry_point());

    uint32_t reset_vec[reset_vec_size] = {0x297,                                        // auipc  t0,0x0
                                          0x28593 + (reset_vec_size * 4 << 20),         // addi   a1, t0, &dtb
                                          0xf1402573,                                   // csrr   a0, mhartid
                                          get_core(0)->get_xlen() == 32 ? 0x0182a283u : // lw     t0,24(t0)
                                              0x0182b283u,                              // ld     t0,24(t0)
                                          0x28067,                                      // jr     t0
                                          0,
                                          (uint32_t)(start_pc & 0xffffffff),
                                          (uint32_t)(start_pc >> 32)};
    if (get_target_endianness() == endianness_big) {
        int i;
        // Instuctions are little endian
        for (i = 0; reset_vec[i] != 0; i++)
            reset_vec[i] = to_le(reset_vec[i]);
        // Data is big endian
        for (; i < reset_vec_size; i++)
            reset_vec[i] = to_be(reset_vec[i]);

        // Correct the high/low order of 64-bit start PC
        if (get_core(0)->get_xlen() != 32)
            std::swap(reset_vec[reset_vec_size - 2], reset_vec[reset_vec_size - 1]);
    } else {
        for (int i = 0; i < reset_vec_size; i++)
            reset_vec[i] = to_le(reset_vec[i]);
    }

    std::vector<char> rom((char *)reset_vec, (char *)reset_vec + sizeof(reset_vec));

    rom.insert(rom.end(), dtb.begin(), dtb.end());
    const int align = 0x1000;
    rom.resize((rom.size() + align - 1) / align * align);

    std::shared_ptr<rom_device_t> boot_rom(new rom_device_t(rom));
    add_device(DEFAULT_RSTVEC, boot_rom);
}

char *sim_t::addr_to_mem(reg_t paddr) {
    if (!paddr_ok(paddr))
        return NULL;
    auto desc = bus.find_device(paddr);
    if (auto mem = dynamic_cast<abstract_mem_t *>(desc.second))
        if (paddr - desc.first < mem->size())
            return mem->contents(paddr - desc.first);
    return NULL;
}

const char *sim_t::get_symbol(uint64_t paddr) { return htif_t::get_symbol(paddr); }

// htif

void sim_t::reset() {
    if (dtb_enabled)
        set_rom();
}

void sim_t::idle() {
    if (done())
        return;

    if (debug || ctrlc_pressed)
        interactive();
    else
        step(INTERLEAVE);

    if (remote_bitbang)
        remote_bitbang->tick();
}

void sim_t::read_chunk(addr_t taddr, size_t len, void *dst) {
    assert(len == 8);
    auto data = debug_mmu->to_target(debug_mmu->load<uint64_t>(taddr));
    memcpy(dst, &data, sizeof data);
}

void sim_t::write_chunk(addr_t taddr, size_t len, const void *src) {
    assert(len == 8);
    target_endian<uint64_t> data;
    memcpy(&data, src, sizeof data);
    debug_mmu->store<uint64_t>(taddr, debug_mmu->from_target(data));
}

endianness_t sim_t::get_target_endianness() const {
    return debug_mmu->is_target_big_endian() ? endianness_big : endianness_little;
}

void sim_t::proc_reset(unsigned id) { debug_module.proc_reset(id); }
