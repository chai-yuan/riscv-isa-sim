if (!STATE.debug_mode && (
        (!STATE.v && STATE.prv == PRV_M && STATE.dcsr->ebreakm) ||
        (!STATE.v && STATE.prv == PRV_S && STATE.dcsr->ebreaks) ||
        (!STATE.v && STATE.prv == PRV_U && STATE.dcsr->ebreaku) ||
        (STATE.v && STATE.prv == PRV_S && STATE.dcsr->ebreakvs) ||
        (STATE.v && STATE.prv == PRV_U && STATE.dcsr->ebreakvu))) {
      printf("Spike exited normally due to ebreak instruct\n");
      exit(0);
	throw trap_debug_mode();
} else {
      printf("Spike exited normally due to ebreak instruct\n");
      exit(0);
	throw trap_breakpoint(STATE.v, pc);
}
