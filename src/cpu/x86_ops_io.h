/* A port access may run the timers (a PIT read brings the TSC up to date), and
   one of them may reset the CPU (the keyboard controller's reset pulse): the
   instruction then ends the recompiled block, which would otherwise go on and
   overwrite the reset state. A trapped I/O cycle asserts SMI# during the
   instruction, also taken at its boundary. */
static __inline int
io_op_end(void)
{
    return x86_was_reset || smi_line || (nmi && nmi_enable && nmi_mask);
}

static int
opIN_AL_imm(uint32_t fetchdat)
{
    uint16_t port = (uint16_t) getbytef();
    check_io_perm(port, 1);
    uint8_t val = inb(port);
    if (x86_was_reset)
        return 1;
    AL = val;
    CLOCK_CYCLES(12 + io_waitstates);
    PREFETCH_RUN(12, 2, -1, 1, 0, 0, 0, 0);
    return io_op_end();
}
static int
opIN_AX_imm(uint32_t fetchdat)
{
    uint16_t port = (uint16_t) getbytef();
    check_io_perm(port, 2);
    uint16_t val = inw(port);
    if (x86_was_reset)
        return 1;
    AX = val;
    CLOCK_CYCLES(12 + io_waitstates);
    PREFETCH_RUN(12, 2, -1, 1, 0, 0, 0, 0);
    return io_op_end();
}
static int
opIN_EAX_imm(uint32_t fetchdat)
{
    uint16_t port = (uint16_t) getbytef();
    check_io_perm(port, 4);
    uint32_t val = inl(port);
    if (x86_was_reset)
        return 1;
    EAX = val;
    CLOCK_CYCLES(12 + io_waitstates);
    PREFETCH_RUN(12, 2, -1, 0, 1, 0, 0, 0);
    return io_op_end();
}

static int
opOUT_AL_imm(uint32_t fetchdat)
{
    uint16_t port = (uint16_t) getbytef();
    check_io_perm(port, 1);
    outb(port, AL);
    CLOCK_CYCLES(10 + io_waitstates);
    PREFETCH_RUN(10, 2, -1, 0, 0, 1, 0, 0);
    return io_op_end();
}
static int
opOUT_AX_imm(uint32_t fetchdat)
{
    uint16_t port = (uint16_t) getbytef();
    check_io_perm(port, 2);
    outw(port, AX);
    CLOCK_CYCLES(10 + io_waitstates);
    PREFETCH_RUN(10, 2, -1, 0, 0, 1, 0, 0);
    return io_op_end();
}
static int
opOUT_EAX_imm(uint32_t fetchdat)
{
    uint16_t port = (uint16_t) getbytef();
    check_io_perm(port, 4);
    outl(port, EAX);
    CLOCK_CYCLES(10 + io_waitstates);
    PREFETCH_RUN(10, 2, -1, 0, 0, 0, 1, 0);
    return io_op_end();
}

static int
opIN_AL_DX(UNUSED(uint32_t fetchdat))
{
    check_io_perm(DX, 1);
    uint8_t val = inb(DX);
    if (x86_was_reset)
        return 1;
    AL = val;
    CLOCK_CYCLES(12 + io_waitstates);
    PREFETCH_RUN(12, 1, -1, 1, 0, 0, 0, 0);
    return io_op_end();
}
static int
opIN_AX_DX(UNUSED(uint32_t fetchdat))
{
    check_io_perm(DX, 2);
    uint16_t val = inw(DX);
    if (x86_was_reset)
        return 1;
    AX = val;
    CLOCK_CYCLES(12 + io_waitstates);
    PREFETCH_RUN(12, 1, -1, 1, 0, 0, 0, 0);
    return io_op_end();
}
static int
opIN_EAX_DX(UNUSED(uint32_t fetchdat))
{
    check_io_perm(DX, 4);
    uint32_t val = inl(DX);
    if (x86_was_reset)
        return 1;
    EAX = val;
    CLOCK_CYCLES(12 + io_waitstates);
    PREFETCH_RUN(12, 1, -1, 0, 1, 0, 0, 0);
    return io_op_end();
}

static int
opOUT_AL_DX(UNUSED(uint32_t fetchdat))
{
    check_io_perm(DX, 1);
    outb(DX, AL);
    CLOCK_CYCLES(11 + io_waitstates);
    PREFETCH_RUN(11, 1, -1, 0, 0, 1, 0, 0);
    return io_op_end();
}
static int
opOUT_AX_DX(UNUSED(uint32_t fetchdat))
{
    check_io_perm(DX, 2);
    outw(DX, AX);
    CLOCK_CYCLES(11 + io_waitstates);
    PREFETCH_RUN(11, 1, -1, 0, 0, 1, 0, 0);
    return io_op_end();
}
static int
opOUT_EAX_DX(UNUSED(uint32_t fetchdat))
{
    check_io_perm(DX, 4);
    outl(DX, EAX);
    CLOCK_CYCLES(11 + io_waitstates);
    PREFETCH_RUN(11, 1, -1, 0, 0, 0, 1, 0);
    return io_op_end();
}
