use crate::{clint, hart_id, qemu_hsm::QemuHsm, FixedRustSBI, Supervisor};
use core::arch::asm;
use core::mem::size_of;
use memoffset::offset_of;
use riscv::register::*;
use rustsbi::spec::binary::SbiRet;

#[repr(usize)]
pub(crate) enum Operation {
    Stop = 0,
    SystemReset = 1,
}

pub(crate) fn execute_supervisor(
    sbi: FixedRustSBI,
    hsm: &QemuHsm,
    supervisor: Supervisor,
) -> Operation {
    clint::msip::clear();
    clint::mtimecmp::clear();
    unsafe {
        asm!("csrw mideleg, {}", in(reg) !0);
        asm!("csrw medeleg, {}", in(reg) !0);
        asm!("csrw mcounteren, {}", in(reg) !0);
        medeleg::clear_supervisor_env_call();
        medeleg::clear_machine_env_call();

        mtvec::write(trap_vec as _, mtvec::TrapMode::Vectored);
        mie::set_mext();
        mie::set_msoft();
        mie::set_mtimer();
    }
    let ctx = SupervisorContext::new(supervisor);

    let mut env = Environment { ctx, sbi };
    mscratch::write(&mut env.ctx as *mut _ as _);

    hsm.record_current_start_finished();
    loop {
        unsafe { m_to_s() };

        use mcause::{Exception, Trap};
        match mcause::read().cause() {
            Trap::Exception(Exception::SupervisorEnvCall) => {
                if let Some(op) = env.handle_ecall() {
                    break op;
                }
            }
            t => env.trap_stop(t),
        }
    }
}

struct Environment<'a> {
    ctx: SupervisorContext,
    sbi: FixedRustSBI<'a>,
}

impl<'a> Environment<'a> {
    fn handle_ecall(&mut self) -> Option<Operation> {
        use rustsbi::spec::{binary::*, hsm::*, srst::*};
        if self.ctx.sbi_extension() == sbi_spec::legacy::LEGACY_CONSOLE_PUTCHAR {
            let ch = self.ctx.a0;
            print!("{:}", ch as u8 as char);
            self.ctx.mepc = self.ctx.mepc.wrapping_add(4);
            return None;
        } else if self.ctx.sbi_extension() == sbi_spec::legacy::LEGACY_CONSOLE_GETCHAR {
            self.ctx.a0 = unsafe { crate::UART.lock().assume_init_mut().receive() } as usize;
            self.ctx.mepc = self.ctx.mepc.wrapping_add(4);
            return None;
        }
        let ans = self.sbi.handle_ecall(
            self.ctx.sbi_extension(),
            self.ctx.sbi_function(),
            self.ctx.sbi_param(),
        );
        // 判断导致退出执行流程的调用
        if ans.error == RET_SUCCESS {
            match (self.ctx.sbi_extension(), self.ctx.sbi_function()) {
                // 核状态
                (EID_HSM, HART_STOP) => return Some(Operation::Stop),
                (EID_HSM, HART_SUSPEND)
                    if matches!(
                        u32::try_from(self.ctx.a0),
                        Ok(HART_SUSPEND_TYPE_NON_RETENTIVE)
                    ) =>
                {
                    return Some(Operation::Stop)
                }
                // 系统重置
                (EID_SRST, SYSTEM_RESET)
                    if matches!(
                        u32::try_from(self.ctx.a0),
                        Ok(RESET_TYPE_COLD_REBOOT) | Ok(RESET_TYPE_WARM_REBOOT)
                    ) =>
                {
                    return Some(Operation::SystemReset)
                }
                _ => {}
            }
        }
        self.ctx.fill_in(ans);
        self.ctx.mepc = self.ctx.mepc.wrapping_add(4);
        None
    }

    fn trap_stop(&self, trap: mcause::Trap) -> ! {
        println!(
            "
-----------------------------
> trap:    {trap:?}
> mstatus: {:#018x}
> mepc:    {:#018x}
> mtval:   {:#018x}
-----------------------------
",
            self.ctx.mstatus,
            self.ctx.mepc,
            mtval::read()
        );
        panic!("stopped with unsupported trap")
    }
}

#[derive(Debug, Default)]
struct SupervisorContext {
    msp: usize,
    ra: usize,
    sp: usize,
    gp: usize,
    tp: usize,
    t0: usize,
    t1: usize,
    t2: usize,
    s0: usize,
    s1: usize,
    a0: usize,
    a1: usize,
    a2: usize,
    a3: usize,
    a4: usize,
    a5: usize,
    a6: usize,
    a7: usize,
    s2: usize,
    s3: usize,
    s4: usize,
    s5: usize,
    s6: usize,
    s7: usize,
    s8: usize,
    s9: usize,
    s10: usize,
    s11: usize,
    t3: usize,
    t4: usize,
    t5: usize,
    t6: usize,
    mstatus: usize,
    mepc: usize,
}

impl SupervisorContext {
    fn new(supervisor: Supervisor) -> Self {
        let mut ctx = Self {
            mepc: supervisor.start_addr,
            ..Default::default()
        };

        unsafe {
            mstatus::set_mpp(mstatus::MPP::Supervisor);
            mstatus::set_mpie();
            asm!("csrr {}, mstatus", out(reg) ctx.mstatus)
        };
        *ctx.hart_id_mut() = hart_id();
        *ctx.opaque_mut() = supervisor.opaque;

        ctx
    }

    #[inline]
    fn sbi_extension(&self) -> usize {
        self.a7
    }

    #[inline]
    fn sbi_function(&self) -> usize {
        self.a6
    }

    #[inline]
    fn sbi_param(&self) -> [usize; 6] {
        [self.a0, self.a1, self.a2, self.a3, self.a4, self.a5]
    }

    #[inline]
    fn fill_in(&mut self, ans: SbiRet) {
        self.a0 = ans.error;
        self.a1 = ans.value;
    }

    #[inline]
    fn hart_id_mut(&mut self) -> &mut usize {
        &mut self.a0
    }

    #[inline]
    fn opaque_mut(&mut self) -> &mut usize {
        &mut self.a1
    }

    #[allow(unused)]
    fn do_transfer_trap(&mut self, cause: scause::Trap) {
        unsafe {
            // 向 S 转发陷入
            mstatus::set_mpp(mstatus::MPP::Supervisor);
            // 转发陷入源状态
            let spp = match (self.mstatus >> 11) & 0b11 {
                // U
                0b00 => mstatus::SPP::User,
                // S
                0b01 => mstatus::SPP::Supervisor,
                // H/M
                mpp => unreachable!("invalid mpp: {mpp:#x} to delegate"),
            };
            mstatus::set_spp(spp);
            // 转发陷入原因
            scause::set(cause);
            // 转发陷入附加信息
            stval::write(mtval::read());
            // 转发陷入地址
            sepc::write(self.mepc);
            // 设置 S 中断状态
            if mstatus::read().sie() {
                mstatus::set_spie();
                mstatus::clear_sie();
            }
            asm!("csrr {}, mstatus", out(reg) self.mstatus);
            // 设置返回地址，返回到 S
            // TODO Vectored stvec?
            self.mepc = stvec::read().address();
        }
    }
}

struct MachineContext {
    supervisor_context: usize,
    ra: usize,
    gp: usize,
    tp: usize,
    s0: usize,
    s1: usize,
    s2: usize,
    s3: usize,
    s4: usize,
    s5: usize,
    s6: usize,
    s7: usize,
    s8: usize,
    s9: usize,
    s10: usize,
    s11: usize,
}

/// M 态转到 S 态。
///
/// # Safety
///
/// 裸函数，手动保存所有上下文环境。
/// 为了写起来简单，占 32 * usize 空间，循环 31 次保存 31 个通用寄存器。
/// 实际 x0(zero) 和 x2(sp) 不需要保存在这里。
#[naked]
unsafe extern "C" fn m_to_s() {
    asm!(
        // 初始化栈帧：sp = Mctx
        "addi  sp, sp, -{machine_context}",
        // 特权上下文地址保存到机器上下文
        "csrr  t0, mscratch",
        "sd  t0, {supervisor_context}(sp)",
        // 保存机器上下文
        "sd  ra, {machine_ra}(sp)",
        "sd  gp, {machine_gp}(sp)",
        "sd  tp, {machine_tp}(sp)",
        "sd  s0, {machine_s0}(sp)",
        "sd  s1, {machine_s1}(sp)",
        "sd  s2, {machine_s2}(sp)",
        "sd  s3, {machine_s3}(sp)",
        "sd  s4, {machine_s4}(sp)",
        "sd  s5, {machine_s5}(sp)",
        "sd  s6, {machine_s6}(sp)",
        "sd  s7, {machine_s7}(sp)",
        "sd  s8, {machine_s8}(sp)",
        "sd  s9, {machine_s9}(sp)",
        "sd  s10, {machine_s10}(sp)",
        "sd  s11, {machine_s11}(sp)",
        // 切换上下文：sp = Sctx
        "csrrw sp, mscratch, sp",
        // 机器上下文地址保存到特权上下文
        "csrr  t0, mscratch",
        "sd  t0, {machine_sp}(sp)",
        // 恢复 csr
        "ld  t0, {supervisor_mstatus}(sp)",
        "ld  t1, {supervisor_mepc}(sp)",
        "csrw  mstatus, t0",
        "csrw  mepc, t1",
        // 恢复特权上下文
        "ld  ra, {supervisor_ra}(sp)",
        "ld  gp, {supervisor_gp}(sp)",
        "ld  tp, {supervisor_tp}(sp)",
        "ld  t0, {supervisor_t0}(sp)",
        "ld  t1, {supervisor_t1}(sp)",
        "ld  t2, {supervisor_t2}(sp)",
        "ld  s0, {supervisor_s0}(sp)",
        "ld  s1, {supervisor_s1}(sp)",
        "ld  a0, {supervisor_a0}(sp)",
        "ld  a1, {supervisor_a1}(sp)",
        "ld  a2, {supervisor_a2}(sp)",
        "ld  a3, {supervisor_a3}(sp)",
        "ld  a4, {supervisor_a4}(sp)",
        "ld  a5, {supervisor_a5}(sp)",
        "ld  a6, {supervisor_a6}(sp)",
        "ld  a7, {supervisor_a7}(sp)",
        "ld  s2, {supervisor_s2}(sp)",
        "ld  s3, {supervisor_s3}(sp)",
        "ld  s4, {supervisor_s4}(sp)",
        "ld  s5, {supervisor_s5}(sp)",
        "ld  s6, {supervisor_s6}(sp)",
        "ld  s7, {supervisor_s7}(sp)",
        "ld  s8, {supervisor_s8}(sp)",
        "ld  s9, {supervisor_s9}(sp)",
        "ld  s10, {supervisor_s10}(sp)",
        "ld  s11, {supervisor_s11}(sp)",
        "ld  t3, {supervisor_t3}(sp)",
        "ld  t4, {supervisor_t4}(sp)",
        "ld  t5, {supervisor_t5}(sp)",
        "ld  t6, {supervisor_t6}(sp)",
        "ld  sp, {supervisor_sp}(sp)",
        // 执行特权程序
        "mret",
        machine_context = const size_of::<MachineContext>(),
        machine_ra = const offset_of!(MachineContext, ra),
        machine_gp = const offset_of!(MachineContext, gp),
        machine_tp = const offset_of!(MachineContext, tp),
        machine_s0 = const offset_of!(MachineContext, s0),
        machine_s1 = const offset_of!(MachineContext, s1),
        machine_s2 = const offset_of!(MachineContext, s2),
        machine_s3 = const offset_of!(MachineContext, s3),
        machine_s4 = const offset_of!(MachineContext, s4),
        machine_s5 = const offset_of!(MachineContext, s5),
        machine_s6 = const offset_of!(MachineContext, s6),
        machine_s7 = const offset_of!(MachineContext, s7),
        machine_s8 = const offset_of!(MachineContext, s8),
        machine_s9 = const offset_of!(MachineContext, s9),
        machine_s10 = const offset_of!(MachineContext, s10),
        machine_s11 = const offset_of!(MachineContext, s11),
        machine_sp = const offset_of!(SupervisorContext, msp),
        supervisor_context = const offset_of!(MachineContext, supervisor_context),
        supervisor_mstatus = const offset_of!(SupervisorContext, mstatus),
        supervisor_mepc = const offset_of!(SupervisorContext, mepc),
        supervisor_ra = const offset_of!(SupervisorContext, ra),
        supervisor_gp = const offset_of!(SupervisorContext, gp),
        supervisor_tp = const offset_of!(SupervisorContext, tp),
        supervisor_t0 = const offset_of!(SupervisorContext, t0),
        supervisor_t1 = const offset_of!(SupervisorContext, t1),
        supervisor_t2 = const offset_of!(SupervisorContext, t2),
        supervisor_s0 = const offset_of!(SupervisorContext, s0),
        supervisor_s1 = const offset_of!(SupervisorContext, s1),
        supervisor_a0 = const offset_of!(SupervisorContext, a0),
        supervisor_a1 = const offset_of!(SupervisorContext, a1),
        supervisor_a2 = const offset_of!(SupervisorContext, a2),
        supervisor_a3 = const offset_of!(SupervisorContext, a3),
        supervisor_a4 = const offset_of!(SupervisorContext, a4),
        supervisor_a5 = const offset_of!(SupervisorContext, a5),
        supervisor_a6 = const offset_of!(SupervisorContext, a6),
        supervisor_a7 = const offset_of!(SupervisorContext, a7),
        supervisor_s2 = const offset_of!(SupervisorContext, s2),
        supervisor_s3 = const offset_of!(SupervisorContext, s3),
        supervisor_s4 = const offset_of!(SupervisorContext, s4),
        supervisor_s5 = const offset_of!(SupervisorContext, s5),
        supervisor_s6 = const offset_of!(SupervisorContext, s6),
        supervisor_s7 = const offset_of!(SupervisorContext, s7),
        supervisor_s8 = const offset_of!(SupervisorContext, s8),
        supervisor_s9 = const offset_of!(SupervisorContext, s9),
        supervisor_s10 = const offset_of!(SupervisorContext, s10),
        supervisor_s11 = const offset_of!(SupervisorContext, s11),
        supervisor_t3 = const offset_of!(SupervisorContext, t3),
        supervisor_t4 = const offset_of!(SupervisorContext, t4),
        supervisor_t5 = const offset_of!(SupervisorContext, t5),
        supervisor_t6 = const offset_of!(SupervisorContext, t6),
        supervisor_sp = const offset_of!(SupervisorContext, sp),
        options(noreturn)
    )
}

/// S 态陷入 M 态。
///
/// # Safety
///
/// 裸函数。
/// 利用恢复的 ra 回到 [`m_to_s`] 的返回地址。
#[naked]
unsafe extern "C" fn s_to_m() -> ! {
    asm!(
        // 切换上下文：sp = Sctx
        "csrrw sp, mscratch, sp",
        "ld  sp, {supervisor_context}(sp)",
        // 保存特权上下文
        "sd  ra, {supervisor_ra}(sp)",
        "sd  gp, {supervisor_gp}(sp)",
        "sd  tp, {supervisor_tp}(sp)",
        "sd  t0, {supervisor_t0}(sp)",
        "sd  t1, {supervisor_t1}(sp)",
        "sd  t2, {supervisor_t2}(sp)",
        "sd  s0, {supervisor_s0}(sp)",
        "sd  s1, {supervisor_s1}(sp)",
        "sd  a0, {supervisor_a0}(sp)",
        "sd  a1, {supervisor_a1}(sp)",
        "sd  a2, {supervisor_a2}(sp)",
        "sd  a3, {supervisor_a3}(sp)",
        "sd  a4, {supervisor_a4}(sp)",
        "sd  a5, {supervisor_a5}(sp)",
        "sd  a6, {supervisor_a6}(sp)",
        "sd  a7, {supervisor_a7}(sp)",
        "sd  s2, {supervisor_s2}(sp)",
        "sd  s3, {supervisor_s3}(sp)",
        "sd  s4, {supervisor_s4}(sp)",
        "sd  s5, {supervisor_s5}(sp)",
        "sd  s6, {supervisor_s6}(sp)",
        "sd  s7, {supervisor_s7}(sp)",
        "sd  s8, {supervisor_s8}(sp)",
        "sd  s9, {supervisor_s9}(sp)",
        "sd  s10, {supervisor_s10}(sp)",
        "sd  s11, {supervisor_s11}(sp)",
        "sd  t3, {supervisor_t3}(sp)",
        "sd  t4, {supervisor_t4}(sp)",
        "sd  t5, {supervisor_t5}(sp)",
        "sd  t6, {supervisor_t6}(sp)",
        "csrrw  t0, mscratch, sp",
        "sd  t0, {supervisor_sp}(sp)",
        // 保存 csr
        "csrr  t0, mstatus",
        "csrr  t1, mepc",
        "sd  t0, {supervisor_mstatus}(sp)",
        "sd  t1, {supervisor_mepc}(sp)",
        // 切换上下文：sp = Mctx
        "ld  sp, {machine_sp}(sp)",
        // 恢复机器上下文
        "ld  ra, {machine_ra}(sp)",
        "ld  gp, {machine_gp}(sp)",
        "ld  tp, {machine_tp}(sp)",
        "ld  s0, {machine_s0}(sp)",
        "ld  s1, {machine_s1}(sp)",
        "ld  s2, {machine_s2}(sp)",
        "ld  s3, {machine_s3}(sp)",
        "ld  s4, {machine_s4}(sp)",
        "ld  s5, {machine_s5}(sp)",
        "ld  s6, {machine_s6}(sp)",
        "ld  s7, {machine_s7}(sp)",
        "ld  s8, {machine_s8}(sp)",
        "ld  s9, {machine_s9}(sp)",
        "ld  s10, {machine_s10}(sp)",
        "ld  s11, {machine_s11}(sp)",
        // 栈帧释放，返回
        "addi  sp, sp, {machine_context}",
        "ret",
        supervisor_context = const offset_of!(MachineContext, supervisor_context),
        supervisor_ra = const offset_of!(SupervisorContext, ra),
        supervisor_gp = const offset_of!(SupervisorContext, gp),
        supervisor_tp = const offset_of!(SupervisorContext, tp),
        supervisor_t0 = const offset_of!(SupervisorContext, t0),
        supervisor_t1 = const offset_of!(SupervisorContext, t1),
        supervisor_t2 = const offset_of!(SupervisorContext, t2),
        supervisor_s0 = const offset_of!(SupervisorContext, s0),
        supervisor_s1 = const offset_of!(SupervisorContext, s1),
        supervisor_a0 = const offset_of!(SupervisorContext, a0),
        supervisor_a1 = const offset_of!(SupervisorContext, a1),
        supervisor_a2 = const offset_of!(SupervisorContext, a2),
        supervisor_a3 = const offset_of!(SupervisorContext, a3),
        supervisor_a4 = const offset_of!(SupervisorContext, a4),
        supervisor_a5 = const offset_of!(SupervisorContext, a5),
        supervisor_a6 = const offset_of!(SupervisorContext, a6),
        supervisor_a7 = const offset_of!(SupervisorContext, a7),
        supervisor_s2 = const offset_of!(SupervisorContext, s2),
        supervisor_s3 = const offset_of!(SupervisorContext, s3),
        supervisor_s4 = const offset_of!(SupervisorContext, s4),
        supervisor_s5 = const offset_of!(SupervisorContext, s5),
        supervisor_s6 = const offset_of!(SupervisorContext, s6),
        supervisor_s7 = const offset_of!(SupervisorContext, s7),
        supervisor_s8 = const offset_of!(SupervisorContext, s8),
        supervisor_s9 = const offset_of!(SupervisorContext, s9),
        supervisor_s10 = const offset_of!(SupervisorContext, s10),
        supervisor_s11 = const offset_of!(SupervisorContext, s11),
        supervisor_t3 = const offset_of!(SupervisorContext, t3),
        supervisor_t4 = const offset_of!(SupervisorContext, t4),
        supervisor_t5 = const offset_of!(SupervisorContext, t5),
        supervisor_t6 = const offset_of!(SupervisorContext, t6),
        supervisor_mstatus = const offset_of!(SupervisorContext, mstatus),
        supervisor_mepc = const offset_of!(SupervisorContext, mepc),
        supervisor_sp = const offset_of!(SupervisorContext, sp),
        machine_sp = const offset_of!(SupervisorContext, msp),
        machine_ra = const offset_of!(MachineContext, ra),
        machine_gp = const offset_of!(MachineContext, gp),
        machine_tp = const offset_of!(MachineContext, tp),
        machine_s0 = const offset_of!(MachineContext, s0),
        machine_s1 = const offset_of!(MachineContext, s1),
        machine_s2 = const offset_of!(MachineContext, s2),
        machine_s3 = const offset_of!(MachineContext, s3),
        machine_s4 = const offset_of!(MachineContext, s4),
        machine_s5 = const offset_of!(MachineContext, s5),
        machine_s6 = const offset_of!(MachineContext, s6),
        machine_s7 = const offset_of!(MachineContext, s7),
        machine_s8 = const offset_of!(MachineContext, s8),
        machine_s9 = const offset_of!(MachineContext, s9),
        machine_s10 = const offset_of!(MachineContext, s10),
        machine_s11 = const offset_of!(MachineContext, s11),
        machine_context = const size_of::<MachineContext>(),
        options(noreturn)
    )
}

/// 中断向量表
///
/// # Safety
///
/// 裸函数。
#[naked]
unsafe extern "C" fn trap_vec() {
    asm!(
        ".align 2",
        ".option push",
        ".option norvc",
        "j {s_to_m}", // exception
        "j {s_to_m}", // supervisor software
        "j {s_to_m}", // reserved
        "j {msoft} ", // machine    software
        "j {s_to_m}", // reserved
        "j {s_to_m}", // supervisor timer
        "j {s_to_m}", // reserved
        "j {mtimer}", // machine    timer
        "j {s_to_m}", // reserved
        "j {s_to_m}", // supervisor external
        "j {s_to_m}", // reserved
        "j {s_to_m}", // machine    external
        ".option pop",
        s_to_m = sym s_to_m,
        mtimer = sym mtimer,
        msoft  = sym msoft,
        options(noreturn)
    )
}

/// machine timer 中断代理
///
/// # Safety
///
/// 裸函数。
#[naked]
unsafe extern "C" fn mtimer() {
    asm!(
        // 换栈：
        // sp      : M sp
        // mscratch: S sp
        "   csrrw sp, mscratch, sp",
        // 需要 a0 传参，保护
        "   addi sp, sp, -16
            sd   ra, 0(sp)
            sd   a0, 8(sp)
        ",
        // clint::mtimecmp::clear();
        "   li   a0, {u64_max}
            call {set_mtimecmp}
        ",
        // mip::set_stimer();
        "   li   a0, {mip_stip}
           csrrs zero, mip, a0
        ",
        // 恢复 a0
        "   ld   a0, 8(sp)
            ld   ra, 0(sp)
            addi sp, sp,  16
        ",
        // 换栈：
        // sp      : S sp
        // mscratch: M sp
        "   csrrw sp, mscratch, sp",
        // 返回
        "   mret",
        u64_max      = const u64::MAX,
        mip_stip     = const 1 << 5,
        set_mtimecmp =   sym clint::mtimecmp::set_naked,
        options(noreturn)
    )
}

/// machine soft 中断代理
///
/// # Safety
///
/// 裸函数。
#[naked]
unsafe extern "C" fn msoft() {
    asm!(
        // 换栈：
        // sp      : M sp
        // mscratch: S sp
        "   csrrw sp, mscratch, sp",
        // 保护 ra
        "   addi sp, sp, -8
            sd   ra, 0(sp)
        ",
        // clint::msip::clear();
        // mip::set_ssoft();
        "   call   {clear_msip}
            csrrsi zero, mip, 1 << 1
        ",
        // 恢复 ra
        "   ld   ra, 0(sp)
            addi sp, sp,  8
        ",
        // 换栈：
        // sp      : S sp
        // mscratch: M sp
        "   csrrw sp, mscratch, sp",
        // 返回
        "   mret",
        clear_msip = sym clint::msip::clear_naked,
        options(noreturn)
    )
}
