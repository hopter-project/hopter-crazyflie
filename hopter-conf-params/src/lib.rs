//! This crate defines system configuration parameters for Hopter embedded
//! operating system. Clients of Hopter can change them as needed.
//!
//! For the correctness of system functioning, make sure the following
//! parameter is correctly set.
//! - [`SYSTICK_FREQUENCY_HZ`] : Hopter depends on it to generate 1 millisecond
//!   interval ticks.
//!
//! Names that are prefixed with single underscore are considered semi-private.
//! One should not change it unless being familiar with Hopter kernel's source
//! code. Names that are prefixed with double underscores are considered
//! private. One should not change it unless the corresponding kernel code and
//! the compiler's source code are also changed accordingly.

#![no_std]

/* ############################ */
/* ### Clock Configurations ### */
/* ############################ */

/// The frequency of the SysTick timer clock. Must be set correctly because
/// Hopter relies on it to configure the SysTick counter to trigger the
/// interrupt at 1 millisecond interval.
pub const SYSTICK_FREQUENCY_HZ: u32 = 168_000_000;

/* ############################ */
/* ### Stack Configurations ### */
/* ############################ */

/// Whether dynamic extension of the stack is allowed.
pub const ALLOW_DYNAMIC_STACK: bool = true;

/// The extra size added to a stacklet allocation request in addition to the
/// allocation size requested by the function prologue.
pub const STACKLET_ADDITION_ALLOC_SIZE: usize = 1536;

/// The number of hot-split site that a task can address. The larger the number
/// is, the unlikely that a task will suffer from hot-split, but the task
/// struct also becomes larger.
pub const HOT_SPLIT_PREVENTION_CACHE_SIZE: usize = 6;

/// During the existance of a stacklet, if [`HOT_SPLIT_DETECTION_THRESHOLD`] or
/// more new stacklet allocation is requested while the task is running with the
/// stacklet, it will be considered as a hot-split site.
pub const HOT_SPLIT_DETECTION_THRESHOLD: usize = 10;

/// The stack size of the main task when it is just created. If
/// [`ALLOW_DYNAMIC_STACK`] is set to true, this value can be kept to 0 so
/// that the stack will be allocated completely dynamically.
pub const MAIN_TASK_INITIAL_STACK_SIZE: usize = 16384;

/// The stack size of the idle task when it is just created. If
/// [`ALLOW_DYNAMIC_STACK`] is set to true, this value can be kept to 0 so
/// that the stack will be allocated completely dynamically.
pub const _IDLE_TASK_INITIAL_STACK_SIZE: usize = 1024;

/// The length of the contiguous stack placed at the beginning of the RAM region.
/// The value must match the one in `memory.x`.
pub const _CONTIGUOUS_STACK_LENGTH: u32 = 0x1000;

/// The bottom of the congituous stack.
pub const _CONTIGUOUS_STACK_BOTTOM: u32 = 0x2000_0000 + _CONTIGUOUS_STACK_LENGTH;

/// The boundary of the contiguous stack that its top should not grow beyond.
pub const __CONTIGUOUS_STACK_BOUNDARY: u32 = 0x2000_0020;

/* ########################### */
/* ### Heap Configurations ### */
/* ########################### */

/// The ending address of the heap region.
pub const RAM_END_ADDR: u32 = 0x2002_0000;

/// Free memory chunks use 16-bit links to form linked lists. Since memory
/// chunks are 4-byte aligned, the linkes can represent a range of 2^18 bytes.
/// The represented range is
/// `[__MEM_CHUNK_LINK_OFFSET, __MEM_CHUNK_LINK_OFFSET + 2^18)`.
pub const __MEM_CHUNK_LINK_OFFSET: u32 = 0x2000_0000;

/* ################################ */
/* ### Interrupt Configurations ### */
/* ################################ */

/// The numerical stepping between two adjacent IRQ priority levels.
/// The interrupt priority registers on Cortex-M reserves 8-bits for each
/// interrupt, but for most MCU implementations only a few significant bits
/// are used. For example, if only the top 5 significant bits are used, then
/// the numerical granularity will be 16. If the top 3 significant bits are
/// used, then the numerical granularity will be 32. See Nested Vectored
/// Interrupt Controller (NVIC) in Cortex-M for details.
///
/// If the MCU supports more functional bits than the default configuration,
/// one can reduce the value of [`IRQ_PRIORITY_GRANULARITY`] to a smaller
/// power of 2, and scale up the IRQ priority constants accordingly. This
/// will allow more levels between [`IRQ_MAX_PRIORITY`] and
/// [`IRQ_MIN_PRIORITY`] for applications to use.
pub const IRQ_PRIORITY_GRANULARITY: u8 = 32;

/// The maximum priority of an interrupt. It has the smallest numerical value.
pub const IRQ_MAX_PRIORITY: u8 = 1 * IRQ_PRIORITY_GRANULARITY;

/// The higher priority of an interrupt. Defined for convenience.
pub const IRQ_HIGH_PRIORITY: u8 = 2 * IRQ_PRIORITY_GRANULARITY;

/// The normal priority of an interrupt. Defined for convenience.
pub const IRQ_NORMAL_PRIORITY: u8 = 3 * IRQ_PRIORITY_GRANULARITY;

/// The lower priority of an interrupt. Defined for convenience.
pub const IRQ_LOW_PRIORITY: u8 = 4 * IRQ_PRIORITY_GRANULARITY;

/// The minimum priority of an interrupt. It has the largest numerical value.
pub const IRQ_MIN_PRIORITY: u8 = 5 * IRQ_PRIORITY_GRANULARITY;

/// Hopter globally enables or disables interrupts by configuring the BASEPRI
/// register. When set to 0, no interrupt will be masked.
pub const IRQ_ENABLE_BASEPRI_PRIORITY: u8 = 0 * IRQ_PRIORITY_GRANULARITY;

/// Hopter globally enables or disables interrupts by configuring the BASEPRI
/// register. IRQs with lower or equal priority (greater or qeual numerical
/// value) than BASEPRI are disabled. This value should be set to be higher or
/// equal than all IRQ priority levels.
pub const IRQ_DISABLE_BASEPRI_PRIORITY: u8 = 1 * IRQ_PRIORITY_GRANULARITY;

/// When the interrupt is not globally masked, i.e. the normal case, the SVC
/// is set to a priority lower than all IRQs, so that IRQs can nest above an
/// active SVC and get served promptly.
pub const SVC_NORMAL_PRIORITY: u8 = 6 * IRQ_PRIORITY_GRANULARITY;

/// When the interrupt is globally masked, SVC still need to be allowed because
/// growing segmented stacks depend on it. During the period that the interrupt
/// is globally masked, the priority of SVC is raised to keep it higher than
/// BASEPRI.
pub const SVC_RAISED_PRIORITY: u8 = 0 * IRQ_PRIORITY_GRANULARITY;

/// PendSV is used to implement context switch. Since an SVC may tail chain a
/// PendSV to perform context switch, PendSV's priority must be lower than SVC.
pub const PENDSV_PRIORITY: u8 = 7 * IRQ_PRIORITY_GRANULARITY;

/// The priority of SysTick interrupt.
pub const SYSTICK_PRIORITY: u8 = IRQ_LOW_PRIORITY;

/* ########################### */
/* ### Task Configurations ### */
/* ########################### */

/// The maximum number of tasks. Must be a power of 2.
pub const MAX_TASK_NUMBER: usize = 16;

/// Whether a ready higher priority task should cause a lower priority running
/// task to yield.
pub const ALLOW_TASK_PREEMPTION: bool = true;

/// The number of breathing tasks that can run concurrently, i.e. not blocked
/// on the `wait` function.
pub const BREATHING_CONCURRENCY: usize = 16;

/// Maximum priority number. Lower numerical numbers represent higher priorities.
/// Allowed priority range: 0 to (TASK_PRIORITY_LEVELS - 1).
pub const TASK_PRIORITY_LEVELS: u8 = 16;

/// The priority of the idle task. Typically this is set to the lowest allowed
/// priority.
pub const IDLE_TASK_PRIORITY: u8 = TASK_PRIORITY_LEVELS - 1;

/// The task priority of the main task.
pub const MAIN_TASK_PRIORITY: u8 = 0;

/// The priority for a task when the priority is not explicitly set during
/// task creation.
pub const DEFAULT_TASK_PRIORITY: u8 = 8;

/// A panicked task will get its priority reduced to the unwind priority,
/// which is very low but still higher than idle priority.
pub const UNWIND_PRIORITY: u8 = TASK_PRIORITY_LEVELS - 3;

/// The ID for the idle task. A task ID does not have functional purpose. It
/// might be helpful for diagnosing bugs.
pub const IDLE_TASK_ID: u8 = 0;

/// The ID for the main task. A task ID does not have functional purpose. It
/// might be helpful for diagnosing bugs.
pub const MAIN_TASK_ID: u8 = 1;

/// The ID for a task when the ID is not explicitly set during task creation.
/// Tasks can have the same ID.
pub const DEFAULT_TASK_ID: u8 = 255;

/// The address in memory where the task local storage is placed. Currently
/// this must be the fixed value `0x2000_0000` because the compiler toolchain
/// assumes this value.
pub const __TLS_MEM_ADDR: u32 = 0x2000_0000;