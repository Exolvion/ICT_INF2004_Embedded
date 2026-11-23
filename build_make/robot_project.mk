# INF2004 IS29 robot application. Included at the end of pico_rp2040.mk.
#
# Adds the app_program/ include paths and the few Pico SDK 2.2.0 peripheral
# drivers board.c uses (ADC, I2C, timeout helper; GPIO when the radio build
# does not already compile it). The BSP keeps its own startup; only the
# peripheral sources are imported.
#
# Robot profiles (see scripts/dev.sh build):
#   offline  make SMP=0 CONSOLE=uart WIFI=none
#   mqtt     make SMP=0 CONSOLE=uart WIFI=cyw43 NET=lwip ROBOT_MQTT=1

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(SMP),0)
$(error This application uses core-0 IRQ masking and requires SMP=0)
endif
ifneq ($(CONSOLE),uart)
$(error This robot uses the UART0 console on GP0/GP1 (shared with the HC-SR04 after start-up); use CONSOLE=uart)
endif
ifeq ($(wildcard $(PICO_SDK_PATH)/src/rp2_common/hardware_adc/adc.c),)
$(error Set PICO_SDK_PATH to the Pico SDK 2.2.0 checkout)
endif
endif

ROBOT_SDK_INCLUDES := common/pico_base_headers common/pico_binary_info \
    common/pico_time common/hardware_claim \
    rp2040/pico_platform rp2040/hardware_regs rp2040/hardware_structs \
    rp2_common/pico_platform_common rp2_common/pico_platform_compiler \
    rp2_common/pico_platform_sections rp2_common/pico_platform_panic \
    rp2_common/hardware_base rp2_common/hardware_gpio \
    rp2_common/hardware_adc rp2_common/hardware_i2c rp2_common/hardware_pwm \
    rp2_common/hardware_irq rp2_common/hardware_resets \
    rp2_common/hardware_clocks rp2_common/hardware_timer \
    rp2_common/hardware_sync rp2_common/hardware_sync_spin_lock

# Override the BSP's stale 2.1.0 version header with the actual SDK version.
CFLAGS := -I"./robot_sdk_headers" $(CFLAGS)
CFLAGS += -I"../app_program" \
    -I"../app_program/subsystem1" -I"../app_program/subsystem2" \
    -I"../app_program/subsystem3" -I"../app_program/subsystem4" \
    -I"../app_program/subsystem5" \
    -I"../lib/libtm/sysdepend/pico_rp2040/usb" \
    -DPICO_RP2040=1 -DPICO_32BIT=1 -DPICO_ON_DEVICE=1 \
    -DPICO_NO_HARDWARE=0 -DNDEBUG -ffunction-sections -fdata-sections \
    -fstack-usage \
    $(foreach part,$(ROBOT_SDK_INCLUDES),-I"$(PICO_SDK_PATH)/src/$(part)/include")
LFLAGS += -Wl,--gc-sections
LDLIBS += -lm

ROBOT_SDK_OBJS := ./mtkernel_3/robot_sdk/adc.o \
    ./mtkernel_3/robot_sdk/i2c.o \
    ./mtkernel_3/robot_sdk/timeout_helper.o
ifneq ($(WIFI),cyw43)
ROBOT_SDK_OBJS += ./mtkernel_3/robot_sdk/gpio.o
endif
OBJS += $(ROBOT_SDK_OBJS)
C_DEPS += $(ROBOT_SDK_OBJS:.o=.d)

mtkernel_3/robot_sdk/adc.o: $(PICO_SDK_PATH)/src/rp2_common/hardware_adc/adc.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -c "$<" -o "$@"

mtkernel_3/robot_sdk/i2c.o: $(PICO_SDK_PATH)/src/rp2_common/hardware_i2c/i2c.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -c "$<" -o "$@"

mtkernel_3/robot_sdk/gpio.o: $(PICO_SDK_PATH)/src/rp2_common/hardware_gpio/gpio.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -c "$<" -o "$@"

mtkernel_3/robot_sdk/timeout_helper.o: $(PICO_SDK_PATH)/src/common/pico_time/timeout_helper.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -c "$<" -o "$@"

# ---- RP2040 boot-ROM float and double maths (INF2004, 23 Nov 2026) ----
# The template links libgcc's generic soft-float and newlib's libm, the
# slowest maths on the Cortex-M0+: on 23 Nov the end report's CPU line had
# Motion (odometry, an 11-bucket speed window, two PIDs) at 1.7 ms a step,
# 34 % of the CPU. A CMake Pico SDK build routes the same calls to the boot
# ROM's routines (pico_float, pico_double) by -Wl,--wrap; this does the
# same with the SDK's own sources. board_boot_maths() fills their tables
# from reset_hdl.c, once .bss is set and before anything uses floating
# point. ROBOT_ROM_MATHS=0 keeps libgcc/libm (the frozen bench image,
# firmware/bench/build.sh).
ROBOT_ROM_MATHS ?= 1
ifeq ($(ROBOT_ROM_MATHS),1)
ROBOT_ROM_INCLUDES := rp2_common/pico_float rp2_common/pico_double \
    rp2_common/pico_bootrom rp2_common/hardware_divider \
    rp2_common/pico_runtime_init rp2_common/pico_runtime \
    rp2_common/boot_bootrom_headers common/boot_picoboot_headers \
    common/boot_picobin_headers rp2_common/hardware_boot_lock \
    rp2_common/pico_flash
CFLAGS += -DROBOT_ROM_MATHS=1 \
    $(foreach part,$(ROBOT_ROM_INCLUDES),-I"$(PICO_SDK_PATH)/src/$(part)/include")
ROBOT_ROM_OBJS := ./mtkernel_3/robot_rom/float_aeabi_rp2040.o \
    ./mtkernel_3/robot_rom/float_init_rom_rp2040.o \
    ./mtkernel_3/robot_rom/float_math.o \
    ./mtkernel_3/robot_rom/float_v1_rom_shim_rp2040.o \
    ./mtkernel_3/robot_rom/double_aeabi_rp2040.o \
    ./mtkernel_3/robot_rom/double_init_rom_rp2040.o \
    ./mtkernel_3/robot_rom/double_math.o \
    ./mtkernel_3/robot_rom/double_v1_rom_shim_rp2040.o \
    ./mtkernel_3/robot_rom/bootrom.o
OBJS += $(ROBOT_ROM_OBJS)
C_DEPS += $(ROBOT_ROM_OBJS:.o=.d)
# The SDK's wrap lists (pico_float/pico_double CMakeLists.txt, RP2040).
ROBOT_ROM_WRAP := __aeabi_fadd __aeabi_fdiv __aeabi_fmul __aeabi_frsub \
    __aeabi_fsub __aeabi_cfcmpeq __aeabi_cfrcmple __aeabi_cfcmple \
    __aeabi_fcmpeq __aeabi_fcmplt __aeabi_fcmple __aeabi_fcmpge \
    __aeabi_fcmpgt __aeabi_fcmpun __aeabi_i2f __aeabi_ui2f __aeabi_f2iz \
    __aeabi_f2uiz __aeabi_l2f __aeabi_ul2f __aeabi_f2lz __aeabi_f2ulz \
    __aeabi_f2d sqrtf cosf sinf tanf atan2f expf logf sincosf ldexpf \
    copysignf truncf floorf ceilf roundf asinf acosf atanf sinhf coshf \
    tanhf asinhf acoshf atanhf exp2f log2f exp10f log10f powf powintf \
    hypotf cbrtf fmodf dremf remainderf remquof expm1f log1pf fmaf \
    __aeabi_dadd __aeabi_ddiv __aeabi_dmul __aeabi_drsub __aeabi_dsub \
    __aeabi_cdcmpeq __aeabi_cdrcmple __aeabi_cdcmple __aeabi_dcmpeq \
    __aeabi_dcmplt __aeabi_dcmple __aeabi_dcmpge __aeabi_dcmpgt \
    __aeabi_dcmpun __aeabi_i2d __aeabi_l2d __aeabi_ui2d __aeabi_ul2d \
    __aeabi_d2iz __aeabi_d2lz __aeabi_d2uiz __aeabi_d2ulz __aeabi_d2f \
    sqrt cos sin tan atan2 exp log ldexp copysign trunc floor ceil round \
    sincos asin acos atan sinh cosh tanh asinh acosh atanh exp2 log2 \
    exp10 log10 pow powint hypot cbrt fmod drem remainder remquo expm1 \
    log1p fma
LFLAGS += $(foreach name,$(ROBOT_ROM_WRAP),-Wl,--wrap=$(name))
ROBOT_ROM_ASFLAGS = $(ASFLAGS) $(filter -I% -D%,$(CFLAGS)) -D$(TARGET) $(INCPATH)
endif

mtkernel_3/robot_rom/%.o: $(PICO_SDK_PATH)/src/rp2_common/pico_float/%.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -c "$<" -o "$@"

mtkernel_3/robot_rom/%.o: $(PICO_SDK_PATH)/src/rp2_common/pico_float/%.S
	@mkdir -p "$(@D)"
	$(GCC) $(ROBOT_ROM_ASFLAGS) -c "$<" -o "$@"

mtkernel_3/robot_rom/%.o: $(PICO_SDK_PATH)/src/rp2_common/pico_double/%.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -c "$<" -o "$@"

mtkernel_3/robot_rom/%.o: $(PICO_SDK_PATH)/src/rp2_common/pico_double/%.S
	@mkdir -p "$(@D)"
	$(GCC) $(ROBOT_ROM_ASFLAGS) -c "$<" -o "$@"

mtkernel_3/robot_rom/%.o: $(PICO_SDK_PATH)/src/rp2_common/pico_bootrom/%.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -c "$<" -o "$@"
