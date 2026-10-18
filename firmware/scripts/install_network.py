#!/usr/bin/env python3
"""Install reviewed network integration into the pinned professor checkout.

No downloads, credentials, compiler installation or compilation occur here.
The root installer copies application C/H files separately. Repeated runs
are idempotent; an unfamiliar source shape fails instead of guessing.
"""
from pathlib import Path
import argparse
import subprocess

PINNED_COMMIT = "2e8e6ad480c2d84a369b12995694b9bf5e9ddfb3"
MARKER = "# INF2004 ROBOT_MQTT integration v1"
RADIO_MARKER = "/* INF2004 ROBOT_MQTT radio startup v2 */"


def replace_once(text: str, old: str, new: str, label: str) -> str:
    """Reject drift that could silently attach a hook at the wrong point."""
    if text.count(old) != 1:
        raise ValueError(f"{label}: expected one anchor; source needs review")
    return text.replace(old, new, 1)


def radio_startup_changes(repo_path: Path, owner: str) -> list[tuple[Path, str]]:
    """Apply initial-only retries and propagate owner task startup errors.

    The pinned SPI driver releases PIO/DMA in cyw43_ll_deinit(). The public
    high-level cyw43_deinit() skips that cleanup when cyw43_poll is NULL,
    including failed bus startup. Both cases are handled before reinitializing.
    No reset is attempted once MQTT has acquired a client or network interface.
    """
    owner_path = repo_path / "lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c"
    kernel_path = repo_path / "lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_kernel.c"
    device_path = repo_path / "kernel/sysdepend/pico_rp2040/devinit.c"
    kernel = kernel_path.read_text()
    device = device_path.read_text()
    marked = [RADIO_MARKER in value for value in (owner, kernel, device)]
    if any(marked):
        if not all(marked):
            raise ValueError("Partial radio startup installation; restore first")
        required = [
            (owner, "result = robot_radio_setup();"),
            (owner, "cyw43_ll_deinit(&cyw43_state.cyw43_ll);"),
            (owner, "attempt < ROBOT_RADIO_START_ATTEMPTS"),
            (kernel, "(void)tk_del_tsk(task);"),
            (device, "board_boot_brake();"),
            (device, "if(radio_result < E_OK) return radio_result;"),
        ]
        if any(value.count(anchor) != 1 for value, anchor in required):
            raise ValueError("Radio startup source drift; restore and reinstall")
        return []
    owner = replace_once(owner,
        "static int scan_result(void *env, const cyw43_ev_scan_result_t *result)",
        "#if !ROBOT_MQTT\n"
        "static int scan_result(void *env, const cyw43_ev_scan_result_t *result)",
        "qualification scan callback guard")
    owner = replace_once(owner,
        "void cyw43_utk_task(int32_t stacd, void *exinf)\n{\n"
        "    cyw43_wifi_scan_options_t options = {0};\n",
        "#endif\n\n" + RADIO_MARKER + "\n" + """#if ROBOT_MQTT
#define ROBOT_RADIO_START_ATTEMPTS 3U

/* Only this owner touches the driver. Retry before any MQTT state exists. */
static int32_t robot_radio_setup(void)
{
    uint32_t attempt;

    for(attempt = 0; attempt < ROBOT_RADIO_START_ATTEMPTS; attempt++) {
        cyw43_thread_enter();
        cyw43_wifi_set_up(&cyw43_state, CYW43_ITF_STA, true,
                          CYW43_COUNTRY_WORLDWIDE);
        if((cyw43_state.itf_state & (1u << CYW43_ITF_STA)) != 0) {
            (void)cyw43_wifi_get_mac(&cyw43_state, CYW43_ITF_STA,
                                     radio_status.mac);
            cyw43_thread_exit();
            return 0;
        }
        if(cyw43_poll != NULL) {
            /* This path also removes any registered lwIP interfaces. */
            cyw43_deinit(&cyw43_state);
        } else {
            /* Failed bus setup can still own PIO/DMA. The high-level
             * deinit skips this case; release before clearing bus_data. */
            cyw43_ll_deinit(&cyw43_state.cyw43_ll);
            cyw43_init(&cyw43_state);
        }
        cyw43_thread_exit();
        if(attempt + 1U < ROBOT_RADIO_START_ATTEMPTS) {
            /* Backoff outside driver ownership: 1 s, then 2 s. */
            (void)tk_dly_tsk(1000U << attempt);
        }
    }
    return -CYW43_EIO;
}
#endif

void cyw43_utk_task(int32_t stacd, void *exinf)
{
#if !ROBOT_MQTT
    cyw43_wifi_scan_options_t options = {0};
#endif
""", "radio startup helper")
    owner = replace_once(owner,
        "    cyw43_init(&cyw43_state);\n    cyw43_thread_enter();\n",
        "    cyw43_init(&cyw43_state);\n#if ROBOT_MQTT\n"
        "    result = robot_radio_setup();\n#else\n"
        "    cyw43_thread_enter();\n", "robot setup instead of scan")
    owner = replace_once(owner,
        "    cyw43_thread_exit();\n\n#if TM_WIFI_NETIF\n"
        "    update_netif_status();\n",
        "    cyw43_thread_exit();\n#endif\n\n#if TM_WIFI_NETIF\n"
        "    update_netif_status();\n", "robot setup guard end")
    owner = replace_once(owner,
        "    radio_status.scan_active = (result == 0);\n",
        "#if ROBOT_MQTT\n"
        "    /* No qualification scan: MQTT owns association immediately. */\n"
        "    radio_status.scan_active = 0;\n#else\n"
        "    radio_status.scan_active = (result == 0);\n#endif\n",
        "robot scan state")
    kernel = replace_once(kernel,
        "    return tk_sta_tsk(task, 0);\n",
        RADIO_MARKER + "\n#if ROBOT_MQTT\n"
        "    {\n"
        "        ER result = tk_sta_tsk(task, 0);\n"
        "        if(result < E_OK) {\n"
        "            /* A failed start leaves a dormant task to release. */\n"
        "            (void)tk_del_tsk(task);\n"
        "        }\n"
        "        return result;\n"
        "    }\n#else\n"
        "    return tk_sta_tsk(task, 0);\n#endif\n",
        "owner failed-start cleanup")
    device = replace_once(device,
        "IMPORT ER cyw43_utk_start(void);\n",
        "IMPORT ER cyw43_utk_start(void);\n#if ROBOT_MQTT\n"
        "/* Early fixed-width board hook; no SDK/kernel typedef mixing. */\n"
        "IMPORT void board_boot_brake(void);\n#endif\n",
        "early brake declaration")
    device = replace_once(device,
        "\t(void)cyw43_utk_start();\n",
        RADIO_MARKER + "\n#if ROBOT_MQTT\n"
        "\t{\n"
        "\t\tER radio_result;\n"
        "\t\t/* usermain is skipped on errors: establish BRAKE first. */\n"
        "\t\tboard_boot_brake();\n"
        "\t\tradio_result = cyw43_utk_start();\n"
        "\t\tif(radio_result < E_OK) return radio_result;\n"
        "\t}\n#else\n"
        "\t(void)cyw43_utk_start();\n#endif\n",
        "owner startup error propagation")
    return [(owner_path, owner), (kernel_path, kernel), (device_path, device)]


def install_network(repo_path: Path) -> list[str]:
    """Patch Make, lwIP options and owner task; return changed relative paths."""
    repo_path = Path(repo_path).resolve()
    head = subprocess.check_output(
        ["git", "-C", str(repo_path), "rev-parse", "HEAD"], text=True
    ).strip()
    if head != PINNED_COMMIT:
        raise ValueError(f"Expected professor commit {PINNED_COMMIT}, got {head}")
    make_path = repo_path / "build_make/pico_rp2040.mk"
    opts_path = repo_path / "lib/libnet/lwip/include/lwipopts.h"
    owner_path = repo_path / "lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c"
    make = make_path.read_text()
    opts = opts_path.read_text()
    owner = owner_path.read_text()
    if MARKER in make:
        if ("net_mqtt_poll(" not in owner) or ("MQTT_OUTPUT_RINGBUF_SIZE" not in opts):
            raise ValueError("Partial MQTT installation; restore and reinstall")
        changes = radio_startup_changes(repo_path, owner)
        for path, content in changes:
            path.write_text(content)
        return [str(path.relative_to(repo_path)) for path, _ in changes]
    make = replace_once(make, "WIFI ?= none\n", """WIFI ?= none
# INF2004 ROBOT_MQTT integration v1
# This independent application profile does not enable upstream echo tests.
ROBOT_MQTT ?= 0
ifneq ($(filter $(ROBOT_MQTT),0 1),$(ROBOT_MQTT))
$(error ROBOT_MQTT must be 0 or 1)
endif
ifeq ($(ROBOT_MQTT),1)
ifneq ($(WIFI),cyw43)
$(error ROBOT_MQTT=1 requires WIFI=cyw43)
endif
ifneq ($(WIFI_JOIN),1)
$(error ROBOT_MQTT=1 requires WIFI_JOIN=1)
endif
ifneq ($(WIFI_NETIF),1)
$(error ROBOT_MQTT=1 requires WIFI_NETIF=1)
endif
ifneq ($(WIFI_DHCP),1)
$(error ROBOT_MQTT=1 requires WIFI_DHCP=1)
endif
ifneq ($(filter 1,$(WIFI_DNS) $(WIFI_UDP) $(WIFI_TCP) $(WIFI_TCPBULK)),)
$(error Disable upstream DNS/UDP/TCP qualification demos for ROBOT_MQTT)
endif
ifeq ($(wildcard ../config/net_mqtt_config.h),)
$(error Copy robot net_mqtt_config.example.h to config/net_mqtt_config.h)
endif
endif
""", "MQTT profile")
    make = replace_once(make, "PROFILE_FILE := .build_profile\n",
                        "PROFILE_ID := $(PROFILE_ID)-robotmqtt$(ROBOT_MQTT)\n"
                        "PROFILE_FILE := .build_profile\n", "profile identity")
    make = replace_once(make,
                        "LWIP_OBJS := $(addprefix ./mtkernel_3/lwip/,$(LWIP_SRCS:.c=.o))",
                        "ifeq ($(ROBOT_MQTT),1)\n"
                        "LWIP_SRCS += apps/mqtt/mqtt.c\n"
                        "endif\n"
                        "LWIP_OBJS := $(addprefix ./mtkernel_3/lwip/,$(LWIP_SRCS:.c=.o))",
                        "MQTT source list")
    make = replace_once(make, "CFLAGS += -DTM_WIFI_CYW43=1",
                        "CFLAGS += -DROBOT_MQTT=$(ROBOT_MQTT) "
                        "-I\"../app_program\"\n"
                        "CFLAGS += -DTM_WIFI_CYW43=1", "MQTT include path")
    opts = replace_once(opts, "#define LWIP_TCP                        TM_WIFI_TCP",
                        "#define LWIP_TCP                        "
                        "(TM_WIFI_TCP || ROBOT_MQTT)", "TCP feature")
    opts = replace_once(opts, "#define LWIP_ALTCP                      0",
                        "#define LWIP_ALTCP                      ROBOT_MQTT",
                        "ALTCP feature")
    opts = opts.replace("#if TM_WIFI_TCP\n",
                        "#if TM_WIFI_TCP || ROBOT_MQTT\n")
    opts = replace_once(opts, "#define MEMP_NUM_SYS_TIMEOUT            8",
                        "#define MEMP_NUM_SYS_TIMEOUT            12",
                        "MQTT timer pool")
    opts = replace_once(opts, "#define LWIP_STATS                      0",
                        "/* Bounded application packet storage: topic + 256 bytes. */\n"
                        "#define MQTT_OUTPUT_RINGBUF_SIZE        1024\n"
                        "#define MQTT_VAR_HEADER_BUFFER_LEN      256\n"
                        "#define MQTT_CONNECT_TIMOUT             30\n"
                        "#define MQTT_REQ_TIMEOUT                15\n\n"
                        "#define LWIP_STATS                      0", "MQTT limits")
    owner = replace_once(owner, '#include "cyw43_utk.h"\n',
                         '#include "cyw43_utk.h"\n'
                         '#if ROBOT_MQTT\n#include "net_mqtt.h"\n#endif\n',
                         "owner include")
    # Robot service owns association/reassociation. Avoid two owners issuing join.
    owner = owner.replace("#if TM_WIFI_JOIN\n",
                          "#if TM_WIFI_JOIN && !ROBOT_MQTT\n")
    owner = replace_once(owner, "        sys_check_timeouts();\n",
                         "        sys_check_timeouts();\n"
                         "#if ROBOT_MQTT\n"
                         "        net_mqtt_poll(&cyw43_state.netif[CYW43_ITF_STA]);\n"
                         "#endif\n", "radio poll hook")
    changes = [(make_path, make), (opts_path, opts)]
    changes += radio_startup_changes(repo_path, owner)
    # All anchors are validated before the first mutation.
    for path, content in changes:
        path.write_text(content)
    return [str(path.relative_to(repo_path)) for path, _ in changes]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("repository", type=Path)
    args = parser.parse_args()
    changes = install_network(args.repository)
    print("Network integration installed: " + (", ".join(changes) or "already present"))


if __name__ == "__main__":
    main()
