#!/usr/bin/env python3
"""Run installed radio startup source with mocked driver/kernel boundaries.

Uses a fresh local clone and actual Pico SDK headers for cyw43_utk.c. The
kernel-facing files use explicit host stubs to keep the two size_t ABIs apart.
This does not execute ARM code, PIO/DMA teardown or radio/network hardware.

The stubs check with assert(). The owner compile line is copied from the
port makefile, which adds -DNDEBUG, so both stubs start with #undef NDEBUG
and the owner command appends -UNDEBUG. Every build also runs a negative
control that must abort on a failed assertion, proving the checks are live.
UBSan findings are fatal (-fno-sanitize-recover=all, halt_on_error=1).
"""
from pathlib import Path
import argparse
import hashlib
import json
import os
import shlex
import shutil
import subprocess
import tempfile

from install_network import install_network


OWNER_STUB = r'''
/* The checks below must run whatever flags the port makefile adds. */
#undef NDEBUG
#include "cyw43.h"
#include "cyw43_utk.h"
#include "lwip/dhcp.h"
#include "lwip/dns.h"
#include "lwip_utk_ipv4.h"
#include "net_mqtt.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

cyw43_t cyw43_state;
void (*cyw43_poll)(void);
volatile uint32_t cyw43_utk_poll_requested;
struct netif *netif_default;
static jmp_buf finish;
static int scenario;
static unsigned attempts, low_teardowns, full_teardowns, init_calls;
static unsigned scans, mqtt_calls, timeout_calls, lwip_calls, backoffs;
static unsigned owned_resources, thread_depth;
static uint32_t delays[2];

static void fake_poll(void) { }
void mp_memory_barrir(void) { }
uint64_t time_us_64(void) { return 1000; }
void cyw43_thread_enter(void) { thread_depth++; }
void cyw43_thread_exit(void) { assert(thread_depth > 0); thread_depth--; }
void lwip_init(void) { lwip_calls++; }
void sys_check_timeouts(void) { timeout_calls++; }
void net_mqtt_poll(struct netif *n)
{
    assert(n == &cyw43_state.netif[CYW43_ITF_STA]);
    assert(cyw43_state.itf_state != 0);
    mqtt_calls++;
}
void cyw43_init(cyw43_t *s)
{
    assert(owned_resources == 0);
    init_calls++;
    memset(s, 0, sizeof(*s));
    cyw43_poll = NULL;
}
void cyw43_ll_deinit(cyw43_ll_t *s)
{
    assert(s == &cyw43_state.cyw43_ll);
    assert(thread_depth > 0);
    low_teardowns++;
    owned_resources = 0;
}
void cyw43_deinit(cyw43_t *s)
{
    assert(cyw43_poll != NULL);
    assert(thread_depth > 0);
    full_teardowns++;
    owned_resources = 0;
    cyw43_init(s);
}
void cyw43_wifi_set_up(cyw43_t *s, int itf, bool up, uint32_t country)
{
    (void)country;
    assert(up && itf == CYW43_ITF_STA && thread_depth > 0);
    assert(owned_resources == 0);
    owned_resources = 1;
    attempts++;
    if((scenario == 1 || scenario == 2) && attempts < 3) {
        if(scenario == 2) cyw43_poll = fake_poll;
        return;
    }
    if(scenario == 3 || scenario == 4) {
        if(scenario == 4) cyw43_poll = fake_poll;
        return;
    }
    s->itf_state = 1;
    s->netif[itf].state = s;
    cyw43_poll = fake_poll;
}
int cyw43_wifi_get_mac(cyw43_t *s, int itf, uint8_t mac[6])
{
    (void)s; (void)itf;
    memset(mac, 1, 6);
    return 0;
}
int cyw43_wifi_scan(cyw43_t *s, cyw43_wifi_scan_options_t *o, void *env,
                   int (*callback)(void *, const cyw43_ev_scan_result_t *))
{
    (void)s; (void)o; (void)env; (void)callback;
    assert(ROBOT_MQTT == 0);
    scans++;
    return scenario == 5 ? -9 : 0;
}
int cyw43_wifi_join(cyw43_t *s, size_t a, const uint8_t *b,
                   size_t c, const uint8_t *d, uint32_t e,
                   const uint8_t *f, uint32_t g)
{
    (void)s; (void)a; (void)b; (void)c; (void)d; (void)e; (void)f; (void)g;
    return 0;
}
int cyw43_wifi_link_status(cyw43_t *s, int itf)
{ (void)s; (void)itf; return CYW43_LINK_DOWN; }
int cyw43_wifi_get_bssid(cyw43_t *s, uint8_t bssid[6])
{ (void)s; memset(bssid, 0, 6); return 0; }
int cyw43_wifi_get_rssi(cyw43_t *s, int32_t *rssi)
{ (void)s; *rssi = -90; return 0; }
u8_t dhcp_supplied_address(const struct netif *n) { (void)n; return 0; }
const ip_addr_t *dns_getserver(u8_t i) { (void)i; return NULL; }
int32_t lwip_utk_ipv4_adopt_netif(struct netif *n) { (void)n; return 0; }
void lwip_utk_ipv4_poll(struct netif *n) { (void)n; }
void lwip_utk_ipv4_get_status(T_LWIP_UTK_IPV4_STATUS *s)
{ memset(s, 0, sizeof(*s)); }
int32_t tk_dly_tsk(uint32_t delay)
{
    assert(thread_depth == 0);
    if(delay == 2) longjmp(finish, 1);
    if(!ROBOT_MQTT || attempts == 3) longjmp(finish, 2);
    assert(backoffs < 2 && owned_resources == 0);
    delays[backoffs++] = delay;
    return 0;
}
int main(int argc, char **argv)
{
    T_CYW43_UTK_STATUS status;
    assert(argc == 2);
    scenario = argv[1][0] - '0';
    if(scenario == 9) {
        /* Negative control: the harness requires this check to abort. */
        assert(scenario != 9);
        printf("NEGATIVE CONTROL NOT TRIGGERED\n");
        return 0;
    }
    int outcome = setjmp(finish);
    if(outcome == 0) cyw43_utk_task(0, NULL);
    cyw43_utk_get_status(&status);
    assert(lwip_calls == 1 && status.init_complete == 1);
    assert(thread_depth == 0);
    if(ROBOT_MQTT) {
        assert(scans == 0 && status.scan_active == 0);
        assert(attempts == (scenario == 0 ? 1U : 3U));
        assert(backoffs == (scenario == 0 ? 0U : 2U));
        if(backoffs != 0) assert(delays[0] == 1000 && delays[1] == 2000);
        if(scenario == 3 || scenario == 4) {
            assert(outcome == 2 && status.ready == 0);
            assert(status.init_result == -CYW43_EIO);
            assert(mqtt_calls == 0 && timeout_calls == 0);
            assert(owned_resources == 0 && init_calls == 4);
        } else {
            assert(outcome == 1 && status.ready == 1);
            assert(status.init_result == 0);
            assert(mqtt_calls == 1 && timeout_calls == 1);
            assert(owned_resources == 1 && init_calls == attempts);
        }
        assert(low_teardowns == (scenario == 1 ? 2U :
                                scenario == 3 ? 3U : 0U));
        assert(full_teardowns == (scenario == 2 ? 2U :
                                 scenario == 4 ? 3U : 0U));
    } else {
        assert(scans == 1 && attempts == 1 && backoffs == 0);
        assert(low_teardowns == 0 && full_teardowns == 0);
        assert(mqtt_calls == 0 && init_calls == 1);
        assert(outcome == (scenario == 5 ? 2 : 1));
        assert(status.init_result == (scenario == 5 ? -9 : 0));
    }
    printf("PASS radio profile=%d scenario=%d\n", ROBOT_MQTT, scenario);
    return 0;
}
'''

KERNEL_HEADER = r'''
#ifndef HOST_KERNEL_H
#define HOST_KERNEL_H
#define EXPORT
#define IMPORT extern
#define E_OK 0
#define TA_HLNG 1
#define TA_RNG1 2
#define PICO_RP2040 1
#define TM_WIFI_CYW43 1
#define TK_SUPPORT_SMP 0
#define USE_OBJECT_NAME 0
#define USE_SDEV_DRV 0
#define USE_TMONITOR 0
#define USE_SHUTDOWN 1
typedef int ER;
typedef int ID;
typedef int INT;
typedef void (*FP)(void);
typedef struct { int tskatr; FP task; int itskpri; int stksz; } T_CTSK;
int tk_cre_tsk(T_CTSK *definition);
int tk_sta_tsk(ID task, int code);
int tk_del_tsk(ID task);
int cyw43_utk_start(void);
int knl_start_device(void);
#endif
'''

KERNEL_STUB = r'''
/* The checks below must run whatever flags the build adds. */
#undef NDEBUG
#include "host_kernel.h"
#include <assert.h>
#include <stdio.h>
static int create_result, start_result, creates, starts, deletes, brakes;
int tk_cre_tsk(T_CTSK *definition)
{
    assert(definition->stksz == 6144);
    if(ROBOT_MQTT) assert(brakes == 1);
    creates++;
    return create_result;
}
int tk_sta_tsk(ID task, int code)
{ assert(task == 7 && code == 0); starts++; return start_result; }
int tk_del_tsk(ID task) { assert(task == 7); deletes++; return 0; }
void board_boot_brake(void) { assert(creates == 0); brakes++; }
void cyw43_utk_task(INT a, void *b) { (void)a; (void)b; }
int main(int argc, char **argv)
{
    (void)argv;
    if(argc > 1) {
        /* Negative control: the harness requires this check to abort. */
        assert(argc == 1);
        printf("NEGATIVE CONTROL NOT TRIGGERED\n");
        return 0;
    }
    for(int scenario = 0; scenario < 3; scenario++) {
        creates = starts = deletes = brakes = 0;
        create_result = scenario == 1 ? -10 : 7;
        start_result = scenario == 2 ? -20 : 0;
        int result = knl_start_device();
        int expected = scenario == 1 ? -10 : scenario == 2 ? -20 : 0;
        assert(result == (ROBOT_MQTT ? expected : 0));
        assert(creates == 1 && starts == (scenario == 1 ? 0 : 1));
        assert(deletes == (ROBOT_MQTT && scenario == 2 ? 1 : 0));
        assert(brakes == (ROBOT_MQTT ? 1 : 0));
        printf("PASS kernel profile=%d scenario=%d\n", ROBOT_MQTT, scenario);
    }
    return 0;
}
'''


def run(command: list[str], cwd: Path, log: Path, env=None) -> str:
    result = subprocess.run(command, cwd=cwd, env=env, text=True,
                            capture_output=True)
    log.write_text(result.stdout + result.stderr +
                   f"\nexit={result.returncode}\n")
    if result.returncode:
        raise RuntimeError(f"Command failed; see {log}")
    return result.stdout


def run_negative_control(command: list[str], cwd: Path, log: Path,
                         env) -> str:
    """Require a deliberately false assert() to abort the binary."""
    result = subprocess.run(command, cwd=cwd, env=env, text=True,
                            capture_output=True)
    output = result.stdout + result.stderr
    log.write_text(output + f"\nexit={result.returncode}\n")
    if result.returncode == 0 or "NEGATIVE CONTROL NOT TRIGGERED" in output:
        raise RuntimeError("Negative control did not fail: assertions are "
                           f"not live in this build; see {log}")
    if "Assertion" not in output:
        raise RuntimeError("Negative control stopped (exit "
                           f"{result.returncode}) before its assertion, for "
                           f"example on a sanitizer error; see {log}")
    return f"aborted as required (exit {result.returncode})"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository", required=True, type=Path)
    parser.add_argument("--sdk", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--cc", default="cc")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    firmware = Path(__file__).resolve().parents[1]
    report = {"scope": "Installed source with host driver/kernel stubs",
              "hardware_test": False, "arm_build": False, "results": []}
    with tempfile.TemporaryDirectory(prefix="inf2004_radio_") as temporary:
        temp = Path(temporary)
        port = temp / "port"
        run(["git", "clone", "--quiet", "--no-hardlinks", "--shared",
             str(args.repository.resolve()), str(port)], temp,
            output / "clone.log")
        changed = install_network(port)
        assert install_network(port) == []
        final_bytes = {name: (port / name).read_bytes() for name in changed}
        owner_name = "lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c"
        kernel_name = "lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_kernel.c"
        device_name = "kernel/sysdepend/pico_rp2040/devinit.c"
        # Reconstruct the previous v1 hook to prove an in-place upgrade.
        old_owner = subprocess.check_output(
            ["git", "-C", str(port), "show", "HEAD:" + owner_name], text=True)
        old_owner = old_owner.replace('#include "cyw43_utk.h"\n',
            '#include "cyw43_utk.h"\n#if ROBOT_MQTT\n'
            '#include "net_mqtt.h"\n#endif\n')
        old_owner = old_owner.replace("#if TM_WIFI_JOIN\n",
                                      "#if TM_WIFI_JOIN && !ROBOT_MQTT\n")
        old_owner = old_owner.replace("        sys_check_timeouts();\n",
            "        sys_check_timeouts();\n#if ROBOT_MQTT\n"
            "        net_mqtt_poll(&cyw43_state.netif[CYW43_ITF_STA]);\n#endif\n")
        (port / owner_name).write_text(old_owner)
        for name in (kernel_name, device_name):
            (port / name).write_bytes(subprocess.check_output(
                ["git", "-C", str(port), "show", "HEAD:" + name]))
        assert set(install_network(port)) == {owner_name, kernel_name, device_name}
        assert all((port / name).read_bytes() == value
                   for name, value in final_bytes.items())
        for name, text_to_remove in [
            (device_name, "/* INF2004 ROBOT_MQTT radio startup v2 */"),
            (owner_name, "result = robot_radio_setup();")]:
            (port / name).write_text((port / name).read_text().replace(
                text_to_remove, "/* injected test-only source drift */", 1))
            before = {item: (port / item).read_bytes() for item in changed}
            try:
                install_network(port)
            except ValueError:
                pass
            else:
                raise AssertionError("Installer accepted a partial/drifted hook")
            assert all((port / item).read_bytes() == value
                       for item, value in before.items())
            (port / name).write_bytes(final_bytes[name])
        report["installer_checks"] = ["Fresh install", "Idempotent reinstall",
            "v1 upgrade", "Partial-install rejection without writes",
            "Owner-source drift rejection without writes"]
        report["installed_sha256"] = {
            name: hashlib.sha256((port / name).read_bytes()).hexdigest()
            for name in changed}
        shutil.copy2(firmware / "target/net_mqtt.h", port / "app_program")
        shutil.copy2(firmware / "target/net_mqtt_config.example.h",
                     port / "config/net_mqtt_config.h")
        (port / "config/wifi_credentials.h").write_text(
            '#define UTK_WIFI_SSID "test"\n'
            '#define UTK_WIFI_PASSWORD "testpass"\n'
            '#define UTK_WIFI_AUTH CYW43_AUTH_WPA2_AES_PSK\n')
        work = port / "build_make"
        plan = run(["make", "-n", "SMP=0", "CONSOLE=uart", "ROBOT_MQTT=1",
                    "WIFI=cyw43", "WIFI_JOIN=1", "WIFI_NETIF=1", "WIFI_DHCP=1",
                    "WIFI_DNS=0", "WIFI_UDP=0", "WIFI_TCP=0", "WIFI_TCPBULK=0",
                    f"PICO_SDK_PATH={args.sdk.resolve()}"], work,
                   output / "make-plan.log")
        owner = "../lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c"
        line = next(line for line in plan.splitlines()
                    if line.startswith("arm-none-eabi-gcc ")
                    and line.endswith('"' + owner + '"'))
        command = shlex.split(line)
        command[0] = args.cc
        position = command.index("-o")
        del command[position:position + 2]
        command = [item for item in command if not item.startswith(
            ("-mcpu=", "-mthumb", "-mfloat-abi=", "-MF", "-MT", "-MMD", "-MP"))
            and item not in ("-c", "-ffreestanding", owner)]
        command = ["-isystem" + item[2:] if item.startswith("-I") else item
                   for item in command]
        host_boundary = temp / "owner_boundary.h"
        host_boundary.write_text(
            '#define CYW43_SMP_COMPAT_H\n#define CYW43_THIS_PRC() 1\n')
        stub = temp / "owner_stub.c"
        stub.write_text(OWNER_STUB)
        # The port makefile passes -DNDEBUG; -UNDEBUG (after it) keeps the
        # stub's assert() checks compiled in.
        command += ["-D__unused=__attribute__((unused))",
                    "-DSSIZE_MAX=__LONG_MAX__", "-D_POSIX_C_SOURCE=200809L",
                    "-include", str(host_boundary), "-Werror", "-UNDEBUG",
                    owner, str(stub)]
        report["owner_host_command"] = command
        headers = temp / "headers"
        headers.mkdir()
        (headers / "host_kernel.h").write_text(KERNEL_HEADER)
        for name in ["tk/tkernel.h", "sys/machine.h", "sys/sysdef.h",
                     "tm/tmonitor.h", "tk/device.h", "kernel.h", "sysdepend.h"]:
            target = headers / name
            target.parent.mkdir(exist_ok=True)
            target.write_text('#include "host_kernel.h"\n')
        kernel_stub = temp / "kernel_stub.c"
        kernel_stub.write_text(KERNEL_STUB)
        kernel_sources = []
        for name in ["lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_kernel.c",
                     "kernel/sysdepend/pico_rp2040/devinit.c"]:
            target = temp / Path(name).name
            shutil.copy2(port / name, target)
            assert target.read_bytes() == (port / name).read_bytes()
            kernel_sources.append(str(target))
        env = os.environ.copy()
        # LeakSanitizer is off by project choice until it has been tried on
        # the CI runner; ROBOT_LSAN=1 enables it for a local run.
        leak_checks = os.environ.get("ROBOT_LSAN") == "1"
        env["ASAN_OPTIONS"] = f"detect_leaks={int(leak_checks)}:halt_on_error=1"
        env["UBSAN_OPTIONS"] = "halt_on_error=1:print_stacktrace=1"
        report["leak_sanitizer"] = ("enabled (ROBOT_LSAN=1)" if leak_checks else
                                    "disabled by project choice pending a CI "
                                    "trial; ROBOT_LSAN=1 enables it")
        report["negative_controls"] = []
        for mode, extra in [("release", ["-O2"]),
                            ("sanitize", ["-O1", "-g",
                                          "-fsanitize=address,undefined,"
                                          "float-cast-overflow",
                                          "-fno-sanitize-recover=all",
                                          "-fno-omit-frame-pointer"])]:
            for profile in (1, 0):
                tag = f"{mode}-profile{profile}"
                binary = temp / tag
                profile_command = [item.replace("-DROBOT_MQTT=1",
                    f"-DROBOT_MQTT={profile}") for item in command]
                run(profile_command + extra + ["-o", str(binary)], work,
                    output / (tag + "-build.log"))
                report["negative_controls"].append(f"{tag} owner: " +
                    run_negative_control([str(binary), "9"], work,
                        output / f"{tag}-negative-control.log", env))
                for scenario in (range(5) if profile else (0, 5)):
                    result = run([str(binary), str(scenario)], work,
                                 output / f"{tag}-scenario{scenario}.log", env)
                    report["results"].append(result.strip())
                kernel_binary = temp / (tag + "-kernel")
                kernel_command = [args.cc, "-std=c99", "-Wall", "-Wextra",
                                  "-Werror", f"-DROBOT_MQTT={profile}",
                                  f"-I{headers}"] + kernel_sources + [
                                  str(kernel_stub)] + extra + ["-o", str(kernel_binary)]
                run(kernel_command, work, output / (tag + "-kernel-build.log"))
                report["negative_controls"].append(f"{tag} kernel: " +
                    run_negative_control([str(kernel_binary), "negative"], work,
                        output / f"{tag}-kernel-negative-control.log", env))
                result = run([str(kernel_binary)], work,
                             output / (tag + "-kernel.log"), env)
                report["results"].extend(result.strip().splitlines())
        report["scenario_executions"] = len(report["results"])
        report["status"] = "PASS"
    (output / "radio-startup-report.json").write_text(
        json.dumps(report, indent=2) + "\n")
    print(f"PASS: {report['scenario_executions']} radio/kernel scenario executions; "
          f"{len(report['negative_controls'])} negative controls aborted as required")


if __name__ == "__main__":
    main()
