#!/usr/bin/env python3
"""Fail-closed structural gate for this project's unstripped RP2040 ELF/UF2.

This checks build artifacts, not execution, stack high-water marks, or hardware.
Only Python's standard library is required. The UF2 padding rules match the
pinned mtk3smp-rp2040/tools/elf2uf2/main.cpp flash-image converter.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import struct
import sys

FLASH_START = 0x10000000
FLASH_END = FLASH_START + 2 * 1024 * 1024
RAM_START = 0x20000000
RAM_END = 0x20040000
HEAP_LIMIT = RAM_END - 2048
MINIMUM_HEAP = 65536
PAGE_SIZE = 256
SECTOR_SIZE = 4096
SYMBOL_NAMES = (
    "__boot2_start__", "__boot2_end__", "__vector_org", "__vector_start",
    "__vector_end", "__data_org", "__data_start", "__data_end",
    "__bss_start", "__bss_end", "__end",
)


class GateError(ValueError):
    """An artifact does not satisfy the selected project's release gate."""


def require(condition, message):
    if not condition:
        raise GateError(message)


def chunk(blob, offset, size, description):
    require(0 <= offset <= len(blob) and 0 <= size <= len(blob) - offset,
            description + " extends outside the file")
    return blob[offset:offset + size]


def c_string(blob, offset):
    require(0 <= offset < len(blob), "string-table offset is invalid")
    end = blob.find(b"\0", offset)
    require(end >= 0, "unterminated string-table entry")
    try:
        return blob[offset:end].decode("ascii")
    except UnicodeDecodeError as exc:
        raise GateError("non-ASCII ELF section or symbol name") from exc


def in_range(start, size, lower, upper):
    return lower <= start <= upper and 0 <= size <= upper - start


def parse_elf(blob):
    require(len(blob) >= 52, "ELF header is truncated")
    require(blob[:7] == b"\x7fELF\x01\x01\x01", "require ELF32 little-endian version 1")
    require(blob[7] == 0, "require System V ELF OSABI")
    fields = struct.unpack_from("<HHIIIIIHHHHHH", blob, 16)
    (kind, machine, version, entry, phoff, shoff, flags, ehsize,
     phsize, phnum, shsize, shnum, shstrndx) = fields
    require(kind == 2 and machine == 40 and version == 1,
            "require an ARM ET_EXEC image")
    require(flags >> 24 == 5 and not flags & 0x400,
            "require ARM EABI5 without hard-float ABI")
    require(ehsize == 52 and phsize == 32 and phnum > 0,
            "invalid or absent ELF32 program headers")
    require(shsize == 40 and 0 < shnum < 0xFF00 and 0 < shstrndx < shnum,
            "require ordinary ELF32 section headers and an unstripped symbol table")
    require(entry & 1 and FLASH_START <= (entry & ~1) < FLASH_END,
            "entry must be a Thumb address in the configured flash")
    chunk(blob, phoff, phsize * phnum, "program-header table")
    chunk(blob, shoff, shsize * shnum, "section-header table")
    sections = [struct.unpack_from("<10I", blob, shoff + i * shsize)
                for i in range(shnum)]
    names_header = sections[shstrndx]
    require(names_header[1] == 3, "section-name table must be SHT_STRTAB")
    names = chunk(blob, names_header[4], names_header[5], "section-name table")
    named_sections = {}
    for section in sections[1:]:
        name = c_string(names, section[0])
        require(name not in named_sections, "duplicate ELF section name: " + name)
        named_sections[name] = section
        if section[1] != 8:
            chunk(blob, section[4], section[5], "section " + name)
        if section[2] & 2 and section[5]:
            require(in_range(section[3], section[5], FLASH_START, FLASH_END) or
                    in_range(section[3], section[5], RAM_START, HEAP_LIMIT),
                    "allocated section outside configured flash/static RAM: " + name)
    symbols = {}
    for table in sections:
        if table[1] != 2:
            continue
        require(table[9] == 16 and table[5] % 16 == 0,
                "invalid ELF32 symbol-table entry size")
        require(0 < table[6] < shnum and sections[table[6]][1] == 3,
                "symbol table has no valid string table")
        strings_header = sections[table[6]]
        strings = chunk(blob, strings_header[4], strings_header[5], "symbol strings")
        data = chunk(blob, table[4], table[5], "symbol table")
        for offset in range(0, len(data), 16):
            name_offset, value, _, _, _, index = struct.unpack_from("<IIIBBH", data, offset)
            if index == 0:
                continue
            name = c_string(strings, name_offset)
            if name in SYMBOL_NAMES:
                require(name not in symbols or symbols[name] == value,
                        "conflicting linker symbol: " + name)
                symbols[name] = value
    missing = sorted(set(SYMBOL_NAMES) - symbols.keys())
    require(not missing, "missing linker symbols: " + ", ".join(missing))
    require(symbols["__boot2_start__"] == FLASH_START and
            symbols["__boot2_end__"] == FLASH_START + PAGE_SIZE,
            "boot2 must occupy the first 256 flash bytes")
    require(symbols["__vector_org"] == FLASH_START + PAGE_SIZE,
            "flash vector table must immediately follow boot2")
    require(symbols["__vector_start"] == RAM_START and
            symbols["__vector_start"] % PAGE_SIZE == 0 and
            0 < symbols["__vector_end"] - RAM_START <= PAGE_SIZE,
            "RAM vector table must be nonempty and 256-byte aligned at RAM start")
    for name in ("__vector_end", "__data_org", "__data_start", "__data_end",
                 "__bss_start", "__bss_end"):
        require(symbols[name] % 4 == 0, name + " must be word aligned")
    heap_start = (symbols["__bss_end"] + 7) & ~7
    require(RAM_START < symbols["__vector_end"] <= symbols["__data_start"] <=
            symbols["__data_end"] <= symbols["__bss_start"] <=
            symbols["__bss_end"] == symbols["__end"] and
            heap_start <= HEAP_LIMIT - MINIMUM_HEAP,
            "RAM ordering or minimum 65536-byte dynamic heap reserve is invalid")
    data_size = symbols["__data_end"] - symbols["__data_start"]
    require(in_range(symbols["__data_org"], data_size, FLASH_START, FLASH_END),
            "initialized data load image lies outside flash")
    expected_sections = {
        "boot2": (1, symbols["__boot2_start__"], PAGE_SIZE),
        "vector": (8, symbols["__vector_start"], symbols["__vector_end"] - RAM_START),
        ".data": (1, symbols["__data_start"], data_size),
        ".bss": (8, symbols["__bss_start"], symbols["__bss_end"] - symbols["__bss_start"]),
    }
    for name, (kind_expected, address, size) in expected_sections.items():
        section = named_sections.get(name)
        require(section is not None, "missing expected section: " + name)
        require(section[1] == kind_expected and section[3] == address and
                section[5] == size and section[2] & 2,
                "section/linker-symbol disagreement: " + name)
    for name, section in named_sections.items():
        if section[2] & 2 and RAM_START <= section[3] < RAM_END:
            require(section[3] + section[5] <= symbols["__bss_end"],
                    "RAM section extends into declared heap: " + name)
    pages = {}
    used = set()
    segments = []
    file_mappings = []
    entry_mapped = False
    data_mapped = data_size == 0
    for i in range(phnum):
        kind, offset, vaddr, paddr, filesz, memsz, segment_flags, align = struct.unpack_from(
            "<8I", blob, phoff + i * phsize)
        if kind != 1:
            continue
        require(filesz <= memsz, "PT_LOAD filesz exceeds memsz")
        require(align in (0, 1) or (align & (align - 1) == 0 and
                vaddr % align == offset % align), "invalid PT_LOAD alignment")
        require(in_range(vaddr, memsz, FLASH_START, FLASH_END) or
                in_range(vaddr, memsz, RAM_START, symbols["__bss_end"]),
                "PT_LOAD virtual range is outside configured static memory")
        if filesz == 0:
            continue
        require(in_range(paddr, filesz, FLASH_START, FLASH_END),
                "file-backed PT_LOAD physical range must be in flash")
        payload = chunk(blob, offset, filesz, "PT_LOAD payload")
        file_mappings.append((offset, vaddr, filesz))
        if segment_flags & 1 and vaddr <= (entry & ~1) < vaddr + filesz:
            entry_mapped = True
        if (vaddr <= symbols["__data_start"] and
                symbols["__data_end"] <= vaddr + filesz and
                paddr + symbols["__data_start"] - vaddr == symbols["__data_org"]):
            data_mapped = True
        for n, byte in enumerate(payload):
            address = paddr + n
            require(address not in used, "overlapping file-backed PT_LOAD ranges")
            used.add(address)
            page = address & ~(PAGE_SIZE - 1)
            pages.setdefault(page, bytearray(PAGE_SIZE))[address - page] = byte
        segments.append({"virtual_address": vaddr, "physical_address": paddr,
                         "file_bytes": filesz, "memory_bytes": memsz})
    for name, section in named_sections.items():
        if section[2] & 2 and section[1] != 8 and section[5]:
            require(any(vaddr <= section[3] and
                        section[3] + section[5] <= vaddr + filesz and
                        offset + section[3] - vaddr == section[4]
                        for offset, vaddr, filesz in file_mappings),
                    "allocated section is not backed by matching PT_LOAD bytes: " + name)
    require(entry_mapped, "Thumb entry is not backed by an executable PT_LOAD")
    require(data_mapped, "data copy source does not match a PT_LOAD load address")
    require(all(FLASH_START + n in used for n in range(PAGE_SIZE)),
            "boot2 bytes are not completely represented in PT_LOAD")
    vector_org = symbols["__vector_org"]
    require(all(vector_org + n in used for n in range(8)),
            "flash reset vector is not represented in PT_LOAD")
    vector_bytes = bytes(pages[(vector_org + n) & ~(PAGE_SIZE - 1)]
                         [(vector_org + n) % PAGE_SIZE] for n in range(8))
    initial_sp, reset_handler = struct.unpack("<II", vector_bytes)
    require(initial_sp == RAM_END and reset_handler == entry,
            "flash vector initial SP or reset entry disagrees with selected layout")
    # Pinned elf2uf2 pads touched sectors before the final represented page.
    last_page = max(pages)
    sectors = {address // SECTOR_SIZE for address in pages}
    for sector in sectors:
        for address in range(sector * SECTOR_SIZE, (sector + 1) * SECTOR_SIZE, PAGE_SIZE):
            if address < last_page:
                pages.setdefault(address, bytearray(PAGE_SIZE))
    return {"entry": entry, "symbols": symbols, "segments": segments,
            "heap_start_aligned": heap_start,
            "heap_available_bytes": HEAP_LIMIT - heap_start,
            "elf_load_bytes": len(used)}, {address: bytes(page) for address, page in pages.items()}


def parse_uf2(blob):
    require(len(blob) > 0 and len(blob) % 512 == 0, "UF2 length must be nonzero whole blocks")
    count = len(blob) // 512
    require(count <= (FLASH_END - FLASH_START) // PAGE_SIZE, "UF2 has too many flash pages")
    pages = {}
    indices = set()
    addresses_by_index = {}
    for offset in range(0, len(blob), 512):
        magic0, magic1, flags, address, size, index, total, family = struct.unpack_from("<8I", blob, offset)
        require((magic0, magic1, struct.unpack_from("<I", blob, offset + 508)[0]) ==
                (0x0A324655, 0x9E5D5157, 0x0AB16F30), "invalid UF2 magic")
        require(flags == 0x2000 and family == 0xE48BFF56,
                "require plain RP2040 family-ID UF2 blocks")
        require(size == PAGE_SIZE and address % PAGE_SIZE == 0 and
                in_range(address, size, FLASH_START, FLASH_END),
                "UF2 payload must be one aligned page in configured flash")
        require(total == count and index < count and index not in indices,
                "UF2 block indices/count are incomplete, duplicate, or inconsistent")
        require(address not in pages, "UF2 flash addresses overlap")
        require(not any(blob[offset + 32 + PAGE_SIZE:offset + 508]),
                "UF2 unused payload bytes must be zero for the pinned converter")
        indices.add(index)
        addresses_by_index[index] = address
        pages[address] = blob[offset + 32:offset + 32 + PAGE_SIZE]
    require([addresses_by_index[i] for i in range(count)] == sorted(pages),
            "UF2 block numbers must correspond to ascending flash addresses")
    return pages


def check_map(text, symbols):
    for name in SYMBOL_NAMES:
        values = re.findall(r"^\s*(0x[0-9a-fA-F]+)\s+" + re.escape(name) +
                            r"\s*=", text, flags=re.MULTILINE)
        require(values and all(int(value, 16) == symbols[name] for value in values),
                "map/linker-symbol disagreement or missing definition: " + name)


def inspect(elf_blob, uf2_blob, map_text):
    details, expected = parse_elf(elf_blob)
    actual = parse_uf2(uf2_blob)
    require(actual.keys() == expected.keys(), "UF2 page addresses do not match ELF plus converter padding")
    for address in expected:
        require(actual[address] == expected[address],
                "UF2 bytes disagree with ELF or zero padding at 0x%08x" % address)
    check_map(map_text, details["symbols"])
    details.update({"uf2_blocks": len(actual), "uf2_flash_payload_bytes": len(actual) * PAGE_SIZE,
                    "flash_last_programmed_exclusive": max(actual) + PAGE_SIZE,
                    "flash_span_bytes": max(actual) + PAGE_SIZE - FLASH_START,
                    "minimum_heap_bytes": MINIMUM_HEAP, "heap_limit": HEAP_LIMIT,
                    "reset_stack_reserved_bytes": RAM_END - HEAP_LIMIT})
    return details


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("elf", "uf2", "map", "output"):
        parser.add_argument("--" + name, required=True, type=Path)
    args = parser.parse_args()
    report = {"schema_version": 1, "gate": "rp2040-image-structure-2026-09-11",
              "status": "failed", "artifacts": {},
              "scope": "Static structure and ELF/UF2/map consistency only; no execution or commissioning proof."}
    try:
        blobs = {}
        for name in ("elf", "uf2", "map"):
            path = getattr(args, name)
            require(path.resolve() != args.output.resolve(), "report must not overwrite an input artifact")
            blobs[name] = path.read_bytes()
            report["artifacts"][name] = {"path": str(path), "bytes": len(blobs[name]),
                                          "sha256": hashlib.sha256(blobs[name]).hexdigest()}
        require(blobs["map"], "linker map is empty")
        report["image"] = inspect(blobs["elf"], blobs["uf2"], blobs["map"].decode("utf-8"))
        report["status"] = "passed"
    except (GateError, OSError, UnicodeError, struct.error) as exc:
        report["error"] = str(exc)
    # An output/input collision must never truncate the input, including on failure.
    if any(getattr(args, name).resolve() == args.output.resolve() for name in ("elf", "uf2", "map")):
        print(json.dumps(report, indent=2), file=sys.stderr)
        return 1
    try:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    except OSError as exc:
        print("Cannot write image gate report: " + str(exc), file=sys.stderr)
        return 1
    print("Image structure gate: " + report["status"])
    if report["status"] != "passed":
        print(report.get("error", "unknown failure"), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
