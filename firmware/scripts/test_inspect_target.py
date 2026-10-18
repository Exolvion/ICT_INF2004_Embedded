#!/usr/bin/env python3
"""Synthetic format fixtures only: these are not ARM compiler/runtime tests."""
import contextlib
import io
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

import inspect_target as gate


F = gate.FLASH_START
R = gate.RAM_START


def fixture(bss_end=gate.HEAP_LIMIT - gate.MINIMUM_HEAP, data_org=F + 0x3000):
    """Build an ELF-shaped byte fixture without invoking any compiler/linker."""
    entry = F + 0x1C1
    symbols = dict(zip(gate.SYMBOL_NAMES,
                       (F, F + 256, F + 256, R, R + 192, data_org,
                        R + 192, R + 208, R + 208, bss_end, bss_end)))
    flash = bytes((n * 7) % 256 for n in range(256)) + bytearray(256)
    flash = bytearray(flash)
    struct.pack_into("<II", flash, 256, gate.RAM_END, entry)
    flash[448:452] = b"\x00\xbf\x00\xbf"
    data = bytes(range(16))
    blob = bytearray(0x600)
    blob[:16] = b"\x7fELF\x01\x01\x01" + bytes(9)
    blob[0x200:0x400] = flash
    blob[0x500:0x510] = data
    strings = bytearray(b"\0")
    symbytes = bytearray(16)
    for name, value in symbols.items():
        index = len(strings)
        strings += name.encode() + b"\0"
        symbytes += struct.pack("<IIIBBH", index, value, 0, 0x10, 0, 0xFFF1)
    string_offset = len(blob)
    blob += strings
    blob += bytes((-len(blob)) % 4)
    symbol_offset = len(blob)
    blob += symbytes
    section_names = bytearray(b"\0")
    name_offsets = {}
    for name in ("boot2", ".text", "vector", ".data", ".bss", ".symtab", ".strtab", ".shstrtab"):
        name_offsets[name] = len(section_names)
        section_names += name.encode() + b"\0"
    names_offset = len(blob)
    blob += section_names
    blob += bytes((-len(blob)) % 4)
    section_offset = len(blob)
    sections = [(0,) * 10]
    def section(name, kind, flags, address, offset, size, link=0, info=0, align=4, entsize=0):
        sections.append((name_offsets[name], kind, flags, address, offset, size, link, info, align, entsize))
    section("boot2", 1, 2, F, 0x200, 256)
    section(".text", 1, 6, F + 256, 0x300, 256)
    section("vector", 8, 3, R, 0, 192)
    section(".data", 1, 3, R + 192, 0x500, 16)
    section(".bss", 8, 3, R + 208, 0, bss_end - R - 208)
    section(".symtab", 2, 0, 0, symbol_offset, len(symbytes), link=7, info=1, entsize=16)
    section(".strtab", 3, 0, 0, string_offset, len(strings), align=1)
    section(".shstrtab", 3, 0, 0, names_offset, len(section_names), align=1)
    for values in sections:
        blob += struct.pack("<10I", *values)
    struct.pack_into("<HHIIIIIHHHHHH", blob, 16, 2, 40, 1, entry, 52, section_offset,
                     0x05000200, 52, 32, 4, 40, len(sections), 8)
    segments = ((1, 0x200, F, F, 512, 512, 5, 4),
                (1, 0x500, R + 192, data_org, 16, 16, 6, 4),
                (1, 0, R, R, 0, 192, 6, 4),
                (1, 0, R + 208, R + 208, 0, bss_end - R - 208, 6, 4))
    for i, values in enumerate(segments):
        struct.pack_into("<8I", blob, 52 + 32 * i, *values)
    # This deliberately sparse fixture requires 14 zero pages in sector zero,
    # then one populated page at +0x3000. Other untouched sectors remain absent.
    pages = {F: bytes(flash[:256]), F + 256: bytes(flash[256:])}
    for address in range(F + 512, F + 4096, 256):
        pages[address] = bytes(256)
    final_page = data_org & ~255
    for address in range(final_page & ~4095, final_page, 256):
        pages.setdefault(address, bytes(256))
    final_payload = bytearray(256)
    final_payload[data_org % 256:data_org % 256 + 16] = data
    pages[final_page] = bytes(final_payload)
    uf2 = bytearray()
    for index, (address, payload) in enumerate(sorted(pages.items())):
        uf2 += struct.pack("<8I", 0x0A324655, 0x9E5D5157, 0x2000, address, 256,
                           index, len(pages), 0xE48BFF56)
        uf2 += payload + bytes(220) + struct.pack("<I", 0x0AB16F30)
    map_text = "\n".join("  0x%016x %s = ." % (value, name) for name, value in symbols.items()) + "\n"
    return bytes(blob), bytes(uf2), map_text


def with_integer(blob, offset, value, format_string="<I"):
    changed = bytearray(blob)
    struct.pack_into(format_string, changed, offset, value)
    return bytes(changed)


class SyntheticImageGateTests(unittest.TestCase):
    def setUp(self):
        self.elf, self.uf2, self.map = fixture()

    def reject_elf(self, changed, message=None):
        with self.assertRaises(gate.GateError) as caught:
            gate.inspect(changed, self.uf2, self.map)
        if message:
            self.assertIn(message, str(caught.exception))

    def reject_uf2(self, changed, message=None):
        with self.assertRaises(gate.GateError) as caught:
            gate.inspect(self.elf, changed, self.map)
        if message:
            self.assertIn(message, str(caught.exception))

    def test_valid_exact_heap_boundary_and_sparse_padding(self):
        result = gate.inspect(self.elf, self.uf2, self.map)
        self.assertEqual(result["heap_available_bytes"], 65536)
        self.assertEqual(result["uf2_blocks"], 17)
        self.assertEqual(result["elf_load_bytes"], 528)
        # Kernel heap initialization rounds its start up to an 8-byte boundary.
        # A word-aligned end four bytes below the boundary must not overreport.
        result = gate.inspect(*fixture(bss_end=gate.HEAP_LIMIT - gate.MINIMUM_HEAP - 4))
        self.assertEqual(result["heap_start_aligned"], gate.HEAP_LIMIT - gate.MINIMUM_HEAP)
        self.assertEqual(result["heap_available_bytes"], 65536)

    def test_valid_larger_heap(self):
        self.assertEqual(gate.inspect(*fixture(bss_end=R + 65536))["heap_available_bytes"], 194560)
        result = gate.inspect(*fixture(bss_end=R + 65540))
        self.assertEqual(result["heap_start_aligned"], R + 65544)
        self.assertEqual(result["heap_available_bytes"], 194552)

    def test_valid_last_flash_word_boundary(self):
        self.assertEqual(gate.inspect(*fixture(data_org=gate.FLASH_END - 16))
                         ["flash_last_programmed_exclusive"], gate.FLASH_END)

    def test_heap_short_by_one_word(self):
        with self.assertRaisesRegex(gate.GateError, "minimum 65536"):
            gate.inspect(*fixture(bss_end=gate.HEAP_LIMIT - gate.MINIMUM_HEAP + 4))

    def test_bad_elf_class_endian_magic(self):
        for offset in (0, 4, 5, 6):
            with self.subTest(offset=offset):
                data = bytearray(self.elf)
                data[offset] ^= 0x03
                self.reject_elf(bytes(data))

    def test_wrong_machine_or_kind(self):
        for offset, value in ((16, 1), (18, 62)):
            with self.subTest(offset=offset):
                self.reject_elf(with_integer(self.elf, offset, value, "<H"))

    def test_hard_float_and_wrong_eabi(self):
        for flags in (0x05000400, 0x04000200):
            with self.subTest(flags=flags):
                self.reject_elf(with_integer(self.elf, 36, flags))

    def test_non_thumb_entry(self):
        self.reject_elf(with_integer(self.elf, 24, F + 0x1C0), "Thumb")

    def test_truncated_header_tables_and_payload(self):
        for length in (0, 16, 51, 100, len(self.elf) - 1):
            with self.subTest(length=length):
                self.reject_elf(self.elf[:length])

    def test_program_headers_outside_file(self):
        self.reject_elf(with_integer(self.elf, 28, 0xFFFFFFFC), "outside")

    def test_missing_symbol_table(self):
        shoff = struct.unpack_from("<I", self.elf, 32)[0]
        self.reject_elf(with_integer(self.elf, shoff + 6 * 40 + 4, 1), "missing linker symbols")

    def test_symbol_table_invalid_link(self):
        shoff = struct.unpack_from("<I", self.elf, 32)[0]
        self.reject_elf(with_integer(self.elf, shoff + 6 * 40 + 24, 99), "string table")

    def test_section_symbol_disagreement(self):
        shoff = struct.unpack_from("<I", self.elf, 32)[0]
        self.reject_elf(with_integer(self.elf, shoff + 1 * 40 + 20, 252), "section/linker-symbol")

    def test_section_file_offset_disagrees_with_load(self):
        shoff = struct.unpack_from("<I", self.elf, 32)[0]
        self.reject_elf(with_integer(self.elf, shoff + 1 * 40 + 16, 0x204), "matching PT_LOAD bytes")

    def test_unaligned_data_copy_source(self):
        with self.assertRaisesRegex(gate.GateError, "word aligned"):
            gate.inspect(*fixture(data_org=F + 0x3001))

    def test_flash_data_exceeds_two_mib(self):
        with self.assertRaisesRegex(gate.GateError, "outside flash"):
            gate.inspect(*fixture(data_org=gate.FLASH_END - 12))

    def test_section_into_heap(self):
        shoff = struct.unpack_from("<I", self.elf, 32)[0]
        changed = with_integer(self.elf, shoff + 2 * 40 + 12, gate.HEAP_LIMIT - 128)
        self.reject_elf(changed, "outside configured")

    def test_file_larger_than_segment_memory(self):
        self.reject_elf(with_integer(self.elf, 52 + 16, 513), "filesz")

    def test_load_payload_outside_file(self):
        self.reject_elf(with_integer(self.elf, 52 + 4, len(self.elf) + 1024), "outside")

    def test_load_payload_outside_flash(self):
        self.reject_elf(with_integer(self.elf, 52 + 12, gate.FLASH_END - 256), "physical range")

    def test_virtual_range_outside_ram(self):
        self.reject_elf(with_integer(self.elf, 52 + 32 + 8, gate.HEAP_LIMIT), "virtual range")

    def test_overlapping_load_bytes(self):
        self.reject_elf(with_integer(self.elf, 52 + 32 + 12, F + 128), "overlapping")

    def test_wrong_data_load_address(self):
        self.reject_elf(with_integer(self.elf, 52 + 32 + 12, F + 0x4000), "data copy source")

    def test_entry_not_executable(self):
        self.reject_elf(with_integer(self.elf, 52 + 24, 4), "executable PT_LOAD")

    def test_wrong_vector_sp_or_reset(self):
        for offset in (0x300, 0x304):
            with self.subTest(offset=offset):
                self.reject_elf(with_integer(self.elf, offset, 0), "initial SP or reset")

    def test_bad_uf2_length(self):
        for data in (b"", self.uf2[:-1], self.uf2 + b"x"):
            with self.subTest(length=len(data)):
                self.reject_uf2(data, "whole blocks")

    def test_bad_uf2_magic(self):
        for offset in (0, 4, 508):
            with self.subTest(offset=offset):
                self.reject_uf2(with_integer(self.uf2, offset, 0), "magic")

    def test_wrong_family_or_flags(self):
        for offset, value in ((28, 0), (8, 0), (8, 0x2001)):
            with self.subTest(offset=offset, value=value):
                self.reject_uf2(with_integer(self.uf2, offset, value), "family-ID")

    def test_unaligned_outside_or_wrong_payload_size(self):
        for offset, value in ((12, F + 1), (12, gate.FLASH_END), (16, 128)):
            with self.subTest(offset=offset, value=value):
                self.reject_uf2(with_integer(self.uf2, offset, value), "aligned page")

    def test_duplicate_or_missing_block_indices(self):
        for offset, value in ((512 + 20, 0), (20, 17), (24, 18)):
            with self.subTest(offset=offset):
                self.reject_uf2(with_integer(self.uf2, offset, value), "indices/count")

    def test_overlapping_flash_pages(self):
        self.reject_uf2(with_integer(self.uf2, 512 + 12, F), "overlap")

    def test_block_number_address_order(self):
        changed = with_integer(self.uf2, 20, 1)
        changed = with_integer(changed, 512 + 20, 0)
        self.reject_uf2(changed, "ascending")

    def test_out_of_file_order_complete_blocks_are_valid(self):
        changed = self.uf2[512:1024] + self.uf2[:512] + self.uf2[1024:]
        self.assertEqual(gate.inspect(self.elf, changed, self.map)["uf2_blocks"], 17)

    def test_uf2_missing_padding_page(self):
        blocks = [self.uf2[i:i + 512] for i in range(0, len(self.uf2), 512)]
        del blocks[5]
        changed = b"".join(with_integer(with_integer(block, 20, i), 24, len(blocks))
                           for i, block in enumerate(blocks))
        self.reject_uf2(changed, "page addresses")

    def test_uf2_modified_load_or_padding_byte(self):
        for offset in (32, 2 * 512 + 32):
            with self.subTest(offset=offset):
                changed = bytearray(self.uf2)
                changed[offset] ^= 1
                self.reject_uf2(bytes(changed), "bytes disagree")

    def test_nonzero_unused_block_bytes(self):
        self.reject_uf2(with_integer(self.uf2, 288, 1), "unused payload")

    def test_mismatched_or_missing_map(self):
        for text in ("", self.map.replace("__bss_end", "__missing"),
                     self.map.replace("0x000000002002f800", "0x000000002002f804")):
            with self.subTest(text_length=len(text)):
                with self.assertRaisesRegex(gate.GateError, "map/linker-symbol"):
                    gate.inspect(self.elf, self.uf2, text)

    def test_cli_records_success_and_hashes(self):
        with tempfile.TemporaryDirectory() as folder:
            paths = [Path(folder) / name for name in ("image.elf", "image.uf2", "image.map", "report.json")]
            for path, contents in zip(paths, (self.elf, self.uf2, self.map.encode())):
                path.write_bytes(contents)
            argv = ["inspect_target.py", "--elf", str(paths[0]), "--uf2", str(paths[1]),
                    "--map", str(paths[2]), "--output", str(paths[3])]
            with patch("sys.argv", argv), contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(gate.main(), 0)
            report = json.loads(paths[3].read_text())
            self.assertEqual(report["status"], "passed")
            self.assertEqual(len(report["artifacts"]["elf"]["sha256"]), 64)
            self.assertIn("no execution", report["scope"])

    def test_cli_failure_and_input_collision_do_not_truncate_input(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "image.elf"
            path.write_bytes(self.elf)
            output = Path(folder) / "report.json"
            argv = ["inspect_target.py", "--elf", str(path), "--uf2", str(path),
                    "--map", str(path), "--output", str(output)]
            with patch("sys.argv", argv), contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(gate.main(), 1)
            self.assertEqual(json.loads(output.read_text())["status"], "failed")
            argv[-1] = str(path)
            with patch("sys.argv", argv), contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(gate.main(), 1)
            self.assertEqual(path.read_bytes(), self.elf)


if __name__ == "__main__":
    unittest.main(verbosity=2)
