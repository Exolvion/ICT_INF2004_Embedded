#!/usr/bin/env bash
# Replay every cut file through a sim (MISSION stream replay) and print one
# line per cut: file, confirmed codes, unconfirmed codes, junk group count.
# Usage: evalcuts.sh <sim> <dir> [<dir> ...]; traces via "stream" too.
set -u
sim=$1
shift
for dir in "$@"; do
    for f in "$dir"/*.txt; do
        out=$("$sim" dump "$f" 2>&1)
        codes=$(grep -a -o 'CODE \*[^ ]*\*[^,(]*' <<<"$out" | sed 's/CODE //; s/ read in order//; s/ read end to start/</' | tr '\n' ' ')
        single=$(grep -a -o 'one swing only, not a code: [^ ]*' <<<"$out" | sed 's/.*: //' | tr '\n' ' ')
        groups=$(grep -a -c 'GROUP ' <<<"$out")
        printf '%s\tcodes: %s\tsingle: %s\tgroups %s\n' "$(basename "$f")" "${codes:--}" "${single:--}" "$groups"
    done
done
