#!/usr/bin/env bash
# runsim.sh <dir>: current car decoder (sim dump, MISSION stream) on every cut
cd "$1"
for f in syn_*.txt; do
  out=$(../simcur/sim dump "$f" 2>&1)
  codes=$(grep -a -o 'CODE \*[^ ]*\*[^,(]*' <<<"$out" | sed 's/CODE //; s/ read in order//; s/ read end to start/</' | tr '\n' ' ')
  single=$(grep -a -o 'one swing only, not a code: [^ ]*' <<<"$out" | sed 's/.*: //' | tr '\n' ' ')
  bars=$(grep -a -o 'GROUP [0-9]* bars' <<<"$out" | awk '{print $2}' | tr '\n' ',')
  printf '%s\t%s\t%s\t%s\n' "$f" "${codes:--}" "${single:--}" "${bars:--}"
done > sim.tsv
echo "$1 done"
