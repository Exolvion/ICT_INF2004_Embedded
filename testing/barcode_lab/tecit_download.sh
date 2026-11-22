#!/usr/bin/env bash
# Download tec-it's Code 39 image for every letter (the free link gives a
# 94 x 100 GIF, 2 px per narrow element) into a new folder, 1 s apart.
# Then: python -I tecit_parse.py tecit_dl tecit_bars.json
set -eu
mkdir -p tecit_dl
for c in A B C D E F G H I J K L M N O P Q R S T U V W X Y Z; do
  curl -s -m 30 -o "tecit_dl/$c.gif" "https://barcode.tec-it.com/barcode.ashx?data=$c&code=Code39&translate-esc=on"
  sleep 1
done
ls tecit_dl | wc -l
