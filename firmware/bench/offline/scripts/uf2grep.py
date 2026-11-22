"""Count a string in a UF2 image's joined payload (UF2 splits data into 256-byte blocks)."""
import sys

data = open(sys.argv[1], 'rb').read()
payload = b''.join(data[i + 32:i + 32 + 256] for i in range(0, len(data), 512))
for text in sys.argv[2:]:
    print(text, payload.count(text.encode()))
