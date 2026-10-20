| Message | Payload before (B) | Payload after (B) | MQTT PUBLISH before (B) | MQTT PUBLISH after (B) |
|---|---:|---:|---:|---:|
| status frame (worst) | 237 | 17 | 275 | 54 |
| status frame (typical) | 145 | 17 | 183 | 54 |
| perception frame (worst) | 252 | 16 | 290 | 53 |
| perception frame (typical) | 142 | 16 | 180 | 53 |
| diagnostic frame (worst) | 144 | 17 | 182 | 54 |
| diagnostic frame (typical) | 78 | 17 | 115 | 54 |
| command (largest token / any command) | 8 | 1 | 43 | 36 |
| STOP command | 4 | 1 | 39 | 36 |
| presence 'online' (status topic) | 6 | 1 | 40 | 35 |
| presence 'offline' (status topic) | 7 | 1 | 41 | 35 |
