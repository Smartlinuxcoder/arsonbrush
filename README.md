# Arsonbrush

Arsonbrush is a custom-made smart electric toothbrush, reusing a DC motor from a dead Oral-B toothbrush. 

## Why did I make this?

I was bored of $100+ dumb electric toothbrushes that have under-specced components to make them die faster.
For a bit of context, two well taken-care-of Oral-B toothbrushes stopped working, both with "exploded" resistors.
And also, ironically, whilst making this, ANOTHER ONE died on me.

I also noticed that there wasn't any open-source/DIY option.
So, may this toothbrush be the stepping stone for open-source electric toothbrushes in the future.

---
## Features!

- **Waterproof Charging**: Pogo pins!
- **Smart Base Station**: powered by ESP32C3
- **Huge Battery**: 5750mAh 21700 Li-Ion battery, do I need to say more?

(2+ years of runtime on a single charge, btw)

## Hardware Components

### Toothbrush Mainboard
- **MCU**: STM32C091FC
- **Motor Controller**: DRV8833RTYR dual H-bridge
- **IMU**: LSM6DSRTR 6-axis accelerometer/gyroscope
- **Charging IC**: MCP73831T-2ACI/OT (500mA charge rate)
- **Voltage Regulator**: MCP1700T-3302E/TT
- **Battery**: 21700 Li-Ion (5750mAh, BAK N21700CH-58E)
- **EMI Shield**: grounded 0.8mm copper sheet between motor and pcb

### Base Station
- **MCU**: Tenstar ESP32C3 SuperMini
- **Display**: Cheap I2C oled screen I have lying around
- **Connector**: 2A waterproof pogo pin connector pair
- **Status LED**: a status led

### 3D Model

![3D Assembly](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MzAxNjQsInB1ciI6ImJsb2JfaWQifX0=--c6d5c4e3b810757d0607bb8597c67744d3b9931d/image.png)

![Case Front](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MzAxNjUsInB1ciI6ImJsb2JfaWQifX0=--786a1fdb69e971fbe279193700e60346bdf3a805/Screenshot%20From%202025-12-26%2021-59-03.png)

![Case Back](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MzAxNjYsInB1ciI6ImJsb2JfaWQifX0=--bb72df95aa57114fcad1afd3d63579b8b216cc2b/Screenshot%20From%202025-12-26%2021-58-58.png)

## Toothbrush PCB!
![Toothbrush Schematic](pcb/schematic-toothbrush.png)
![alt text](images/image-5.png)
## Base station PCB!
![Base Station Schematic](https://blueprint.hackclub.com/user-attachments/blobs/proxy/eyJfcmFpbHMiOnsiZGF0YSI6MzAxNjksInB1ciI6ImJsb2JfaWQifX0=--c5c75e834cf81e9b0771032d0e5aa0b94c620854/image.png)

![Base Station PCB](images/image-4.png)



## Bill of Materials
### This is in addition to [the toothbrush PCBA BOM](pcb/bom-toothbrush.csv)

| Component | Quantity | Description | Supplier | Unit Cost | Total Cost (+shipping) | Image |
|-----------|----------|-------------|----------|-----------|------------------------|-------|
| Toothbrush PCB | 5, 2 PCBA |toothbrush handle mainboard | JLCPCB | Not applicable | 70 EUR |![alt text](images/image-5.png) |
| Base board | 5 | Toothbrush base PCB | JLCPCB | 1 EUR | 5 EUR | ![alt text](images/image-4.png)|
| 2A Pogo pins connector pair | 1 pair | Waterproof connection | [Aliexpress](https://it.aliexpress.com/item/1005006525401310.html) | 5.89 EUR | 5.89 EUR |![alt text](images/image.png) |
| Tenstar ESP32C3 supermini | 1 | Base MCU | [Aliexpress](https://it.aliexpress.com/item/1005009897797706.html) | 3.36 EUR | 3.36 EUR |![alt text](images/image-1.png) |
| Copper sheet 0.8mm 50x50cm | 1 | Motor EMI protection plane | [Aliexpress](https://it.aliexpress.com/item/1005006915598911.html) | 3.69 EUR | 3.69 EUR | ![alt text](images/image-2.png)|
| 21700 battery | 1 | Toothbrush battery | [nkon.nl](https://www.nkon.nl/en/bak-n21700ch-58e-5750mah-11-2a.html) | 3.75 EUR | 9.14 EUR |![alt text](images/image-3.png) |

**Total Project Cost:** 97.08 EUR

**Notes:**
- All costs are in EUR
- Quantities reflect MOQ

## File tree
```
arsonbrush/
├── 3d/                    # 3D models and case designs
├── code/arsonbrush/       # Firmware
│   ├── Core/Src           # Firmware source code
├── pcb/                   # EasyEDA PCB
│   ├── arsonBrush.eprj    # Complete project
│   ├── bom-toothbrush.csv # toothbrush PCBA BOM
│   ├── gerber-base.zip    # Base station gerbers
│   ├── gerber-toothbrush.zip # Toothbrush gerbers
│   ├── schematic-base.pdf # Base station schematic
│   └── schematic-toothbrush.pdf # Toothbrush schematic
├── datasheets/            # Component datasheets
└── README.md              # This file!
```

## Building the Toothbrush firmware

### Prerequisites
- Nix pakage manager
- ST-Link or compatible programmer

### Compilation
```bash
cd code/arsonbrush
nix-shell
make
```

### Flashing
Flash the generated `.bin/.hex` from `/build`


## License

This project is licensed under the [GNU General Public License v3.0](LICENSE).

## Acknowledgments

- Project developed as part of Hack Club Blueprint
- Awesome thanks to DragonSlayer and Adam Turaj for helping me make this!
