# Mouse-Micro — Custom Ergonomic Gaming Mouse

A fully custom gaming mouse built from scratch — from clay hand modeling and photogrammetry all the way to a custom KiCad PCB. Designed to prevent repetitive strain injuries during extended use while delivering high-precision tracking.

## Features

- **Ergonomic form** designed from a clay hand model, digitized via photogrammetry (Meshroom), refined in Fusion360 using surface modeling
- **Custom PCB** designed in KiCad with:
  - **Arduino Pro Micro** microcontroller
  - **PMW3389** high-precision optical sensor (SPI communication)
  - Left click, right click, forward, back, scroll wheel (rotation + press), adjustable DPI
- **USB-Micro** connection
- Uses the **AdvMouse** library for low-latency HID reporting
- Compatible with any device — no drivers required
- Integrated mounting for buttons, PCB, and sensor alignment

## Repository Structure

```
Mouse-Micro/
├── CAD/                     # Fusion360 mouse body surface model
├── Code/PMW3389DM_Mouse/    # Arduino firmware (PMW3389 + AdvMouse)
├── D2F-01/                  # Omron switch footprints
├── PMW3360/                 # PMW3360 sensor footprints (alternate)
├── PMW3389/                 # PMW3389 sensor footprints
├── ProMicroKiCad-master/   # Pro Micro KiCad library
├── ul_EC10E1220501/         # Scroll encoder footprint
├── Mouse-backups/          # KiCad schematic backups
├── Output/                  # Gerber and production output files
├── Mouse.kicad_pro          # KiCad project file
└── Mouse-Shaft.SLDPRT      # SolidWorks shaft model
```

## Tools Used

- **Meshroom** — Photogrammetry mesh generation from clay model
- **Fusion360** — Surface modeling and CAD refinement
- **KiCad** — Schematic capture and PCB layout
- **Arduino IDE** — Firmware development
- **SolidWorks** — Mechanical shaft design

## Key Components

| Component | Function |
|-----------|----------|
| Arduino Pro Micro | USB HID microcontroller |
| PMW3389 | High-precision optical sensor (up to 16000 DPI) |
| Omron D2F-01 | Tactile click switches |
| EC10E1220501 | Scroll wheel rotary encoder |

## Author

**Jackson Barber** — [github.com/jacksterb1234](https://github.com/jacksterb1234)
