
# findCrow 2.0 - UWB Indoor Tracking System

Real-time, coordinate-level indoor positioning using Ultra-Wideband (UWB) ranging, trilateration, and a live Python visualizer with Kalman filtering.

> **Problem:** BLE-based indoor tracking is limited to room-level detection due to RSSI variance from multipath fading and signal reflections. 

> **Solution:** findCrow 2.0 replaces BLE with UWB to achieve accurate localisation.

---

## Demo

<img width="422" height="349" alt="image" src="https://github.com/user-attachments/assets/83564dd4-14fa-41bf-987e-6a6c88198e8a" />


> *Tag moving across a living area tracked in real time across 3 anchors*


https://github.com/user-attachments/assets/5fbf6bab-57e2-43f2-ab98-8742bac2eb4d


---

## How It Works

```
[Anchor 1] ──┐
[Anchor 2] ──┼──(UWB ToF ranging)──► [Tag (ESP32)] ──(WiFi/UDP)──► [Python Visualizer]
[Anchor 3] ──┘
```

1. **Anchors** continuously range against the tag using UWB Time-of-Flight (TWR protocol via DW1000).
2. **Tag** receives distances from all 3 anchors, applies EMA smoothing + outlier rejection, and streams results over UDP.
3. **Visualizer** runs trilateration, applies a median pre-filter and 2D Kalman filter, and plots the live position with a trail.

---

## Hardware

| Component | Board | Role |
|---|---|---|
| Tag | Makerfabs ESP32 UWB Pro (with display) | Moving object being tracked |
| Anchor 1–3 | Makerfabs ESP32 UWB High Power (120 m) | Fixed reference nodes |

---

## Repository Structure

```
findcrow2/
├── anchor/
│   └── anchor.ino          # Flash to each of the 3 anchor boards (change ANCHOR_ID)
├── tag/
│   └── tag.ino             # Flash to the tag board
├── visualizer/
│   └── visualizer.py       # Python live position visualizer
└── README.md
```

---

## Anchor Placement Tips

- Place anchors at known positions forming a large triangle
- Avoid placing anchors collinear (all on one wall).
- Keep anchors at the same height as the tag if possible, or account for height differences.

---

## Tech Stack

`ESP32` · `UWB (DW1000)` · `Embedded C/C++` · `Python` · `NumPy` · `Matplotlib` · `Kalman Filter` · `UDP` · `WiFi`

---
