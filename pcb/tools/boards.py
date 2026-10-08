"""What is on each board: parts, nets, where they sit and how they are routed.

Board coordinates are in mm from the board's top-left corner, x to the right
and y down (KiCad's convention). A route is a list of points: ("U1", "11")
means pad 11 of U1, a tuple of numbers is a corner. All tracks are on the
top layer; the bottom layer is a solid ground plane.
"""

DATE = "2026-10-08"
REPO = "github.com/CoraBeanz/linescan-hyperspectral-splatting"

# JLCPCB / LCSC parts, checked on lcsc.com (October 2026)
R10K = [("LCSC", "C17414"), ("MPN", "UNI-ROYAL 0805W8F1002T5E"), ("Assembly", "JLC")]
R1K = [("LCSC", "C17513"), ("MPN", "UNI-ROYAL 0805W8F1001T5E"), ("Assembly", "JLC")]
R220 = [("LCSC", "C17557"), ("MPN", "UNI-ROYAL 0805W8F2200T5E"), ("Assembly", "JLC")]
R4K7 = [("LCSC", "C17673"), ("MPN", "UNI-ROYAL 0805W8F4701T5E"), ("Assembly", "JLC")]
C100N = [("LCSC", "C49678"), ("MPN", "YAGEO CC0805KRX7R9BB104"), ("Assembly", "JLC")]
C1N = [("LCSC", "C46653"), ("MPN", "Samsung CL21B102KBCNNNC"), ("Assembly", "JLC")]
SS34 = [("LCSC", "C8678"), ("MPN", "MDD SS34"), ("Assembly", "JLC")]
SMAJ15A = [("LCSC", "C113958"), ("MPN", "MDD SMAJ15A"), ("Assembly", "JLC")]
LED_G = [("LCSC", "C2297"), ("MPN", "Hubei KENTO KT-0805G"), ("Assembly", "JLC")]
LED_R = [("LCSC", "C84256"), ("MPN", "NATIONSTAR NCD0805R1"), ("Assembly", "JLC")]
PTC = [("LCSC", "C492015"), ("MPN", "PTTC SMD1812P050TF/30"), ("Assembly", "JLC")]
XH4 = [("LCSC", "C144395"), ("MPN", "JST B4B-XH-A(LF)(SN)"), ("Assembly", "hand")]
XH3 = [("LCSC", "C144394"), ("MPN", "JST B3B-XH-A(LF)(SN)"), ("Assembly", "hand")]


def hand(mpn):
    return [("LCSC", ""), ("MPN", mpn), ("Assembly", "hand")]


# ------------------------------------------------------------------------------
# Scan-mirror controller
# ------------------------------------------------------------------------------
# U1 pads: 1-19 are the DevKitC's header J2 (3V3 ... 5V), 20-38 its header J3
# (GND ... CLK). Pins from firmware/include/board_pins.h.
CONTROLLER_PARTS = [
    # ref, symbol, value, footprint, {pin: net}, fields, schematic (x, y), extra
    ("U1", "ESP32-DevKitC", "ESP32-DevKitC (38-pin)", "rig:ESP32-DevKitC_Socket",
     {"1": "+3V3", "8": "HALL", "9": "DIR", "10": "STEP", "11": "TMC_EN", "14": "GND", "19": "+5V",
      "20": "GND", "26": "GND", "27": "TRIG", "28": "MOVING", "30": "UART_TX", "31": "TMC_UART", "34": "STATUS"},
     hand("2x female header 1x19, 2.54 mm (ESP32-DevKitC plugs in)"), (114.3, 165.1), {}),
    ("U2", "BTT_TMC2209", "BTT TMC2209 V1.3", "rig:StepStick_Socket_BTT_TMC2209",
     {"1": "TMC_EN", "2": "GND", "3": "GND", "4": "PDN_UART", "5": "PDN_ALT", "6": "GND", "7": "STEP",
      "8": "DIR", "9": "GND", "10": "+3V3", "11": "COIL_B2", "12": "COIL_B1", "13": "COIL_A1",
      "14": "COIL_A2", "15": "GND", "16": "VM"},
     hand("2x female header 1x8, 2.54 mm (driver plugs in)"), (254.0, 152.4), {}),
    ("JP1", "SolderJumper_3_Bridged12", "UART on pin 4",
     "rig:SolderJumper-3_P1.3mm_Bridged12_RoundedPad1.0x1.5mm_NumberLabels",
     {"1": "PDN_UART", "2": "TMC_UART", "3": "PDN_ALT"}, [], (205.74, 129.54),
     {"in_bom": False, "ref_at": (205.74, 124.46), "val_at": (205.74, 127.0)}),
    ("R1", "R", "10k", "rig:R_0805_2012Metric", {"1": "+3V3", "2": "TMC_EN"}, R10K, (162.56, 210.82), {}),
    ("R2", "R", "10k", "rig:R_0805_2012Metric", {"1": "HALL_IN", "2": "+3V3"}, R10K, (284.48, 220.98), {}),
    ("R3", "R", "1k", "rig:R_0805_2012Metric", {"1": "UART_TX", "2": "TMC_UART"}, R1K, (187.96, 210.82), {}),
    ("R4", "R", "1k", "rig:R_0805_2012Metric", {"1": "HALL_IN", "2": "HALL"}, R1K, (309.88, 220.98), {}),
    ("R5", "R", "220", "rig:R_0805_2012Metric", {"1": "MOVING_OUT", "2": "MOVING"}, R220, (55.88, 248.92), {}),
    ("R6", "R", "220", "rig:R_0805_2012Metric", {"1": "TRIG_OUT", "2": "TRIG"}, R220, (81.28, 248.92), {}),
    ("R7", "R", "4.7k", "rig:R_0805_2012Metric", {"1": "PWR_LED", "2": "VM"}, R4K7, (284.48, 50.8), {}),
    ("R8", "R", "1k", "rig:R_0805_2012Metric", {"1": "STATUS_LED", "2": "STATUS"}, R1K, (106.68, 248.92), {}),
    ("C1", "C_Polarized", "100uF 25V", "rig:CP_Radial_D6.3mm_P2.50mm", {"1": "VM", "2": "GND"},
     hand("100 uF 25 V or 35 V electrolytic, 6.3 mm, 2.5 mm pitch"), (172.72, 50.8), {}),
    ("C2", "C", "100nF", "rig:C_0805_2012Metric", {"1": "VM", "2": "GND"}, C100N, (198.12, 50.8), {}),
    ("C3", "C", "1nF", "rig:C_0805_2012Metric", {"1": "HALL", "2": "GND"}, C1N, (335.28, 220.98), {"dnp": True}),
    ("D1", "D_Schottky", "SS34", "rig:D_SMA", {"1": "VM", "2": "VIN_F"}, SS34, (139.7, 50.8), {}),
    ("D2", "D_TVS_Unidirectional", "SMAJ15A", "rig:D_SMA", {"1": "VM", "2": "GND"}, SMAJ15A, (238.76, 55.88), {}),
    ("D3", "LED", "green", "rig:LED_0805_2012Metric", {"1": "GND", "2": "PWR_LED"}, LED_G, (322.58, 55.88), {}),
    ("D4", "LED", "red", "rig:LED_0805_2012Metric", {"1": "GND", "2": "STATUS_LED"}, LED_R, (132.08, 254.0), {}),
    ("F1", "Polyfuse", "500mA", "rig:Fuse_1812_4532Metric", {"1": "VIN", "2": "VIN_F"}, PTC, (104.14, 50.8), {}),
    ("J1", "Barrel_Jack_Switch", "12V 5.5x2.1", "rig:BarrelJack_Horizontal", {"1": "VIN", "2": "GND", "3": "GND"},
     hand("DC-005 style 5.5 x 2.1 mm jack, 3 pins"), (50.8, 50.8), {}),
    ("J2", "Screw_Terminal_01x02", "12V", "rig:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal",
     {"1": "VIN", "2": "GND"}, hand("2-pin screw terminal, 5.08 mm (KF301-5.08 or Phoenix MKDS 1,5/2-5,08)"),
     (50.8, 81.28), {}),
    ("J3", "Conn_01x04", "NEMA 8", "rig:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical",
     {"1": "COIL_A2", "2": "COIL_A1", "3": "COIL_B1", "4": "COIL_B2"}, XH4, (365.76, 152.4), {}),
    ("J4", "Conn_01x03", "A3144", "rig:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical",
     {"1": "+5V", "2": "GND", "3": "HALL_IN"}, XH3, (365.76, 205.74), {}),
    ("J5", "Conn_01x03", "Jetson", "rig:PinHeader_1x03_P2.54mm_Vertical",
     {"1": "MOVING_OUT", "2": "TRIG_OUT", "3": "GND"}, hand("1x3 pin header, 2.54 mm"), (30.48, 248.92), {}),
]

CONTROLLER_FLAGS = [("GND", 355.6, 43.18), ("VM", 370.84, 43.18)]

CONTROLLER_NOTES = [
    (25.4, 25.4, 2.0, "Scan-mirror controller: ESP32-DevKitC + BTT TMC2209 for the NEMA 8 mirror stepper"),
    (25.4, 30.48, 1.27, "12 V in on J1 or J2 (ALITOVE 12 V 3 A). F1 limits it, D1 blocks reverse polarity, "
                        "D2 clamps spikes under the TMC2209's 29 V limit. D3 is lit while the motor has power."),
    (25.4, 106.68, 1.27, "The ESP32 is powered over USB from the Jetson, which is also its command link, "
                         "so there is no 5 V regulator here."),
    (25.4, 110.49, 1.27, "With USB unplugged the TMC2209 has no logic supply (VDD), so the motor cannot move."),
    (175.26, 99.06, 1.27, "R1 holds EN high (motor off) while the ESP32 boots."),
    (175.26, 102.87, 1.27, "UART: RX (IO16) straight to PDN_UART, TX (IO17) through R3."),
    (175.26, 106.68, 1.27, "JP1 picks the module pin: 1-2 = pin 4 (BTT default), 2-3 = pin 5."),
    (175.26, 110.49, 1.27, "MS1 = MS2 = GND: UART address 0. CLK = GND: internal clock."),
    (175.26, 190.5, 1.27, "BEFORE FIRST POWER-UP: with the motor unplugged, turn VREF to 0.28 V (0.2 A)."),
    (175.26, 194.31, 1.27, "Never plug or unplug the motor with 12 V on."),
    (271.78, 238.76, 1.27, "Hall: A3144 runs on 5 V; its open-collector OUT is pulled up to 3.3 V by R2."),
    (271.78, 242.57, 1.27, "R4 limits current into IO33 if the cable is miswired. "
                           "C3 is fitted only if motor noise shows up."),
    (25.4, 266.7, 1.27, "J5 to the Jetson's 40-pin header (3.3 V GPIO): MOVING, TRIG, GND. "
                        "R5 and R6 protect both ends from a wrong pin."),
    (25.4, 270.51, 1.27, "D4 is the firmware's status LED on IO2 (off: disabled, on: enabled, blinking: driver fault); "
                         "a genuine DevKitC has no LED there. The LED and R8 keep IO2 low at boot, as it must be."),
]

# Board: 95 x 66 mm
CONTROLLER_SIZE = (95.0, 66.0)

CONTROLLER_PLACE = {
    # ref: (x, y, rotation) of the footprint origin (pad 1 for most parts)
    "U1": (39.37, 53.34, 180),   # J2 header on the right, USB at the top edge
    "U2": (46.99, 15.24, 0),     # STEP and DIR line up with IO26 and IO25
    "JP1": (43.18, 24.13, 270),
    "R1": (36.40, 28.852, 90),
    "R2": (49.53, 38.50, 90),
    "R3": (17.00, 26.312, 90),
    "R4": (43.18, 38.50, 90),
    "R5": (8.25, 33.02, 0),
    "R6": (8.25, 35.56, 0),
    "R7": (56.50, 3.60, 0),
    "R8": (8.25, 17.78, 0),
    "C1": (60.50, 8.50, 0),
    "C2": (61.75, 3.60, 0),
    "C3": (45.72, 38.50, 270),
    "D1": (66.50, 14.00, 0),
    "D2": (69.00, 3.60, 0),
    "D3": (52.50, 3.60, 0),
    "D4": (4.20, 17.78, 0),
    "F1": (74.00, 14.00, 180),
    "J1": (81.00, 14.00, 180),   # jack opening at the right edge
    "J2": (89.40, 28.00, 90),    # wire entry at the right edge
    "J3": (66.04, 20.32, 270),
    "J4": (45.72, 61.00, 0),
    "J5": (2.54, 33.02, 0),
}

# reference designators: (x, y, angle), or None to hide
CONTROLLER_REFS = {
    "R1": (34.6, 28.85, 90), "R2": (51.4, 38.5, 90), "R3": (18.9, 26.31, 90), "R4": (41.4, 38.5, 90),
    "C3": (47.6, 38.5, 90), "R5": (8.25, 31.4, 0), "R6": (8.25, 37.2, 0), "R7": (56.5, 1.8, 0),
    "D3": (52.5, 1.8, 0), "C2": (61.75, 1.8, 0), "D2": (69.0, 1.4, 0), "C1": (57.3, 8.5, 90),
    "D1": (66.5, 16.3, 0), "F1": (74.0, 16.6, 0), "J1": (77.5, 19.5, 0), "J2": (82.5, 25.5, 0),
    "J3": (66.04, 32.0, 0), "J4": (41.5, 61.0, 90), "J5": (2.54, 30.8, 0), "JP1": (43.18, 20.6, 0),
    "R8": (8.25, 16.2, 0), "D4": (4.2, 16.2, 0),
}

CONTROLLER_HOLES = [(3.5, 3.5), (91.5, 3.5), (3.5, 62.5), (91.5, 62.5)]

SIG, V3, VM, MOT = 0.25, 0.3, 0.8, 0.5

CONTROLLER_ROUTES = [
    # step, direction, enable
    ("STEP", SIG, [("U1", "10"), ("U2", "7")]),
    ("DIR", SIG, [("U1", "9"), ("U2", "8")]),
    ("TMC_EN", SIG, [("U1", "11"), (40.64, 29.21), (48.5, 29.21), (48.5, 15.24), ("U2", "1")]),
    ("TMC_EN", SIG, [("U1", "11"), ("R1", "2")]),
    # single-wire UART, around the USB end of the DevKit
    ("TMC_UART", SIG, [("U1", "31"), ("R3", "2")]),
    ("TMC_UART", SIG, [("R3", "2"), (17.0, 4.45), (41.28, 4.45), (41.28, 24.13), ("JP1", "2")]),
    ("UART_TX", SIG, [("U1", "30"), (16.28, 27.94), ("R3", "1")]),
    ("PDN_UART", SIG, [("JP1", "1"), ("U2", "4")]),
    ("PDN_ALT", SIG, [("JP1", "3"), ("U2", "5")]),
    # 3.3 V from the DevKit to the driver's logic supply and the pull-ups
    ("+3V3", V3, [("R1", "1"), (36.4, 34.29)]),
    ("+3V3", V3, [(36.4, 34.29), (36.4, 53.34), ("U1", "1")]),
    ("+3V3", V3, [(36.4, 34.29), (49.53, 34.29)]),
    ("+3V3", V3, [(49.53, 34.29), (52.07, 34.29), (52.07, 30.48), ("U2", "10")]),
    ("+3V3", V3, [("R2", "2"), (49.53, 34.29)]),
    # 5 V from the DevKit to the hall sensor
    ("+5V", V3, [("U1", "19"), (33.5, 7.62), (33.5, 57.15), (44.45, 57.15), (45.72, 58.42), ("J4", "1")]),
    # hall sensor
    ("HALL", SIG, [("U1", "8"), (41.91, 35.56), (43.18, 36.83), ("R4", "2")]),
    ("HALL", SIG, [("R4", "2"), ("C3", "1")]),
    ("HALL_IN", SIG, [("R4", "1"), (43.18, 42.0), (49.53, 42.0)]),
    ("HALL_IN", SIG, [(49.53, 42.0), (50.72, 42.0), ("J4", "3")]),
    ("HALL_IN", SIG, [("R2", "1"), (49.53, 42.0)]),
    ("GND", SIG, [("C3", "2"), (45.72, 40.9)]),
    # timing outputs to the Jetson
    ("MOVING", SIG, [("U1", "28"), ("R5", "2")]),
    ("MOVING_OUT", SIG, [("R5", "1"), ("J5", "1")]),
    ("TRIG", SIG, [("U1", "27"), ("R6", "2")]),
    ("TRIG_OUT", SIG, [("R6", "1"), ("J5", "2")]),
    # status LED on IO2
    ("STATUS", SIG, [("U1", "34"), ("R8", "2")]),
    ("STATUS_LED", SIG, [("R8", "1"), ("D4", "2")]),
    ("GND", SIG, [("D4", "1"), (1.9, 17.78)]),
    # 12 V input, fuse, reverse-polarity diode
    ("VIN", VM, [("J1", "1"), ("F1", "1")]),
    ("VIN", VM, [("J2", "1"), (81.0, 28.0), ("J1", "1")]),
    ("VIN_F", VM, [("F1", "2"), ("D1", "2")]),
    ("VM", VM, [("D1", "1"), (61.0, 14.0)]),
    ("VM", VM, [(61.0, 14.0), ("U2", "16")]),
    ("VM", VM, [("C1", "1"), (60.5, 13.5), (61.0, 14.0)]),
    ("VM", MOT, [("C2", "1"), ("C1", "1")]),
    ("VM", MOT, [("R7", "2"), ("C2", "1")]),
    ("VM", VM, [("D2", "1"), (67.0, 11.5), ("D1", "1")]),
    ("PWR_LED", SIG, [("R7", "1"), ("D3", "2")]),
    ("GND", MOT, [("C2", "2"), (63.0, 4.6), ("C1", "2")]),
    ("GND", MOT, [("D2", "2"), (72.6, 3.6)]),
    ("GND", SIG, [("D3", "1"), (49.9, 3.6)]),
    # motor coils
    ("COIL_A2", MOT, [("U2", "14"), ("J3", "1")]),
    ("COIL_A1", MOT, [("U2", "13"), ("J3", "2")]),
    ("COIL_B1", MOT, [("U2", "12"), ("J3", "3")]),
    ("COIL_B2", MOT, [("U2", "11"), ("J3", "4")]),
]

# ground vias: under SMD ground pads, and stitching the two ground pours
CONTROLLER_VIAS = [(45.72, 40.9), (72.6, 3.6), (49.9, 3.6), (1.9, 17.78),
                   (25.0, 12.0), (25.0, 30.0), (25.0, 48.0), (8.0, 12.0), (8.0, 50.0),
                   (56.0, 45.0), (70.0, 45.0), (84.0, 45.0), (75.0, 25.0), (60.0, 58.0), (84.0, 58.0)]

CONTROLLER_TEXT = [
    # text, x, y, size, angle, justify
    ("12 V IN", 80.0, 4.6, 1.0, 0, "center"),
    ("+", 83.0, 29.6, 1.0, 0, "center"),
    ("-", 83.0, 22.92, 1.0, 0, "center"),
    ("STATUS", 6.2, 19.7, 0.8, 0, "center"),
    ("A2", 70.2, 20.32, 0.9, 0, "left"),
    ("A1", 70.2, 22.82, 0.9, 0, "left"),
    ("B1", 70.2, 25.32, 0.9, 0, "left"),
    ("B2", 70.2, 27.82, 0.9, 0, "left"),
    ("MOTOR", 70.2, 30.4, 0.9, 0, "left"),
    ("HALL: 5V GND OUT", 55.0, 61.0, 0.9, 0, "left"),
    ("JETSON: MOVING TRIG GND", 1.3, 41.5, 0.8, 270, "left"),
    ("MOTOR PWR", 52.5, 6.2, 0.8, 0, "center"),
    ("SET TMC2209 VREF TO 0.28 V (0.2 A)", 56.0, 37.0, 1.0, 0, "left"),
    ("BEFORE CONNECTING THE NEMA 8.", 56.0, 39.0, 1.0, 0, "left"),
    ("NEVER PLUG OR UNPLUG THE MOTOR", 56.0, 41.6, 1.0, 0, "left"),
    ("WITH 12 V ON (LED LIT).", 56.0, 43.6, 1.0, 0, "left"),
    ("SCAN-MIRROR CONTROLLER", 56.0, 48.6, 1.5, 0, "left"),
    ("rev A  " + DATE, 56.0, 51.2, 0.9, 0, "left"),
    ("CoraBeanz/linescan-hyperspectral-splatting", 56.0, 53.2, 0.8, 0, "left"),
]

# ------------------------------------------------------------------------------
# Hall-sensor breakout
# ------------------------------------------------------------------------------
HALL_PARTS = [
    ("U1", "A3144", "A3144", "rig:A3144_UA_Leads", {"1": "+5V", "2": "GND", "3": "HALL_OUT"},
     hand("Allegro A3144 (Gikfun), UA package"), (101.6, 88.9),
     {"ref_at": (114.3, 81.28), "val_at": (114.3, 96.52)}),
    ("C1", "C", "100nF", "rig:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder", {"1": "+5V", "2": "GND"}, C100N,
     (139.7, 88.9), {}),
    ("J1", "Conn_01x03", "to controller J4", "rig:Wire_Pads_1x03_P2.54mm",
     {"1": "+5V", "2": "GND", "3": "HALL_OUT"}, hand("3-wire cable soldered in (or a 1x3 pin header)"),
     (170.18, 88.9), {}),
]
HALL_FLAGS = [("+5V", 76.2, 66.04), ("GND", 88.9, 66.04)]
HALL_NOTES = [
    (25.4, 25.4, 2.0, "Hall-sensor breakout for the scanner head's +X wall"),
    (25.4, 30.48, 1.27, "The A3144 sits in its pocket in the wall; "
                        "its leads pass through the wall's slot and this board."),
    (25.4, 34.29, 1.27, "C1 decouples the sensor at the end of its cable. "
                        "The output pull-up (10k to 3.3 V) is on the controller."),
    (25.4, 38.1, 1.27, "J1 pin 1 = 5 V, 2 = GND, 3 = OUT, the same order as the controller's J4."),
]

HALL_SIZE = (14.0, 12.4)
HALL_PLACE = {
    "U1": (7.0, 4.6, 0),
    "C1": (7.635, 1.9, 180),
    "J1": (9.54, 7.6, 180),
}
HALL_HOLES = []
HALL_REFS = {"U1": None, "C1": None, "J1": None}
HALL_M2 = (12.0, 2.3)
HALL_RELIEF = [(2.4, 10.6), (11.6, 10.6)]
HALL_ROUTES = [
    ("+5V", 0.4, [("U1", "1"), (8.27, 3.2), ("C1", "1")]),
    ("+5V", 0.4, [("U1", "1"), (8.27, 5.9), ("J1", "1")]),
    ("HALL_OUT", 0.4, [("U1", "3"), (5.73, 5.9), ("J1", "3")]),
    ("GND", 0.4, [("C1", "2"), (5.5, 1.9)]),
]
HALL_VIAS = [(5.5, 1.9)]
HALL_TEXT = [
    ("S", 4.46, 9.9, 0.9, 0, "center"),
    ("G", 7.0, 9.9, 0.9, 0, "center"),
    ("+", 9.54, 9.9, 0.9, 0, "center"),
]
HALL_BACK_TEXT = [("HALL rev A", 7.0, 6.0, 0.9, 0, "center")]
