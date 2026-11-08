"""Boards and modules that ride in Mico's torso and head, as envelopes for the CAD.

Every size is in millimetres. MEASURE: these come from datasheets and shop listings and differ between clones.
Measure the module you bought (outline, hole centres, tallest part) with calipers and update this file before
printing a sled: the sled, its bosses and the clearance checks are generated from here.

    size      (along the sled's long axis, across it)
    holes     hole centres from the board centre, same axes; empty = no holes, the module is tied to its sled
    hole_d    hole diameter on the board (an M2 screw passes every one listed)
    top       tallest part above the top face of the PCB, plugs inserted
    bottom    tallest part below the PCB (solder joints, wires)
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class Board:
    code: str
    name: str
    size: tuple
    pcb: float = 1.6
    holes: tuple = ()
    hole_d: float = 0.0
    top: float = 5.0
    bottom: float = 2.0
    mass: float = 0.0
    shop: str = ""


BOARDS = {
    "PCA": Board("PCA", "PCA9685 16-channel PWM module (Adafruit-style clone)", (62.5, 25.4),
                 holes=((-27.95, -9.5), (27.95, -9.5), (-27.95, 9.5), (27.95, 9.5)), hole_d=2.5, top=23.0,
                 bottom=1.5, mass=9.0, shop="mạch PCA9685 16 kênh"),                                   # MEASURE
    "RADXA": Board("RADXA", "Radxa Zero 3W (Pi Zero outline)", (65.0, 30.0),
                   holes=((-29.0, -11.5), (29.0, -11.5), (-29.0, 11.5), (29.0, 11.5)), hole_d=2.75, top=4.0,
                   bottom=1.5, mass=12.0, shop="Radxa Zero 3W"),                                       # MEASURE
    "PERF": Board("PERF", "A+ monitoring board: 7 × 5 cm perfboard carrying the STM32F411 Black Pill in female "
                          "headers, LMV358 × 6, CD74HC4067, MPU6050", (70.0, 50.0),
                  holes=((-31.5, -21.5), (31.5, -21.5), (-31.5, 21.5), (31.5, 21.5)), hole_d=2.2, top=21.0,
                  bottom=3.0, mass=28.0, shop="phíp đồng lỗ 7×5 cm (khoan 4 lỗ Ø2.2 theo bản vẽ)"),
    "UBEC": Board("UBEC", "6 V 5 A UBEC (heat-shrunk, no holes)", (45.0, 25.0), pcb=0, top=10.0, bottom=0,
                  mass=14.0, shop="UBEC 5A 6V"),                                                        # MEASURE
    "BUCK": Board("BUCK", "5 V buck for the Radxa (Mini560 class)", (22.0, 17.0), pcb=0, top=5.0, bottom=0,
                  mass=3.0, shop="mạch hạ áp Mini560 5V"),                                              # MEASURE
    "INA": Board("INA", "INA226 module CJMCU-226, R010 shunt", (21.0, 16.0), pcb=0, top=5.0, bottom=0,
                 mass=2.0, shop="CJMCU-226 INA226"),                                                   # MEASURE
    "BATT": Board("BATT", "2S LiPo 7.4 V (about 1000 mAh)", (56.0, 30.0), pcb=0, top=19.0, bottom=0,
                  mass=57.0, shop="pin LiPo 2S 1000 mAh"),                                              # MEASURE
    "CAM": Board("CAM", "Raspberry Pi Camera V2 (IMX219)", (25.0, 24.0),
                 holes=((-10.5, 6.25), (10.5, 6.25), (-10.5, -6.25), (10.5, -6.25)), hole_d=2.2, top=6.0,
                 bottom=2.5, mass=3.0, shop="camera Raspberry Pi V2 IMX219"),                          # MEASURE
    "TOF": Board("TOF", "VL53L0X module (GY-VL53L0XV2)", (25.0, 10.7), holes=((-9.5, 0.0), (9.5, 0.0)), hole_d=2.2,
                 top=3.0, bottom=1.5, mass=1.0, shop="cảm biến VL53L0X"),                               # MEASURE
    "DISP": Board("DISP", "1.9″ 170 × 320 IPS (ST7789) module", (50.0, 28.0), pcb=0, top=4.0, bottom=1.5, mass=6.0,
                  shop="màn IPS 1.9 inch ST7789"),                                                       # MEASURE
}
