# Training Board firmware — IMU bridge for Avionics C

The Avionics C onboard IMU could not be repaired, so this board is bodged onto
Avionics C's J4 header and streams its own LSM6DSV80X samples over UART.
Avionics C consumes that stream instead of its onboard IMU.

## Wiring (Training Board J4 -> Avionics C J4)

| Training Board J4 (1x16, 1.27 mm) | Avionics C J4 (JST-XH 4 pin, "UART breakout for backup GPS") |
|---|---|
| pin 1  `+3.3V` | pin 1 `+3.3V` |
| pin 2  `GND`   | pin 4 `GND` |
| pin 15 `TX0` (GPIO43) | pin 3 `RX0` (GPIO44) |

## Data path

- IMU at 960 Hz (low-g accel ±16 g, high-g accel ±64 g, gyro ±2000 dps), same
  configuration as Avionics C's onboard sensor.
- INT1 data-ready pulse (GPIO39) wakes a high-priority task on core 1, which
  reads the sample in two SPI burst transfers and writes one 32-byte frame
  straight into the UART0 TX FIFO at 921600 baud.
- Frame: sync `A5 5A`, protocol version, sequence number, sender timestamp,
  raw int16 sensor data, fresh-data flags, 8-bit checksum, CRC-16/CCITT-FALSE.
  See `main/imu_link.h` (byte-identical copy lives in the Avionics C repo).
- Console/logs go out over the native USB Serial/JTAG port (UART0 is the link).
  Stats are printed once per second: packets sent, rate, interrupt vs. poll
  fallbacks, SPI/UART errors, and the last sample.

## Raw sensor stream over USB (branch `spm-imu-usb-stream`, SPM hip rig)

On this branch the board also streams every IMU sample (240 Hz, raw LSB), the magnetometer
(~100 Hz) and a 1 Hz INFO frame (scale factors, drop counters) over the native USB port.
Everything on that port is framed: `A5 96 | type | len | payload | CRC-16/CCITT-FALSE (LE)`.
ESP_LOG text is wrapped in TEXT frames so logs never corrupt samples. See `main/usb_stream.h`.
The UART link to Avionics C is unchanged. The host side lives in the spmcontroller repo
(`spm/imu_board.py`). When nothing has the port open, frames are dropped (counted in INFO),
and the PC discards the stale buffered frames it receives when the port is opened.

## Datalogging (SD card + NOR flash)

Besides streaming the IMU, the board logs an 80-byte record every 10 ms
(100 Hz) with the latest IMU sample sent, the barometer and the magnetometer:

- SD card: `/sdcard/session_NNNNNN.bin`, one file per boot (never overwritten).
- NOR flash (W25Q64, 8 MB, raw append): the same records. **At every boot the
  whole NOR backlog is copied to `/sdcard/nor_dump_NNNNNN.bin` and the NOR is
  wiped only after that dump verified** (scheme from the Avionics B
  coprocessor). No card, or a failed dump, means the NOR keeps its data.
- Session number, NOR write pointer and a 10-entry boot log (reset reason,
  dump outcome) live in NVS. `nor_log.c` / `storage.c` are byte-identical
  copies of the Avionics C files.
- Record layout: `main/tb_log.h` (`tb_log_record_t`, magic `SBTB`, CRC32).
  `imu_t_us` / `imu_pkt_count` identify the last IMU frame sent over UART;
  Avionics C records the same `imu_t_us` with its own clock, which is how the
  two logs are re-aligned if the UART link drops.
- Boot order matters on the shared SPI2 bus: SD card first (a freshly powered
  card in native SD mode disturbs the bus until CMD0 puts it in SPI mode),
  then NOR, then IMU.

The bridge task keeps the highest priority on core 1; storage tasks run on
core 0. They only meet at the SPI2 bus lock, and the per-second stats report
how many IMU reads were delayed by more than 1 ms / 5 ms because of it.

## Reconstructing a session

`tools/parse_logs.py` reads either `session_NNNNNN.bin` or `nor_dump_NNNNNN.bin`
from this board or from Avionics C, verifies every record's CRC32, and writes
CSV with engineering units (mg, dps, degC) or a PNG overview.

```sh
tools/parse_logs.py /Volumes/CARD/nor_dump_000191.bin
tools/parse_logs.py nor_dump_000191.bin --csv session.csv --plot session.png
tools/parse_logs.py --pair <avionics_c_log> <training_board_log>   # clock alignment
```

## Code layout

- `main/main.c` — buses, boot sequence (SD, NOR, IMU, session, dump), stats logging.
- `main/bridge.c` — IMU -> UART bridge task (INT1, SPI burst read, frame TX).
- `main/tb_log.c`, `main/sdcard.c`, `main/nor_log.c`, `main/storage.c` — datalogging.
- `main/imu_polling.c`, `main/imu.h` — LSM6DSV80X init (WHO_AM_I probe over SPI
  mode 0/3, reset, ODR/FS/filters, INT1 routing) and burst sample read.
- `main/imu_link.{h,c}` — link protocol (framing, checksum, CRC). Pure C.
- `main/pins.h` — pin map.
- `main/barometer.c`, `main/magnetometer.c`, `main/sensor_data.c` — I2C sensor
  drivers and the shared latest-value struct used by the logger.
- `components/` — vendor drivers.

## Build / flash / monitor

- Set Target: `idf.py set-target esp32s3`
- Build: `idf.py build`
- Flash + monitor: `idf.py --port <port> flash monitor`
