#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import asyncio, json, datetime, math, contextlib, os
import websockets
from websockets.exceptions import ConnectionClosedOK, ConnectionClosedError
from rplidar import RPLidar, RPLidarException

# ====== CONFIG ======
# LiDAR
LIDAR_PORT = '/dev/ttyUSB0'   # Windows: 'COM4'
LIDAR_BAUD = 115200

# GPS (NMEA). Pon None si no quieres leer GPS aquí.
GPS_PORT = '/dev/serial0'
GPS_BAUD = 9600

# Brújula HMC5883L (I2C, 0x1E) usando smbus2
COMPASS_ENABLED = True
I2C_BUS_NUM = 1
HMC5883L_ADDR = 0x1E
# Poner en True si la brújula está montada "de cabeza" (invierte eje X)
COMPASS_UPSIDE_DOWN = True
# Declinación magnética en grados (ajústala a tu zona si quieres compensar)
MAG_DECLINATION_DEG = 0.0

# WebSocket
WS_HOST, WS_PORT = '0.0.0.0', 8765

# Backpressure: cuántos mensajes “pendientes” por cliente toleramos
CLIENT_MAX_QUEUE = 1

# ====== Estado global ======
CLIENTS = set()                # websockets conectados
latest_gps = {}                # snapshot de GPS
latest_compass = {}            # snapshot de brújula
gps_lock = asyncio.Lock()
compass_lock = asyncio.Lock()
shutdown_evt = asyncio.Event()

def now_iso():
    return datetime.datetime.utcnow().isoformat(timespec='milliseconds') + 'Z'

# ====== Broadcast auxiliar ======
async def broadcast_json(obj):
    if not CLIENTS:
        return
    payload = json.dumps(obj, separators=(',',':'))
    dead = []
    tasks = []
    for ws in CLIENTS:
        if getattr(ws, 'pending', 0) >= CLIENT_MAX_QUEUE:
            # cliente lento → soltamos el mensaje para él
            continue
        ws.pending = getattr(ws, 'pending', 0) + 1
        tasks.append(_send_one(ws, payload, dead))
    if tasks:
        await asyncio.gather(*tasks, return_exceptions=True)
    for ws in dead:
        CLIENTS.discard(ws)

async def _send_one(ws, payload, dead):
    try:
        await ws.send(payload)
    except (ConnectionClosedOK, ConnectionClosedError):
        dead.append(ws)
    except Exception:
        dead.append(ws)
    finally:
        ws.pending = max(0, getattr(ws, 'pending', 1) - 1)

# ====== WS handler ======
async def on_client(ws, path):
    ws.pending = 0
    CLIENTS.add(ws)
    print('[WS] Cliente', ws.remote_address, 'conectado (total=', len(CLIENTS),')')
    try:
        async for _ in ws:
            pass
    finally:
        CLIENTS.discard(ws)
        print('[WS] Cliente desconectado (total=', len(CLIENTS),')')

# ====== LiDAR loop ======
async def lidar_loop():
    lidar = None
    try:
        print('[LIDAR] Abriendo', LIDAR_PORT, '@', LIDAR_BAUD)
        lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUD)
        lidar.start_motor()
        loop = asyncio.get_running_loop()
        it = lidar.iter_scans()  # bloqueante → off-thread
        seq = 0
        while not shutdown_evt.is_set():
            scan = await loop.run_in_executor(None, next, it)
            seq += 1
            pts = [[a, d, q] for (q, a, d) in scan]

            # snapshots opcionales
            async with gps_lock:
                gps_snapshot = dict(latest_gps) if latest_gps else None
            async with compass_lock:
                comp_snapshot = dict(latest_compass) if latest_compass else None

            msg = {
                "type": "scan",
                "seq": seq,
                "ts": now_iso(),
                "points": pts,
                "gps": gps_snapshot,
                "compass": comp_snapshot
            }
            if seq % 5 == 0:
                print(f'[LIDAR] scan #{seq} → {len(pts)} pts')
            await broadcast_json(msg)
    except (RPLidarException, OSError) as e:
        print('[LIDAR] ⚠️', e)
    except asyncio.CancelledError:
        pass
    finally:
        if lidar:
            with contextlib.suppress(Exception):
                lidar.stop(); lidar.stop_motor(); lidar.disconnect()
        print('[LIDAR] loop terminado')

# ====== GPS (NMEA) loop ======
async def gps_loop():
    if not GPS_PORT:
        print('[GPS] Desactivado (GPS_PORT=None)')
        return
    try:
        import serial
    except ImportError:
        print('[GPS] pyserial no instalado; omitiendo GPS')
        return

    def nmea_checksum_ok(line: bytes) -> bool:
        try:
            s = line.strip()
            if not s.startswith(b"$") or b"*" not in s: return False
            star = s.rfind(b"*"); payload = s[1:star]; rx = s[star+1:star+3]
            calc = 0
            for b in payload: calc ^= b
            return calc == int(rx, 16)
        except Exception: return False

    def dm_to_deg(dm, hemi, is_lat):
        if not dm or not hemi: return None
        try:
            if is_lat:
                deg, minutes = int(dm[:2]), float(dm[2:])
            else:
                deg, minutes = int(dm[:3]), float(dm[3:])
            val = deg + minutes/60.0
            if str(hemi).upper() in ('S','W'): val = -val
            return val
        except: return None

    import serial
    print('[GPS] Abriendo', GPS_PORT, '@', GPS_BAUD)
    while not shutdown_evt.is_set():
        try:
            ser = serial.Serial(GPS_PORT, GPS_BAUD, timeout=1.0)
        except Exception as e:
            print('[GPS] No se pudo abrir:', e, 'reintento en 2s')
            await asyncio.sleep(2)
            continue

        try:
            while not shutdown_evt.is_set():
                raw = await asyncio.to_thread(ser.readline)
                if not raw or not raw.startswith(b"$") or b"*" not in raw:
                    continue
                if not nmea_checksum_ok(raw):
                    continue
                s = raw.strip().decode(errors='ignore')
                core = s[1:s.rfind("*")]
                p = core.split(",")
                typ = p[0][-3:]

                if typ == 'GGA':
                    fix = p[6] if len(p)>6 else None
                    sats = p[7] if len(p)>7 else None
                    lat = dm_to_deg(p[2], p[3], True) if len(p)>4 else None
                    lon = dm_to_deg(p[4], p[5], False) if len(p)>6 else None
                    alt = float(p[9]) if len(p)>9 and p[9] else None
                    data = {
                        "type":"gps","ts":now_iso(),
                        "fix": int(fix) if fix and fix.isdigit() else None,
                        "sats": int(sats) if sats and sats.isdigit() else None,
                        "lat": lat, "lon": lon, "alt_m": alt
                    }
                    async with gps_lock:
                        for k,v in data.items():
                            if k not in ('type',) and v is not None:
                                latest_gps[k] = v
                    await broadcast_json(data)

                elif typ == 'RMC':
                    spd_kn = p[7] if len(p)>7 else None
                    crs = p[8] if len(p)>8 else None
                    data = {
                        "type":"gps","ts":now_iso(),
                        "speed_kn": float(spd_kn) if spd_kn else None,
                        "course": float(crs) if crs else None,
                    }
                    async with gps_lock:
                        for k in ('speed_kn','course'):
                            if data[k] is not None:
                                latest_gps[k] = data[k]
                    await broadcast_json(data)
        except asyncio.CancelledError:
            pass
        except Exception as e:
            print('[GPS] ⚠️', e)
        finally:
            with contextlib.suppress(Exception): ser.close()
            await asyncio.sleep(0.5)
    print('[GPS] loop terminado')

# ====== Brújula (I2C, HMC5883L @ 0x1E) ======
class CompassHMC5883L:
    """
    Implementación basada en tu script:
    - smbus2.SMBus(1)
    - Dirección 0x1E
    - Config: 0x00=0x70 (8 muestras, 15Hz), 0x01=0xA0 (ganancia), 0x02=0x00 (continuo)
    - Lectura: X(0x03,0x04), Z(0x05,0x06), Y(0x07,0x08)
    - Heading = atan2(Y, X) [+ declinación]
    """
    def __init__(self, bus_num=1, addr=0x1E, decl_deg=0.0, upside_down=False):
        try:
            import smbus2
        except Exception as e:
            raise RuntimeError(f"smbus2 no disponible: {e}")
        self.smbus2 = __import__('smbus2')
        self.bus = self.smbus2.SMBus(bus_num)
        self.addr = addr
        self.decl = math.radians(decl_deg)
        self.upside_down = upside_down
        # init registers
        self._write8(0x00, 0x70)  # Config A: 8 muestras, 15Hz
        self._write8(0x01, 0xA0)  # Config B: rango ±5 gauss
        self._write8(0x02, 0x00)  # Modo: continuo

    def _write8(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val)

    def _read_axis(self, msb_reg, lsb_reg):
        msb = self.bus.read_byte_data(self.addr, msb_reg)
        lsb = self.bus.read_byte_data(self.addr, lsb_reg)
        val = (msb << 8) | lsb
        if val > 32767:
            val -= 65536
        return val

    def read_raw(self):
        x = self._read_axis(0x03, 0x04)
        z = self._read_axis(0x05, 0x06)
        y = self._read_axis(0x07, 0x08)
        return float(x), float(y), float(z)

    def heading_deg(self):
        x, y, z = self.read_raw()
        if self.upside_down:
            x = -x
        rad = math.atan2(y, x) + self.decl
        deg = math.degrees(rad)
        if deg < 0: deg += 360.0
        if deg >= 360.0: deg -= 360.0
        return deg, (x, y, z)

# ====== Compass loop ======
async def compass_loop():
    if not COMPASS_ENABLED:
        print('[COMPASS] Desactivado')
        return
    # comprobación rápida de /dev/i2c-*
    if not os.path.exists(f"/dev/i2c-{I2C_BUS_NUM}"):
        print(f'[COMPASS] Bus I2C {I2C_BUS_NUM} no existe. Habilita I2C (raspi-config) y reinicia.')
        return
    try:
        comp = CompassHMC5883L(
            I2C_BUS_NUM,
            HMC5883L_ADDR,
            MAG_DECLINATION_DEG,
            upside_down=COMPASS_UPSIDE_DOWN,
        )
        print(f'[COMPASS] HMC5883L detectado @ 0x{HMC5883L_ADDR:02X}')
    except Exception as e:
        print('[COMPASS] No disponible:', e)
        return

    while not shutdown_evt.is_set():
        try:
            # llama en thread para no bloquear el loop
            hdg_deg, (x, y, z) = await asyncio.to_thread(comp.heading_deg)
            snap = {
                "type":"compass",
                "ts": now_iso(),
                "heading_deg": round(hdg_deg, 1),
                "x": int(x), "y": int(y), "z": int(z),
                "model": "HMC5883L",
                "upside_down": COMPASS_UPSIDE_DOWN,
            }
            async with compass_lock:
                latest_compass.clear()
                latest_compass.update(snap)
            await broadcast_json(snap)
            await asyncio.sleep(0.1)  # ~10 Hz
        except asyncio.CancelledError:
            break
        except Exception as e:
            print('[COMPASS] ⚠️', e)
            await asyncio.sleep(0.2)
    print('[COMPASS] loop terminado')

# ====== MAIN ======
async def main():
    tasks = [asyncio.create_task(lidar_loop())]
    if GPS_PORT:
        tasks.append(asyncio.create_task(gps_loop()))
    if COMPASS_ENABLED:
        tasks.append(asyncio.create_task(compass_loop()))

    async with websockets.serve(
        on_client, WS_HOST, WS_PORT,
        max_size=8*1024*1024,
        max_queue=0,
        compression='deflate',
    ):
        print(f'[WS] WebSocket en ws://{WS_HOST}:{WS_PORT}')
        try:
            while True:
                print('[PY]', now_iso(), 'alive')
                await asyncio.sleep(5)
        except asyncio.CancelledError:
            pass
        finally:
            shutdown_evt.set()
            for t in tasks: t.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            print('[MAIN] apagado')

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
