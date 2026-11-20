#!/usr/bin/env python3
"""Connected-Pico serial validation (macOS/Linux; Python standard library only).

This replaces/runs the device's pianola. Only the chosen logical LED bit is
asserted, but disconnect attached equipment if its mapping is uncertain.
No output-capture hardware is used: automated PASS means protocol/responsiveness,
not that the requested waveform actually appeared. --interactive adds visual
waveform and stop/restart observations, separately labelled HUMAN.
"""
import argparse
import glob
import json
import math
import os
from pathlib import Path
import random
import select
import sys
import time

TICK = 2.56e-6
MASK = 0xFFFFFF

class Failure(Exception):
    """A semantic assertion failed; the byte stream may still be usable."""

class TransportFailure(Failure):
    """Stop testing: reply boundaries or connection health can no longer be trusted."""

class TimingWarning(Exception):
    pass

def ticks(seconds):
    value = math.floor(seconds / TICK + .5)
    if not 1 <= value <= MASK:
        raise ValueError('Entry duration must be 1..16777215 ticks')
    return value

def u24(value):
    return (value & MASK).to_bytes(3, 'big')

def entry(address, pattern, duration):
    if not 0 <= address <= 255 or not 0 <= pattern <= 255 or not 0 <= duration <= MASK:
        raise ValueError('Invalid sequence entry')
    return bytes((1, address, pattern)) + u24(duration)

class SerialPort:
    """Raw 115200 8N1 tty with monotonic deadlines on BOTH reads and writes.

    Opening a real tty is confined to this class. The rest is exercised offline
    with a scripted peer. No background reader or overlapping transactions exist.
    """
    def __init__(self, path):
        import termios
        import tty
        import fcntl
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.saved = None
        try:
            self.saved = termios.tcgetattr(self.fd)
            if hasattr(termios, 'TIOCEXCL'):
                fcntl.ioctl(self.fd, termios.TIOCEXCL)
            tty.setraw(self.fd)
            settings = termios.tcgetattr(self.fd)
            settings[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE | getattr(termios, 'CRTSCTS', 0))
            settings[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
            settings[4] = settings[5] = termios.B115200
            termios.tcsetattr(self.fd, termios.TCSANOW, settings)
        except BaseException:
            self.close()
            raise

    def write(self, data, timeout):
        deadline = time.monotonic() + timeout
        remaining = memoryview(data)
        while remaining:
            wait = deadline - time.monotonic()
            if wait <= 0 or not select.select([], [self.fd], [], max(0, wait))[1]:
                raise TransportFailure(f'Write timeout with {len(remaining)} bytes unsent')
            try:
                n = os.write(self.fd, remaining)
                if not n:
                    raise TransportFailure('Serial write returned zero bytes')
                remaining = remaining[n:]
            except BlockingIOError:
                continue

    def read(self, size, timeout):
        if not select.select([self.fd], [], [], max(0, timeout))[0]:
            return b''
        try:
            data = os.read(self.fd, size)
        except BlockingIOError:
            return b''
        if not data:
            raise TransportFailure('Serial device disconnected (EOF)')
        return data

    def close(self):
        if getattr(self, 'fd', None) is not None:
            import termios
            try:
                if self.saved is not None:
                    termios.tcsetattr(self.fd, termios.TCSANOW, self.saved)
            finally:
                os.close(self.fd)
                self.fd = None

class Box:
    def __init__(self, port, timeout=.5, quiet=.003):
        self.port, self.timeout, self.quiet = port, timeout, quiet
        self.trace = []
        self.latencies = []
        self.transactions = 0

    def send(self, data):
        self.trace.append({'t': time.monotonic(), 'tx': data.hex()})
        self.port.write(data, self.timeout)

    def query(self, data, count):
        # Never send a wake-up command/retry while waiting for a reply: this
        # specifically detects responses stranded until another RX interrupt.
        started = time.monotonic()
        self.send(data)
        deadline, response = started + self.timeout, bytearray()
        while len(response) < count:
            wait = deadline - time.monotonic()
            if wait <= 0:
                raise TransportFailure(f'Opcode 0x{data[0]:02x}: expected {count} reply bytes, received {len(response)} ({response.hex() or "none"}); no additional command was sent to wake the device')
            response.extend(self.port.read(count - len(response), wait))
        finished = time.monotonic()
        self.trace.append({'t': finished, 'rx': response.hex()})
        self.latencies.append(finished - started)
        self.transactions += 1
        extra = self.port.read(256, self.quiet)
        if extra:
            self.trace.append({'t': time.monotonic(), 'unexpected_rx': extra.hex()})
            raise TransportFailure(f'Opcode 0x{data[0]:02x}: unsolicited/extra reply bytes {extra.hex()}')
        return bytes(response)

    def clock(self):
        return int.from_bytes(self.query(b'\x08', 3), 'big')

    def version(self):
        return tuple(self.query(b'\xfd', 2))

    def stop(self):
        self.send(b'\x07')

    def upload(self, entries, repeat=False, repeat_from=0, burst=False):
        if not 1 <= len(entries) <= 256 or not 0 <= repeat_from < len(entries):
            raise ValueError('Sequence/repeat range invalid')
        self.stop()
        # The clock response is a processing barrier for stop, not proof of its
        # physical effect. No-response uploads are intentionally tested in bursts.
        self.clock()
        packets = [entry(i, mask, duration) for i, (mask, duration) in enumerate(entries)]
        packets += [bytes((3, repeat_from)), bytes((4, int(repeat))), bytes((2, len(entries)-1))]
        if burst:
            self.send(b''.join(packets))
        else:
            # A conservative upload uses response barriers. Burst mode omits
            # these, matching the host's actual no-response command batching.
            for packet in packets:
                self.send(packet)
                self.clock()
        self.clock()

    def run(self):
        return int.from_bytes(self.query(b'\x05', 3), 'big')

    def schedule(self, lead_ms):
        now = self.clock()
        target = (now + ticks(lead_ms / 1000)) & MASK
        response = self.query(b'\x06' + u24(target), 4)
        if response[0] not in (0, 1):
            raise TransportFailure(f'Scheduled-start status is not boolean: {response.hex()}')
        if not response[0]:
            reported = int.from_bytes(response[1:], 'big')
            raise Failure(f'Scheduled start rejected: target=0x{target:06x}, reported now=0x{reported:06x}, lead={lead_ms:g} ms (USB/host delay can consume the lead)')
        return target

    def synchronise(self):
        # A reset train is the XMOS recovery convention: enough bytes to finish
        # even the longest partial packet, followed by complete reset opcodes.
        self.send(b'\xff' * 32)
        end = time.monotonic() + .15
        discarded = bytearray()
        while time.monotonic() < end:
            discarded.extend(self.port.read(256, min(.01, end-time.monotonic())))
            if len(discarded) > 4096:
                raise TransportFailure('Device continuously emitting data during initial synchronisation')
        if discarded:
            self.trace.append({'t': time.monotonic(), 'startup_discarded': discarded.hex()})
        return self.version()

    def all_low(self):
        # Stop alone does not promise low outputs. Upload/run an explicit low
        # entry, allow execution, then stop. Best effort only if serial is broken.
        self.upload([(0, ticks(.01))])
        self.run()
        time.sleep(.03)
        self.stop()

class Reporter:
    def __init__(self, strict=False):
        self.results, self.strict, self.broken = [], strict, False

    def test(self, name, area, fn, human=False):
        if self.broken:
            self.results.append(dict(name=name, area=area, category='HUMAN' if human else 'FUNCTIONAL', status='SKIP', detail='Serial stream no longer trusted; power-cycle/reconnect before retrying'))
            print(f'SKIP  {name} — serial stream no longer trusted', flush=True)
            return
        category = 'HUMAN' if human else 'FUNCTIONAL'
        try:
            fn()
            status, detail = 'PASS', ''
        except ObservationSkipped as exc:
            status, detail = 'SKIP', str(exc)
        except TimingWarning as exc:
            category, status, detail = 'TIMING', 'WARN', str(exc)
        except OSError as exc:
            status, detail = 'FAIL', f'Serial I/O error: {exc}'
            self.broken = True
        except Failure as exc:
            status, detail = 'FAIL', str(exc)
            if isinstance(exc, TransportFailure):
                self.broken = True
        self.results.append(dict(name=name, area=area, category=category, status=status, detail=detail))
        print(f'{status:4}  [{category}] {name} — {area}', flush=True)
        if detail:
            print('      ' + detail, flush=True)

    def exit_code(self):
        return int(any(r['status']=='FAIL' or (self.strict and r['status']=='WARN') for r in self.results))

def automated(box, report, args):
    led = 1 << args.led_bit
    expected_version = None
    def identify():
        nonlocal expected_version
        expected_version = box.synchronise()
        if expected_version != (6, 6):
            raise TransportFailure(f'Expected this firmware/protocol version (6,6), got {expected_version}; verify target before running sequences')
        print(f'      Firmware/current-compatible versions: {expected_version}', flush=True)
    report.test('identify', 'reset/reframe, then version query with no wake-up command', identify)

    def idle_queries():
        for i in range(10):
            if box.version() != expected_version:
                raise Failure(f'Idle version query {i}: inconsistent reply')
            box.clock()
            time.sleep(.01)
    report.test('idle_responses', 'isolated queries must receive replies without further TX', idle_queries)

    def clock_progress():
        start = time.monotonic(); first = box.clock(); first_done = time.monotonic()
        time.sleep(.10)
        last_start = time.monotonic(); last = box.clock(); end = time.monotonic()
        delta = (last-first) & MASK
        if not 0 < delta < 0x800000:
            raise Failure(f'Clock did not progress forwards: {first:06x} -> {last:06x}')
        # USB delays mean this is deliberately a broad interval consistency check,
        # not oscillator calibration. Firmware samples must lie within each query.
        low, high = last_start-first_done, end-start
        elapsed = delta*TICK
        if not low-.005 <= elapsed <= high+.005:
            raise TimingWarning(f'Clock advanced {elapsed:.6f}s; host query bounds approximately {low:.6f}..{high:.6f}s; USB buffering or stale/device timing may be involved')
    report.test('clock_progress', 'clock progression over 100 ms; coarse host-time bounds', clock_progress)

    def source_check():
        source, inverted = box.query(bytes((10, args.led_bit)), 2)
        if (source, inverted) != (args.led_bit, 0):
            raise TransportFailure(f'Chosen LED output {args.led_bit} reports source/inversion {(source,inverted)}, expected {(args.led_bit,0)}; this script requires the fixed Pico mapping')
    report.test('led_mapping', 'validate chosen logical output readback before playback', source_check)

    def uploads():
        for count in (3, 32, 256):
            sequence = [(led if i%2==0 else 0, 4 + i%19) for i in range(count)]
            box.upload(sequence, burst=True)
            box.run()
            box.stop()
            if box.version() != expected_version:
                raise Failure(f'Upload length {count}: query corrupted after playback/stop')
        print('      Replies/framing verified; protocol has no sequence readback, so uploaded contents are not verified.', flush=True)
    report.test('upload_bursts', 'back-to-back uploads including full 256-entry sequence', uploads)

    def scheduling():
        box.upload([(led, ticks(.025)), (0, ticks(.075))], repeat=True)
        box.schedule(args.lead_ms)
        box.schedule(args.lead_ms * 2)  # Replacement while initially pending where host latency permits.
        time.sleep(args.lead_ms * 2 / 1000 + .03)
        box.run()                       # Restart while the loop should be playing.
        box.clock()
        box.stop()
        box.clock()
        print('      Start acceptance, replies and continued service verified; physical cancellation/restart not observed.', flush=True)
    report.test('scheduled_restart_stop', 'schedule/replace/restart/stop with replies', scheduling)

    def fragmented():
        # A complete, delayed packet tests ordinary fragmentation. Do not leave
        # an intentionally truncated packet which could wedge existing firmware.
        packet = entry(0, 0, 4)
        box.stop();box.clock()
        box.send(packet[:2]);time.sleep(.02);box.send(packet[2:]);box.clock()
        if box.version() != expected_version:
            raise Failure('Fragmented complete packet corrupted following query')
    report.test('fragmented_upload', 'one packet delivered in two writes with a 20 ms pause', fragmented)

    def stress():
        rng = random.Random(args.seed)
        sequence = [(led if i%2==0 else 0, 4 if i%4 else ticks(.01)) for i in range(32)]
        box.upload(sequence, repeat=True, burst=True)
        box.schedule(args.lead_ms)
        deadline = time.monotonic() + args.seconds
        iterations = 0
        while time.monotonic() < deadline:
            action = rng.randrange(6)
            if action == 0:
                if box.version() != expected_version:
                    raise Failure(f'Seed {args.seed}, event {iterations}: incorrect version reply')
            elif action == 1:
                source = box.query(bytes((10, args.led_bit)), 2)
                if source != bytes((args.led_bit, 0)):
                    raise Failure(f'Seed {args.seed}, event {iterations}: incorrect fixed-source reply {source.hex()}')
            elif action == 2:
                box.run()
            elif action == 3:
                box.stop();box.clock();box.schedule(args.lead_ms)
            elif action == 4:
                box.schedule(args.lead_ms)
            else:
                box.clock()
            iterations += 1
        box.stop();box.clock()
        print(f'      Completed {iterations} seeded events over {args.seconds:g}s (seed={args.seed}).', flush=True)
    report.test('mixed_traffic', 'bounded serialized command load while playback is requested', stress)
    return led

def ask(question):
    while True:
        try:
            answer = input(question + ' [y/n/s=skip]: ').strip().lower()
        except EOFError:
            raise Failure('No interactive input; omit --interactive for unattended runs') from None
        if answer in ('y', 'yes'):
            return
        if answer in ('n', 'no'):
            raise Failure('Operator did not observe the expected LED behaviour')
        if answer in ('s', 'skip'):
            raise ObservationSkipped('Operator skipped this observation')

class ObservationSkipped(Failure):
    pass

def ready_to_watch(instructions):
    """Gate every observed action; the operator need not read during a window."""
    print('\n' + instructions, flush=True)
    print('The onboard LED should be OFF while you read. If it is ON/flashing, enter n.', flush=True)
    while True:
        try:
            answer = input('Press Return when ready to watch the LED (s=skip, n=wrong starting state): ').strip().lower()
        except EOFError:
            raise Failure('No readiness input; omit --interactive for unattended runs') from None
        if not answer:
            return
        if answer in ('n', 'no'):
            raise Failure('LED was not OFF before the observation; test was not started')
        if answer in ('s', 'skip'):
            raise ObservationSkipped('Operator skipped before starting the observation')
        print('Press Return to start, s to skip, or n if the LED is not OFF.', flush=True)


def human_tests(box, report, args, led):
    print('\nHUMAN checks: watch the Pico onboard LED (GP25), not the green/blue status LEDs.', flush=True)

    def prepare_dark():
        # Previous script patterns have at most a one-second cycle. Allow any
        # already queued work to finish, then execute a separate low pattern.
        # Operator readiness also verifies the physical state: replies cannot.
        box.stop(); time.sleep(1.1)
        box.all_low(); time.sleep(.05)

    def observation_finished(stop_playback=True):
        if stop_playback:
            box.stop()
        print('Observation finished — look back at the terminal to answer.', flush=True)

    def blink():
        prepare_dark()
        box.upload([(led, ticks(.25)), (0, ticks(.75))], repeat=True, burst=True)
        ready_to_watch('BLINK CHECK: After Return, expect about half a second OFF, then watch\n'
                       '5 seconds of flashing once per second: ON briefly (about 1/4 second),\n'
                       'OFF for about 3/4 second.\n'
                       'You do not need to read the screen again until the viewing window ends.')
        # This is a visual waveform check, not a short-lead acceptance test.
        # A timestamp returned through USB serial may already be ~16 ms old;
        # use ample margin rather than letting the default 10 ms prevent viewing.
        box.schedule(500)
        time.sleep(5.5)
        observation_finished()
        ask('During the viewing window, did the LED flash about once per second with roughly 25% ON time?')
    report.test('led_1hz_25percent', 'visible scheduled repeating waveform; coarse timing only', blink, human=True)

    def stop():
        prepare_dark()
        box.upload([(led, ticks(.25)), (0, ticks(.75))], repeat=True)
        ready_to_watch('STOP CHECK: After Return, watch for 7 seconds. The LED should blink for\n'
                       'about 4 seconds, then stop blinking and remain steady for the rest.\n'
                       'It should freeze ON (entry zero is ON). No input is needed during viewing.')
        box.run()
        time.sleep(4)
        box.stop()
        time.sleep(3)
        observation_finished()
        ask('Did the LED first blink, then stop and remain continuously ON during the final few seconds?')
    report.test('led_stop', 'visible stop; freeze entry-zero ON pattern', stop, human=True)

    def replacement():
        prepare_dark()
        box.upload([(led, ticks(.25)), (0, ticks(.75))], repeat=True)
        ready_to_watch('REPLACEMENT CHECK: After Return, watch for 10 seconds. The LED should stay\n'
                       'OFF for about 6 seconds, then blink once per second for the rest.\n'
                       'In particular, it must NOT start after just 2 seconds. Both questions\n'
                       'will be asked AFTER the entire viewing window, not halfway through.')
        # No prompts or console instructions between scheduling and observation.
        # A longer lead makes replacement visible without precise instrumentation.
        box.schedule(2000)
        box.schedule(6000)
        time.sleep(10)
        observation_finished()
        ask('Did the LED remain OFF for roughly the first 6 seconds (no early start at about 2 seconds)?')
        ask('Did it then BEGIN blinking and continue for the rest of the viewing window?')
    report.test('led_replace_pending', 'visible replacement of a pending start', replacement, human=True)

    def restart():
        prepare_dark()
        # One-shot, deliberately not repeating: later cycles cannot be mistaken
        # for successful restart. Without restart ON lasts 4 s; a restart at 3 s
        # should extend it to 7 s continuously, followed by a latched low pattern.
        box.upload([(led, ticks(4)), (0, ticks(2))], repeat=False)
        ready_to_watch('RESTART CHECK (ONE-SHOT): After Return, watch for 11 seconds. The LED\n'
                       'should turn ON and stay continuously ON for about 7 seconds, then turn\n'
                       'OFF and stay OFF for the rest. The script restarts the 4-second ON phase\n'
                       'after 3 seconds. An early OFF near 4 seconds, any OFF gap, or coming\n'
                       'back ON afterwards is a failure. No input is needed during viewing.')
        box.run()
        time.sleep(3)
        box.run()
        time.sleep(8)
        # This one-shot is already finished. StopAndReset would intentionally
        # apply its ON entry zero again, falsely appearing to be a restart glitch.
        observation_finished(stop_playback=False)
        ask('Was the LED continuously ON for about 7 seconds, then OFF for the rest, without gaps or relighting?')
    report.test('led_restart', 'one-shot restart extends a continuous ON interval', restart, human=True)

    def final_hold():
        prepare_dark()
        box.upload([(0, ticks(1)), (led, 1)], repeat=False)
        ready_to_watch('FINAL HOLD CHECK: After Return, watch for 4 seconds. The LED should\n'
                       'stay OFF for about one second, then turn ON and stay ON until you answer.')
        box.run()
        time.sleep(4)
        observation_finished(stop_playback=False)
        ask('Did the LED turn ON after about one second and stay ON for the rest of the observation?')
    report.test('led_final_hold', 'one-shot playback holds the final ON pattern', final_hold, human=True)

def choose_port():
    """Offer only macOS USB-serial callout ports; never open one implicitly."""
    ports = sorted(set(glob.glob('/dev/cu.usbserial*')))
    if not ports:
        raise ValueError('No /dev/cu.usbserial* ports found. Connect the device, or specify --port explicitly.')
    print('Available USB-serial ports:', flush=True)
    for index, port in enumerate(ports, 1):
        print(f'  {index}. {port}', flush=True)
    while True:
        try:
            answer = input(f'Choose a port [1-{len(ports)}, q=cancel]: ').strip()
        except EOFError:
            raise ValueError('No selection received. Use --port for unattended runs.') from None
        if answer.lower() in ('q', 'quit'):
            return None
        if answer.isascii() and answer.isdecimal() and 1 <= int(answer) <= len(ports):
            return ports[int(answer)-1]
        print(f'Enter a number from 1 to {len(ports)}, or q to cancel.', flush=True)

def parser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', help='serial device; if omitted, choose from available /dev/cu.usbserial* ports')
    ap.add_argument('--list-ports', action='store_true', help='list candidate tty paths without opening a device')
    ap.add_argument('--seconds', type=float, default=10, help='mixed-traffic duration in seconds (default 10)')
    ap.add_argument('--timeout', type=float, default=.5, help='maximum seconds per transaction/write (default .5)')
    ap.add_argument('--lead-ms', type=float, default=10, help='scheduled-start lead, ms (default 10; USB latency can matter)')
    ap.add_argument('--seed', type=int, default=20261120, help='reproducible mixed-traffic seed')
    ap.add_argument('--led-bit', type=int, default=4, choices=range(8), help='logical sequence LED bit (default 4 = GP25 in current Pico firmware)')
    ap.add_argument('--interactive', action='store_true', help='add optional human LED observations')
    ap.add_argument('--strict-timing', action='store_true', help='timing warnings also fail the exit status')
    ap.add_argument('--json', type=Path, help='write results, timings and full serial trace to this file')
    return ap

def main(argv=None):
    ap = parser();args = ap.parse_args(argv)
    if args.list_ports:
        for path in sorted(set(glob.glob('/dev/cu.*') + glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))):
            print(path)
        return 0
    if not all(math.isfinite(x) for x in (args.seconds,args.timeout,args.lead_ms)) or not 0 < args.seconds <= 3600 or not .01 <= args.timeout <= 10 or not 1 <= args.lead_ms <= 1000:
        ap.error('Require 0 < seconds <= 3600, .01 <= timeout <= 10, and 1 <= lead-ms <= 1000')
    if args.interactive and args.led_bit != 4:
        ap.error('Interactive onboard-LED checks require --led-bit 4')
    if not args.port:
        try:
            args.port = choose_port()
        except ValueError as exc:
            ap.error(str(exc))
        except KeyboardInterrupt:
            print('\nCancelled; no device opened.')
            return 130
        if args.port is None:
            print('Cancelled; no device opened.')
            return 0
    report, port, box = Reporter(args.strict_timing), None, None
    cleanup = 'not attempted'
    interrupted = False
    try:
        print(f'Opening {args.port} at 115200 8N1. This replaces/runs the pianola; logical LED bit={args.led_bit}.', flush=True)
        print('Disconnect attached equipment if mapping is uncertain. Automated results check serial service, not physical waveforms.', flush=True)
        port = SerialPort(args.port);box = Box(port,args.timeout)
        led = automated(box,report,args)
        if args.interactive:
            human_tests(box,report,args,led)
    except KeyboardInterrupt:
        interrupted = True
        print('\nInterrupted; attempting cleanup.', flush=True)
    except (OSError, ValueError, TransportFailure) as exc:
        report.results.append(dict(name='connection',area='serial transport',category='FUNCTIONAL',status='FAIL',detail=str(exc)))
        print(f'FAIL  Connection: {exc}', flush=True)
    finally:
        if box is not None:
            try:
                if report.broken:
                    box.stop();cleanup='Stop command sent; serial stream untrusted, output state unverified'
                else:
                    box.all_low();cleanup='Explicit low sequence and stop sent; output state not physically verified'
            except (Failure,OSError) as exc:
                cleanup=f'Cleanup failed: {exc}; power-cycle/check outputs'
                report.results.append(dict(name='cleanup',area='best-effort low/stop',category='FUNCTIONAL',status='FAIL',detail=cleanup))
            print('Cleanup: ' + cleanup, flush=True)
        if port is not None:
            try:
                port.close()
            except OSError as exc:
                report.results.append(dict(name='close',area='restore/close serial port',category='FUNCTIONAL',status='FAIL',detail=str(exc)))
                print(f'FAIL  Serial close: {exc}', flush=True)
    if box and box.latencies:
        ordered=sorted(box.latencies)
        print(f'Replies: {box.transactions}; latency median={ordered[len(ordered)//2]*1000:.2f} ms, max={max(ordered)*1000:.2f} ms (host/USB round trip, not pin latency).', flush=True)
    counts={status:sum(r['status']==status for r in report.results) for status in ('PASS','FAIL','WARN','SKIP')}
    print('Results: ' + ', '.join(f'{value} {key}' for key,value in counts.items()), flush=True)
    if args.json:
        payload={'port':args.port,'seed':args.seed,'arguments':vars(args)|{'json':str(args.json)},'results':report.results,'cleanup':cleanup,'interrupted':interrupted,'reply_latencies_seconds':box.latencies if box else [],'serial_trace':box.trace if box else []}
        args.json.write_text(json.dumps(payload,indent=2)+'\n')
    return 130 if interrupted else report.exit_code()

if __name__ == '__main__':
    sys.exit(main())
