"""Offline checks of the connected-device runner itself; no real port is opened.

A scripted peer deliberately returns fragmented, short, extra or absent replies.
A pseudo-terminal check exercises the actual POSIX transport independently of a
Pico. These tests validate the runner, not the firmware or hardware timings.

Run: python3 -m unittest discover -s TimingBoxPico/tests -p 'test_hardware_stress.py' -v
"""
import contextlib
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
sys.dont_write_bytecode = True
import hardware_stress as hw

class Peer:
    """Protocol peer with write-triggered replies, or an intentionally bad variant."""
    def __init__(self, mode='normal'):
        self.mode, self.pending, self.writes = mode, bytearray(), []
        self.buffer = bytearray()
        self.now = 100
        self.closed = False

    def write(self, data, timeout):
        self.writes.append(bytes(data)); self.buffer.extend(data)
        lengths={1:6,2:2,3:2,4:2,5:1,6:4,7:1,8:1,9:4,10:2,11:5,12:19,13:3,253:1,254:1,255:1}
        while self.buffer:
            op=self.buffer[0]; n=lengths.get(op,1)
            if len(self.buffer)<n: return
            packet=bytes(self.buffer[:n]);del self.buffer[:n]
            if op==253: response=b'\x06\x06'
            elif op in (5,8):
                self.now=(self.now+100)&hw.MASK;response=hw.u24(self.now)
            elif op==6: response=b'\x01'+hw.u24(self.now)
            elif op==10: response=bytes((packet[1],0))
            else: response=b''
            if self.mode=='silent': response=b''
            if self.mode=='extra' and response: response+=b'\xaa'
            if self.mode=='short' and response: response=response[:1]
            self.pending.extend(response)

    def read(self, count, timeout):
        # Fragment ALL replies into individual bytes to exercise read_exact.
        if self.pending:
            result=bytes(self.pending[:1]);del self.pending[:1];return result
        return b''

    def close(self): self.closed=True

class RunnerTests(unittest.TestCase):
    def test_fragmented_reply_without_wakeup_command(self):
        p=Peer();box=hw.Box(p,timeout=.01,quiet=0)
        self.assertEqual(box.version(),(6,6))
        self.assertEqual(p.writes,[b'\xfd'])

    def test_missing_reply_has_deadline_and_no_retry(self):
        p=Peer('silent');box=hw.Box(p,timeout=.001,quiet=0)
        with self.assertRaisesRegex(hw.TransportFailure,'no additional command'):
            box.version()
        self.assertEqual(p.writes,[b'\xfd'])

    def test_short_reply_is_not_treated_as_success(self):
        with self.assertRaises(hw.TransportFailure): hw.Box(Peer('short'),timeout=.001,quiet=0).version()

    def test_extra_reply_is_framing_failure(self):
        with self.assertRaisesRegex(hw.TransportFailure,'extra reply'):
            hw.Box(Peer('extra'),timeout=.01,quiet=0).version()

    def test_burst_packets_and_full_address_range(self):
        peer=Peer();box=hw.Box(peer,timeout=.01,quiet=0)
        box.upload([(16 if i%2 else 0,4+i) for i in range(256)],repeat=True,repeat_from=17,burst=True)
        burst=next(w for w in peer.writes if len(w)>100)
        self.assertEqual(len(burst),256*6+6)
        self.assertEqual(burst[:6],hw.entry(0,0,4))
        self.assertEqual(burst[-12:-6],hw.entry(255,16,259))
        self.assertEqual(burst[-6:],bytes((3,17,4,1,2,255)))

    def test_scheduled_target_wraps(self):
        peer=Peer();peer.now=0xFFFFFE-100
        box=hw.Box(peer,timeout=.01,quiet=0)
        target=box.schedule(10)
        self.assertEqual(target,(0xFFFFFE+hw.ticks(.01))&hw.MASK)
        self.assertEqual(peer.writes[-1],b'\x06'+hw.u24(target))

    def test_no_response_commands_do_not_read_ack(self):
        peer=Peer();hw.Box(peer).stop()
        self.assertEqual(peer.writes,[b'\x07'])
        self.assertFalse(peer.pending)

    def test_classification_skip_and_broken_session(self):
        report=hw.Reporter();ran=[]
        def broken(): raise hw.TransportFailure('bad framing')
        with contextlib.redirect_stdout(io.StringIO()):
            report.test('bad','transport',broken)
            report.test('later','later',lambda:ran.append(1))
        self.assertEqual([x['status'] for x in report.results],['FAIL','SKIP'])
        self.assertFalse(ran);self.assertEqual(report.exit_code(),1)

    def test_human_skip_and_timing_exit_policy(self):
        report=hw.Reporter()
        def skipped(): raise hw.ObservationSkipped('skip')
        def timing(): raise hw.TimingWarning('clock')
        with contextlib.redirect_stdout(io.StringIO()):
            report.test('visual','LED',skipped,human=True)
            report.test('clock','clock',timing)
        self.assertEqual([r['status'] for r in report.results],['SKIP','WARN'])
        self.assertEqual(report.exit_code(),0)
        report.strict=True;self.assertEqual(report.exit_code(),1)

    def test_user_prompts(self):
        with patch('builtins.input',return_value='y'): hw.ask('question')
        with patch('builtins.input',return_value='n'),self.assertRaises(hw.Failure): hw.ask('question')
        with patch('builtins.input',return_value='s'),self.assertRaises(hw.ObservationSkipped): hw.ask('question')

    def test_cleanup_programs_low_not_just_stop(self):
        p=Peer();hw.Box(p,timeout=.01,quiet=0).all_low()
        self.assertIn(hw.entry(0,0,hw.ticks(.01)),p.writes)
        self.assertEqual(p.writes[-1],b'\x07')

    def test_complete_runner_with_json_and_cleanup(self):
        peer=Peer()
        # No physical timing is claimed here; fake clock deliberately exercises
        # the timing-warning path while all transport cases execute normally.
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'report.json'
            with patch.object(hw,'SerialPort',return_value=peer),patch.object(hw.time,'sleep'),contextlib.redirect_stdout(io.StringIO()):
                code=hw.main(['--port','FAKE','--seconds','.001','--timeout','.01','--json',str(path)])
            report=json.loads(path.read_text())
        self.assertEqual(code,0)
        self.assertTrue(peer.closed)
        self.assertTrue(report['serial_trace'])
        self.assertTrue(all(r['status'] in ('PASS','WARN') for r in report['results']))
        self.assertIn('Explicit low',report['cleanup'])

    def test_interactive_cases_are_separate_observations(self):
        box=hw.Box(Peer(),timeout=.01,quiet=0);report=hw.Reporter()
        args=hw.parser().parse_args(['--port','FAKE'])
        with patch.object(hw.time,'sleep'),patch('builtins.input',side_effect=lambda text: '' if text.startswith('Press Return') else 'y') as prompt,contextlib.redirect_stdout(io.StringIO()):
            hw.human_tests(box,report,args,16)
        self.assertEqual(prompt.call_count,11)
        self.assertEqual(len(report.results),5)
        self.assertTrue(all(r['category']=='HUMAN' and r['status']=='PASS' for r in report.results))

    def test_human_actions_wait_for_readiness_and_restart_is_one_shot(self):
        events=[]
        class RecordingBox:
            def stop(self): events.append(('stop',))
            def all_low(self): events.append(('dark',))
            def upload(self,entries,**kw): events.append(('upload',entries,kw))
            def run(self): events.append(('run',))
            def schedule(self,lead): events.append(('schedule',lead))
        def input_for(prompt):
            if prompt.startswith('Press Return'):
                events.append(('ready',));return ''
            events.append(('answer',));return 'y'
        args=hw.parser().parse_args(['--port','FAKE']);report=hw.Reporter()
        with patch.object(hw.time,'sleep',side_effect=lambda n:events.append(('sleep',n))),patch('builtins.input',side_effect=input_for),contextlib.redirect_stdout(io.StringIO()):
            hw.human_tests(RecordingBox(),report,args,16)
        starts=[i for i,e in enumerate(events) if e[0]=='ready']
        self.assertEqual(len(starts),5)
        for start,end in zip([0]+starts[:-1],starts):
            # Each section prepares darkness before offering readiness.
            self.assertIn(('dark',),events[start:end])
            dark=max(i for i in range(start,end) if events[i][0]=='dark')
            self.assertFalse(any(e[0] in ('run','schedule') for e in events[dark+1:end]),'Observed action started before readiness')
        blink=events[starts[0]:starts[1]]
        self.assertIn(('schedule',500),blink)
        self.assertNotIn(('schedule',10),blink)
        self.assertIn(('sleep',5.5),blink)
        replacement=events[starts[2]:starts[3]]
        first_answer=next(i for i,e in enumerate(replacement) if e[0]=='answer')
        self.assertIn(('schedule',2000),replacement[:first_answer])
        self.assertIn(('schedule',6000),replacement[:first_answer])
        self.assertIn(('sleep',10),replacement[:first_answer])
        restart_upload=[e for e in events if e[0]=='upload'][-2]
        self.assertFalse(restart_upload[2]['repeat'])
        self.assertEqual(restart_upload[1],[(16,hw.ticks(4)),(0,hw.ticks(2))])
        restart=events[starts[3]:]
        restart=restart[:restart.index(('answer',))+1]
        self.assertEqual([e for e in restart if e[0]=='run'],[('run',),('run',)])
        self.assertIn(('sleep',3),restart)
        self.assertIn(('sleep',8),restart)
        self.assertNotIn(('stop',),restart,'Stopping after the viewing window would relight ON entry zero')
        final_upload=[e for e in events if e[0]=='upload'][-1]
        self.assertFalse(final_upload[2]['repeat'])
        self.assertEqual(final_upload[1],[(0,hw.ticks(1)),(16,1)])
        self.assertEqual(events[starts[4]+1:],[('run',),('sleep',4),('answer',)])

    def test_readiness_skip_wrong_start_and_eof(self):
        for answer,exception in [('s',hw.ObservationSkipped),('n',hw.Failure)]:
            with patch('builtins.input',return_value=answer),contextlib.redirect_stdout(io.StringIO()),self.assertRaises(exception):
                hw.ready_to_watch('instructions')
        with patch('builtins.input',side_effect=EOFError),contextlib.redirect_stdout(io.StringIO()),self.assertRaisesRegex(hw.Failure,'readiness'):
            hw.ready_to_watch('instructions')

    def test_io_error_aborts_following_cases(self):
        report=hw.Reporter()
        def disconnected(): raise OSError('device gone')
        with contextlib.redirect_stdout(io.StringIO()):
            report.test('io','transport',disconnected)
        self.assertTrue(report.broken)
        self.assertEqual(report.exit_code(),1)

    def test_port_chooser_sorts_and_requires_selection_even_for_one(self):
        with patch.object(hw.glob,'glob',return_value=['/dev/cu.usbserial-B','/dev/cu.usbserial-A']),patch('builtins.input',return_value='2'),contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(hw.choose_port(),'/dev/cu.usbserial-B')
        self.assertIn('1. /dev/cu.usbserial-A',output.getvalue())
        with patch.object(hw.glob,'glob',return_value=['/dev/cu.usbserial-A']),patch('builtins.input',return_value='1') as prompt,contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(hw.choose_port(),'/dev/cu.usbserial-A')
        prompt.assert_called_once()

    def test_port_chooser_reprompts_invalid_input(self):
        with patch.object(hw.glob,'glob',return_value=['/dev/cu.usbserial-A']),patch('builtins.input',side_effect=['','0','2','abc','1']),contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(hw.choose_port(),'/dev/cu.usbserial-A')

    def test_port_chooser_cancel_never_opens_device(self):
        with patch.object(hw.glob,'glob',return_value=['/dev/cu.usbserial-A']),patch('builtins.input',return_value='q'),patch.object(hw,'SerialPort') as port,contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(hw.main([]),0)
        port.assert_not_called()

    def test_missing_ports_and_eof_have_clear_errors(self):
        with patch.object(hw.glob,'glob',return_value=[]),self.assertRaisesRegex(ValueError,'No /dev/cu.usbserial'):
            hw.choose_port()
        with patch.object(hw.glob,'glob',return_value=['/dev/cu.usbserial-A']),patch('builtins.input',side_effect=EOFError),contextlib.redirect_stdout(io.StringIO()),self.assertRaisesRegex(ValueError,'--port for unattended'):
            hw.choose_port()

    def test_explicit_port_bypasses_chooser(self):
        with patch.object(hw,'choose_port') as choose,patch.object(hw,'SerialPort',side_effect=OSError('test connection')),contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(hw.main(['--port','EXPLICIT']),1)
        choose.assert_not_called()

    def test_list_ports_never_opens_device(self):
        with patch.object(hw,'SerialPort') as port,contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(hw.main(['--list-ports']),0)
        port.assert_not_called()

    @unittest.skipUnless(os.name=='posix','POSIX serial backend')
    def test_real_transport_through_pseudoterminal(self):
        import pty
        master,slave=pty.openpty();port=None
        try:
            port=hw.SerialPort(os.ttyname(slave))
            port.write(b'\xfd',.1)
            self.assertEqual(os.read(master,1),b'\xfd')
            os.write(master,b'\x06\x06')
            self.assertEqual(port.read(2,.1),b'\x06\x06')
            self.assertEqual(port.read(2,.001),b'')
        finally:
            if port: port.close()
            os.close(master);os.close(slave)

if __name__=='__main__': unittest.main()
