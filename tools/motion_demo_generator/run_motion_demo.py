# Headless demo runner: drives HexaCore over the RDT wire (newline-delimited JSON on :30002).
# LOAD_FILE motion_types_demo.json (controller arms on load), then programCommand RUN, then
# monitors the broadcast status until the program finishes or faults.
import json, socket, sys, time, io

from pathlib import Path
_HERE = Path(__file__).resolve().parent
_PROJECT = _HERE.parents[1]   # <workspace>/HexaMotion (script lives in HexaMotion/tools/motion_demo_generator)
# Wire skeleton: a default-constructed ControlState dumped by dump_controlstate.cpp (see
# README.md). MUST be regenerated whenever kRdtProtocolVersion or the ControlState schema
# changes - the runner cross-checks the version against the live broadcast below.
SKELETON = json.load(io.open(_HERE / 'controlstate_default.json', encoding='utf-8'))

HOST, PORT = '127.0.0.1', 30002
import random
_base = random.randint(10000, 800000)
LOAD_REQ_ID = _base
RUN_REQ_ID = _base + 1

def send(sock, obj):
    sock.sendall((json.dumps(obj) + '\n').encode('utf-8'))

LAST_HEARTBEAT = {'req': 0}
# Version acks the server expects on every uplink: without them it re-sends the heavy
# config/program/trajectory payloads in EVERY broadcast until acked, and a slow reader's socket
# backlog explodes (observed: broadcasts stopped arriving after the demo upload).
ACKS = {'ackConfigVersion': 0, 'ackTrajVersion': 0, 'ackProgramVersion': 0, 'ackFileOpId': 0}

def note_status(msg):
    LAST_HEARTBEAT['req'] = (msg.get('sys') or {}).get('heartbeat_request', LAST_HEARTBEAT['req'])
    ACKS['ackConfigVersion'] = msg.get('configVersion', ACKS['ackConfigVersion'])
    ACKS['ackTrajVersion'] = msg.get('trajVersion', ACKS['ackTrajVersion'])
    ACKS['ackProgramVersion'] = msg.get('programVersion', ACKS['ackProgramVersion'])
    resp = msg.get('fileOpResponse') or {}
    ACKS['ackFileOpId'] = max(ACKS['ackFileOpId'], resp.get('processedFileOpId', 0))

# F2 contract: one-shot uplink fields live in a single overwritable slot on the controller and are
# sampled at the NRT tick — a command must RIDE EVERY snapshot until its req-id is consumed
# (the controller dedups by id). PERSIST carries the active command on every outgoing packet.
PERSIST = {}

def stamped(base):
    pkt = dict(base)
    pkt['heartbeat_response'] = LAST_HEARTBEAT['req']
    pkt.update(ACKS)
    pkt.update(PERSIST)
    return pkt

def keepalive(sock):
    # The server drops clients with no uplink traffic (RdtServer stale-client timeout) - the GUI
    # snapshots every 20 ms. An idle skeleton with current acks keeps us alive and stops the
    # heavy-payload resend loop.
    send(sock, stamped(SKELETON))

def statuses(sock, buf, timeout):
    sock.settimeout(timeout)
    try:
        chunk = sock.recv(65536)
    except socket.timeout:
        keepalive(sock)
        return buf, []
    if not chunk:
        raise ConnectionError('server closed the connection')
    buf += chunk.decode('utf-8', errors='replace')
    out = []
    while '\n' in buf:
        line, buf = buf.split('\n', 1)
        if line.strip():
            try:
                msg = json.loads(line)
                note_status(msg)
                out.append(msg)
            except json.JSONDecodeError:
                pass
    keepalive(sock)
    return buf, out

def main():
    sock = socket.create_connection((HOST, PORT), timeout=5)
    buf = ''

    # 1. First status: learn the schema field names we need.
    deadline = time.time() + 5
    first = None
    while time.time() < deadline and first is None:
        buf, msgs = statuses(sock, buf, 0.2)
        if msgs:
            first = msgs[-1]
    if first is None:
        sys.exit('no status broadcast received')
    live_ver = first.get('protocolVersion')
    if live_ver != SKELETON.get('protocolVersion'):
        sys.exit(f'protocol version mismatch: skeleton v{SKELETON.get("protocolVersion")} vs '
                 f'controller v{live_ver} - rebuild dump_controlstate.cpp and regenerate the '
                 'skeleton (README.md)')
    print('status keys:', sorted(first.keys()))
    for k in sorted(first.keys()):
        v = first[k]
        if isinstance(v, dict):
            print(f'  {k}: {sorted(v.keys())}')

    # 2. Upload the program directly (ControlState.newProgram) - the exact path the NG editor's
    # RUN uses; the controller arms the sequencer on upload. Since generator wire-format v2 the
    # demo file IS the controller envelope {formatVersion, program{name, steps}} - no conversion
    # (the old pendant->wire mapping block was deleted with the pendant format).
    doc = json.load(io.open(_PROJECT / 'programs' / 'motion_types_demo.json', encoding='utf-8'))
    if doc.get('formatVersion') != 2 or 'steps' not in (doc.get('program') or {}):
        sys.exit('demo file is not the wire-format v2 envelope; regenerate with generate_motion_demo.py')
    PERSIST['programUpdateReqId'] = LOAD_REQ_ID
    PERSIST['newProgram'] = doc['program']
    send(sock, stamped(SKELETON))
    print('sent program upload (req', LOAD_REQ_ID, ',', len(steps), 'steps)')

    loaded = False
    deadline = time.time() + 10
    while time.time() < deadline and not loaded:
        buf, msgs = statuses(sock, buf, 0.2)
        for m in msgs:
            if m.get('processedProgramReqId') == LOAD_REQ_ID:
                name = (m.get('loadedProgram') or {}).get('name')
                nsteps = len((m.get('loadedProgram') or {}).get('steps') or [])
                print('program accepted:', name, f'({nsteps} steps), programVersion', m.get('programVersion'))
                loaded = True
                break
    if not loaded:
        sys.exit('no upload confirmation within 10 s')

    # 2.5 Clear a latched error from any previous faulted run: while in Error the controller
    # ignores program commands entirely (recovery-only gate), exactly like the GUI CLEAR button.
    PERSIST.pop('newProgram', None)   # program confirmed; stop re-sending the heavy payload
    PERSIST['clearError'] = True
    t_end = time.time() + 1.5
    while time.time() < t_end:
        buf, _ = statuses(sock, buf, 0.2)
    PERSIST['clearError'] = False
    print('error-clear cycle done')

    # 3. RUN — rides every snapshot from here on (controller executes the req-id exactly once).
    time.sleep(0.3)
    PERSIST['programCommand'] = 1
    PERSIST['programCmdReqId'] = RUN_REQ_ID
    send(sock, stamped(SKELETON))
    print('sent RUN (req', RUN_REQ_ID, ')')

    # 4. Monitor to completion (status keys: prog{currentLine,isRunning,isPaused,programName},
    # sys{activeErrors,isEStop}).
    last_line = None
    was_running = False
    deadline = time.time() + 240
    while time.time() < deadline:
        buf, msgs = statuses(sock, buf, 0.2)
        for m in msgs:
            prog = m.get('prog') or {}
            sysd = m.get('sys') or {}
            running = prog.get('isRunning')
            row = prog.get('currentLine')
            msg_txt = sysd.get('activeErrors')
            if row != last_line:
                print(f'  t={time.strftime("%H:%M:%S")} running={running} line={row} sys={msg_txt}')
                last_line = row
            if running:
                was_running = True
            if was_running and running is False:
                final = 'FAULT' if (msg_txt and 'error' in str(msg_txt).lower()) else 'FINISHED'
                print(f'PROGRAM {final}: line={row} sys={msg_txt}')
                return
    print('TIMEOUT waiting for program completion')

main()
