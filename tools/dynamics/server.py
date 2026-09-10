#!/usr/bin/env python3
"""Loopback native sim host: fixed physics clock, bounded input age, HTTP snapshots."""
import argparse
import json
import threading
import subprocess
import math
import gzip
import time
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs
from native import Native, ROOT
VEHICLES = {'mclaren-p1', 'mcl36'}
TRACKS = {'spa-francorchamps','monza','silverstone','suzuka','technical_test_circuit'}
OBJECTIVES = {'fastest','shortest','clearance','neutral','platform','fuel','wear','custom'}
def provenance(vehicle,track):
    vehicle_data=json.loads((ROOT/f'configs/vehicles/{vehicle}-dynamic.json').read_text())
    course='spa-francorchamps-dynamic' if track=='spa-francorchamps' else track
    track_data=json.loads((ROOT/f'configs/tracks/{course}.json').read_text())
    return {'vehicle':{k:vehicle_data.get(k) for k in ['name','parameter_classes','sources','assumptions']},
            'track':{k:track_data.get(k) for k in ['kind','provenance','source','elevation','course_selection','coordinate_system']}}
class ResumedJob:
    """Keep an already-running solver observable across host-only restarts."""
    def __init__(self,pid,path):self.pid=pid;self.path=path;self.returncode=None
    def poll(self):
        command=subprocess.run(['ps','-p',str(self.pid),'-o','command='],capture_output=True,text=True).stdout
        if str(ROOT/'tools/dynamics/optimize.py') in command:return None
        progress=json.loads(self.path.read_text()) if self.path.exists() else {}
        self.returncode=0 if progress.get('status')=='accepted' else 1
        return self.returncode

class Host:
    def __init__(self):
        self.lock=threading.Lock();self.native=Native();self.vehicle='mclaren-p1';self.track='spa-francorchamps'
        self.meta=self.native.snapshot(True);self.state=self.native.snapshot();self.running=False;self.error='';self.last_input=0.;self.costs=[];self.lag=0.;self.job=None;self.job_path=None
        self.meta['provenance']=provenance(self.vehicle,self.track)
        self.owner=None
    def configure(self,j):
        owner=j.get('clientId')
        if not isinstance(owner,str) or not 1<=len(owner)<=128:raise ValueError('A browser session identifier is required')
        vehicle=j.get('vehicle',self.vehicle);track=j.get('track',self.track)
        if vehicle not in VEHICLES or track not in TRACKS: raise ValueError('Unknown vehicle or track')
        replacement=Native(vehicle,track)
        self.native.close();self.native=replacement;self.owner=owner;self.vehicle=vehicle;self.track=track
        self.meta=self.native.snapshot(True);self.running=False;self.error='';self.state=self.native.snapshot()
        self.meta['provenance']=provenance(self.vehicle,self.track)
    def require_owner(self,client):
        if not self.owner or client!=self.owner:raise PermissionError('Session controlled by another browser. Reload or select a session to take control.')
    def loop(self):
        target=time.perf_counter()
        while True:
            target+=.01
            with self.lock:
                now=time.perf_counter()
                if self.running:
                    try:
                        if now-self.last_input>.35:self.native.command(throttle=0,brake=0,steering=0,steerKey=0,drs=False)
                        start=time.perf_counter();self.native.advance(.01)
                        self.costs.append(time.perf_counter()-start);self.costs=self.costs[-1000:]
                        self.state=self.native.snapshot()
                    except Exception as e:self.error=str(e);self.running=False
                self.lag=max(0.,now-target)
            # Do not skip physics: if overloaded, expose wall lag and slow down explicitly.
            if self.lag>.05:target=time.perf_counter()
            time.sleep(max(0.,target-time.perf_counter()))
    def report(self):
        costs=sorted(self.costs)
        return {**self.state,'running':self.running,'error':self.error,'inputStale':time.perf_counter()-self.last_input>.35,
          'performance':{'physicsHz':500,'publishHz':100,'stepBatchMeanMs':sum(costs)/max(1,len(costs))*1000,
          'stepBatchP95Ms':costs[min(len(costs)-1,int(len(costs)*.95))]*1000 if costs else 0,'wallLagMs':self.lag*1000}}
host=None
class Handler(BaseHTTPRequestHandler):
    def log_message(self,*args):pass
    def allowed(self):
        origin=self.headers.get('Origin')
        return origin is None or origin in {'http://127.0.0.1:5173','http://localhost:5173','http://127.0.0.1:4173'}
    def send(self,value,status=200):
        raw=json.dumps(value,allow_nan=False,separators=(',',':')).encode();compressed=len(raw)>100000 and 'gzip' in self.headers.get('Accept-Encoding','')
        if compressed:raw=gzip.compress(raw,compresslevel=1)
        self.send_response(status)
        if self.allowed() and self.headers.get('Origin'):self.send_header('Access-Control-Allow-Origin',self.headers['Origin'])
        if compressed:self.send_header('Content-Encoding','gzip')
        self.send_header('Content-Type','application/json');self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(raw)));self.end_headers()
        try:self.wfile.write(raw)
        except (BrokenPipeError,ConnectionResetError):pass # Client navigated away during a large replay response.
    def do_OPTIONS(self):
        if not self.allowed():return self.send({'error':'Origin not allowed'},403)
        self.send_response(204);self.send_header('Access-Control-Allow-Origin',self.headers.get('Origin','http://127.0.0.1:5173'));self.send_header('Access-Control-Allow-Methods','GET, POST');self.send_header('Access-Control-Allow-Headers','Content-Type');self.send_header('Access-Control-Max-Age','600');self.end_headers()
    def do_GET(self):
        if not self.allowed():return self.send({'error':'Origin not allowed'},403)
        path=urlparse(self.path).path
        # Capture shared state under the lock; disk I/O, compression and HTTP
        # transmission must never stall the 500 Hz simulator.
        with host.lock:
            if path=='/state':value=host.report()
            elif path=='/metadata':value=host.meta
            elif path=='/optimization':
                job_path=host.job_path
                active=host.job is not None and host.job.poll() is None
                failed=host.job is not None and not active and host.job.returncode
            elif path in {'/ghost','/reference'}:vehicle,track=host.vehicle,host.track
            else:return self.send({'error':'Not found'},404)
        if path in {'/state','/metadata'}:return self.send(value)
        if path=='/optimization':
            progress=json.loads(job_path.read_text()) if job_path and job_path.exists() else {}
            if failed and progress.get('status') in {'starting','running'}:progress['status']='failed; inspect optimizer log'
            return self.send({'running':active,**progress})
        suffix='-reference' if path=='/reference' else ''
        objective=parse_qs(urlparse(self.path).query).get('objective',['fastest'])[0]
        if objective not in OBJECTIVES:return self.send({'error':'Unknown objective'},400)
        p=ROOT/f'data/generated/dynamics/{track}-{vehicle}-{objective}{suffix}.json'
        if not p.exists() and track=='technical_test_circuit':p=ROOT/f'data/generated/dynamics/study/{vehicle}-{objective}{suffix}.json'
        if path=='/reference':
            policy_path=p.with_name(p.name.replace('-reference.json','.json'))
            if policy_path.exists():
                policy=json.loads(policy_path.read_text())
                artifact=policy.get('reference_artifact')
                if artifact and Path(artifact).name==artifact:p=p.parent/artifact
        if not p.exists():return self.send({'error':'No validated dynamic-model policy ghost available'},404)
        j=json.loads(p.read_text())
        if (path=='/ghost' and not j.get('accepted')) or j.get('model')!='dynamic_contact':return self.send({'error':'Ghost failed validation'},409)
        payload={k:v for k,v in j.items() if k not in {'commands','evaluations','profile'}}
        payload['samples']=j['samples'][::2]
        if payload['samples'][-1]!=j['samples'][-1]:payload['samples'].append(j['samples'][-1])
        return self.send(payload)
    def do_POST(self):
        if not self.allowed():return self.send({'error':'Origin not allowed'},403)
        try:
            length=int(self.headers.get('Content-Length','0'))
            if length<1 or length>4096:raise ValueError('Invalid command size')
            j=json.loads(self.rfile.read(length))
            if not isinstance(j,dict):raise ValueError('Command must be object')
            with host.lock:
                if self.path in {'/command','/optimize'}:host.require_owner(j.pop('clientId',None))
                if self.path=='/optimize':
                    if host.job is not None and host.job.poll() is None:raise ValueError('Optimization already running')
                    objective=j.get('objective','fastest')
                    if objective not in OBJECTIVES:raise ValueError('Unknown objective')
                    nodes=int(j.get('nodes',4))
                    if not 4<=nodes<=16:raise ValueError('Nodes outside 4–16')
                    args=[str(ROOT/'.cache/optimizer-venv/bin/python'),str(ROOT/'tools/dynamics/optimize.py'),'--vehicle',host.vehicle,'--track',host.track,'--objective',objective,'--nodes',str(nodes)]
                    if objective=='custom':
                        weights=j.get('weights',[])
                        if len(weights)!=7 or any(not isinstance(w,(int,float)) or not math.isfinite(w) or w<0 for w in weights) or sum(weights)<=0:raise ValueError('Invalid objective weights')
                        args+=['--weights',*map(str,weights)]
                    host.job_path=ROOT/f'data/generated/dynamics/{host.track}-{host.vehicle}-{objective}.progress.json'
                    host.job_path.write_text(json.dumps({'status':'starting'}))
                    logfile=host.job_path.with_suffix('.log')
                    with logfile.open('w') as log:host.job=subprocess.Popen(args,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
                    return self.send({'status':'started'})
                elif self.path=='/configure':host.configure(j)
                elif self.path=='/command':
                    if 'running' in j:host.running=bool(j.pop('running'))
                    # No network teleport, speed injection or arbitrary config paths.
                    allowed={'reset','throttle','brake','steering','steeringRate','saturation','deployment','drs','aeroMode','assisted','keyboardFriendly','steerKey','inputRise','inputReturn','throttleRise','throttleRelease','brakeRise','brakeRelease'}
                    if set(j)-allowed:raise ValueError('Unsupported input')
                    host.native.command(**j);host.last_input=time.perf_counter();host.error=''
                    host.state=host.native.snapshot()
                else:return self.send({'error':'Not found'},404)
                self.send(host.report())
        except PermissionError as e:self.send({'error':str(e)},409)
        except (ValueError,RuntimeError,KeyError) as e:self.send({'error':str(e)},400)
def main():
    global host
    parser=argparse.ArgumentParser();parser.add_argument('--port',type=int,default=8765);parser.add_argument('--resume-job',type=int);parser.add_argument('--resume-progress',type=Path);args=parser.parse_args()
    host=Host()
    if args.resume_job and args.resume_progress:host.job_path=args.resume_progress;host.job=ResumedJob(args.resume_job,args.resume_progress)
    threading.Thread(target=host.loop,daemon=True).start()
    print(f'Native 500 Hz DRIVE server: http://127.0.0.1:{args.port}',flush=True)
    ThreadingHTTPServer(('127.0.0.1',args.port),Handler).serve_forever()
if __name__=='__main__':main()
