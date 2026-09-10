"""Thin ABI binding. Physics stays in the C++ runtime; calls are serialized by caller."""
import ctypes
import json
import os
import time
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
LIB = Path(os.environ['APEXLAB_RUNTIME']) if os.environ.get('APEXLAB_RUNTIME') else next((ROOT / 'build/apps/sim-server').glob('*apexlab_runtime.*'))
lib = ctypes.CDLL(str(LIB))
lib.apex_create.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
lib.apex_create.restype = ctypes.c_void_p
lib.apex_destroy.argtypes = [ctypes.c_void_p]
lib.apex_command.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
lib.apex_advance.argtypes = [ctypes.c_void_p, ctypes.c_double]
lib.apex_snapshot.argtypes = [ctypes.c_void_p, ctypes.c_int]
lib.apex_snapshot.restype = ctypes.c_char_p
lib.apex_error.argtypes = [ctypes.c_void_p]
lib.apex_error.restype = ctypes.c_char_p
lib.apex_rollout.argtypes = [ctypes.c_void_p,ctypes.c_char_p,ctypes.c_int]
lib.apex_rollout.restype = ctypes.c_char_p
class Native:
    def __init__(self, vehicle='mclaren-p1', track='spa-francorchamps'):
        course = 'spa-francorchamps-dynamic' if track == 'spa-francorchamps' else track
        self.ptr = lib.apex_create(str(ROOT / f'configs/vehicles/{vehicle}-dynamic.json').encode(), str(ROOT / f'configs/tracks/{course}.json').encode())
        if not self.ptr: raise RuntimeError(lib.apex_error(None).decode())
    def check(self, ok):
        if not ok: raise RuntimeError(lib.apex_error(self.ptr).decode())
    def command(self, **values): self.check(lib.apex_command(self.ptr, json.dumps(values,allow_nan=False).encode()))
    def advance(self, dt): self.check(lib.apex_advance(self.ptr, dt))
    def snapshot(self, metadata=False):
        raw = lib.apex_snapshot(self.ptr, int(metadata)); self.check(raw)
        return json.loads(raw)
    def rollout(self, request, replay=False):
        start=time.perf_counter();payload=json.dumps(request,allow_nan=False).encode();encoded=time.perf_counter()
        raw=lib.apex_rollout(self.ptr,payload,int(replay));self.check(raw);returned=time.perf_counter()
        result=json.loads(raw);parsed=time.perf_counter()
        if request.get('profileTiming'):
            result['binding_timing']={'encode_s':encoded-start,'native_and_serialization_s':returned-encoded,'decode_s':parsed-returned,'total_s':parsed-start,'ipc_s':0}
        return result
    def close(self):
        if self.ptr: lib.apex_destroy(self.ptr); self.ptr = None
    def __enter__(self): return self
    def __exit__(self, *args): self.close()
