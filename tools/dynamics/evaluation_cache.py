"""Exact deterministic evaluation cache; configuration and native binary are part of identity."""
import hashlib
import json
import os
from pathlib import Path

class EvaluationCache:
    def __init__(self, directory: Path, context):
        self.directory = directory
        self.context = context
        self.hits = 0
        self.misses = 0
        self.memory = {}
    def key(self, candidate):
        payload = json.dumps([self.context, candidate], sort_keys=True, separators=(',', ':'), allow_nan=False)
        return hashlib.sha256(payload.encode()).hexdigest()
    def evaluate(self, candidate, compute):
        key = self.key(candidate)
        if key in self.memory:
            self.hits += 1
            return self.memory[key]
        path = self.directory / (key + '.json')
        if path.exists():
            try:
                value = json.loads(path.read_text())
                if value['key'] == key:
                    self.hits += 1
                    self.memory[key] = value['result']
                    return value['result']
            except (ValueError, KeyError):
                pass
        self.misses += 1
        result = compute()
        self.directory.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(f'.{os.getpid()}.tmp')
        temporary.write_text(json.dumps({'key': key, 'result': result}, allow_nan=False))
        temporary.replace(path)
        self.memory[key] = result
        return result

def configuration_identity(root, library):
    # Includes inherited tire/chassis configs, track context and runtime binary.
    files = sorted((root/'configs').rglob('*.json')) + [library]
    return {str(p.relative_to(root)) if p.is_relative_to(root) else str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
