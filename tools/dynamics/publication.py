"""Keep the last accepted policy readable throughout a new solve and replay audit."""
import hashlib
import json
from pathlib import Path

def save(path,value):
    temporary=path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
    temporary.replace(path)

def publish_accepted(path:Path,result,reference):
    if not result.get('accepted'):raise ValueError('Only accepted policies can be published')
    identity={k:v for k,v in reference.items() if k!='wall_s'}
    digest=hashlib.sha256(json.dumps(identity,sort_keys=True,allow_nan=False).encode()).hexdigest()[:16]
    reference_path=path.with_name(f'{path.stem}-reference-{digest}.json')
    if not reference_path.exists():save(reference_path,reference)
    result={**result,'reference_artifact':reference_path.name}
    save(path,result) # One atomic pointer change publishes a policy/reference pair.
    save(path.with_name(path.stem+'-reference.json'),reference) # Legacy offline tooling.
    return result
