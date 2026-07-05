import json, csv
from pathlib import Path
from datetime import datetime

def ensure_dir(path): Path(path).mkdir(parents=True, exist_ok=True)
def load_json(path, default=None):
    try:
        return json.loads(Path(path).read_text(encoding='utf-8'))
    except Exception:
        return default or {}
def save_json(path, data):
    ensure_dir(Path(path).parent); Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
def timestamp_name(prefix, suffix): return f"{prefix}_{datetime.now().strftime('%Y%m%d_%H%M%S')}{suffix}"
def save_csv(path, rows, headers):
    ensure_dir(Path(path).parent)
    with open(path,'w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=headers); w.writeheader(); w.writerows(rows)
