from datetime import datetime
class AppLogger:
    def __init__(self): self.records=[]; self.listeners=[]
    def add_listener(self, cb): self.listeners.append(cb)
    def log(self, op, msg, **kw):
        rec={"time":datetime.now().strftime('%Y-%m-%d %H:%M:%S'),"operation":op,"message":msg,**kw}
        self.records.append(rec)
        line=f"[{rec['time']}] {op}：{msg}"
        for cb in self.listeners: cb(line)
        return rec
