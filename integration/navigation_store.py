"""Single local writer per durable interlock store (Linux/macOS advisory lock)."""
import fcntl
from pathlib import Path


class StoreOwner:
    def __init__(self,path):
        path=Path(str(path)+'.owner.lock');path.parent.mkdir(parents=True,exist_ok=True)
        self.file=path.open('a')
        try:fcntl.flock(self.file,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except OSError:
            self.file.close()
            raise ValueError('Another interlock owns this durable store')
        self.closed=False

    def close(self):
        self.closed=True;self.file.close()

    def __del__(self):
        file=getattr(self,'file',None)
        if file is not None and not file.closed:file.close()
