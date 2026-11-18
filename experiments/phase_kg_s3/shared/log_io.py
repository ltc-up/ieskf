"""Transparent byte-preserving reads of original, gzip or XZ research logs."""
import gzip,lzma
from pathlib import Path


def candidates(path):
    path=Path(path)
    base=path.with_suffix('') if path.suffix in ('.gz','.xz') else path
    return list(dict.fromkeys([path,base,base.with_name(base.name+'.gz'),base.with_name(base.name+'.xz')]))


def log_path(path):
    return next((p for p in candidates(path) if p.exists()),Path(path))


def open_text(path):
    for p in candidates(path):
        try:
            if p.suffix=='.gz':return gzip.open(p,'rt',encoding='utf-8')
            if p.suffix=='.xz':return lzma.open(p,'rt',encoding='utf-8')
            return p.open(encoding='utf-8')
        except FileNotFoundError:continue
    raise FileNotFoundError(str(path))
