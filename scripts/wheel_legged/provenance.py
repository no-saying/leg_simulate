"""Hash extracted implementations as well as legacy entry-point wrappers."""

import hashlib
from pathlib import Path


def implementation_digest(*entrypoints):
    root = Path(__file__).resolve().parent
    paths = set(root.rglob('*.py')) | {Path(path).resolve() for path in entrypoints}
    digest = hashlib.sha256()
    for path in sorted(paths):
        digest.update(str(path.relative_to(root.parent)).encode())
        digest.update(b'\0')
        digest.update(path.read_bytes())
        digest.update(b'\0')
    return digest.hexdigest()
