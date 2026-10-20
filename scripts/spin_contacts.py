"""Training-only, nonmutating contact-force/velocity query at policy rate."""

import ctypes
import hashlib
import subprocess
import tempfile
from pathlib import Path

import mujoco
import numpy as np


class SpinContacts:
    def __init__(self, model, count, threads):
        source = Path(__file__).with_name("spin_contact_bridge.c")
        installation = Path(mujoco.__file__).parent
        digest = hashlib.sha256(source.read_bytes()+mujoco.__version__.encode()
                                + str(installation).encode()).hexdigest()[:16]
        folder = source.parent.parent/"logs/rl_training"/f"contacts_{digest}"
        folder.mkdir(parents=True, exist_ok=True)
        library = folder/"contacts.so"
        if not library.exists():
            with tempfile.TemporaryDirectory(dir=folder) as staging:
                output = Path(staging)/"contacts.so"
                subprocess.run(["gcc", "-O2", "-Wall", "-Wextra", "-fPIC", "-shared", "-fopenmp",
                                str(source), "-I"+str(installation/"include"),
                                str(next(installation.glob("libmujoco.so.*"))),
                                "-Wl,-rpath,"+str(installation), "-lm", "-o", str(output)], check=True)
                output.replace(library)
        self.lib = ctypes.CDLL(str(library))
        ptr = ctypes.c_void_p
        self.lib.spin_contacts_create.argtypes = [ptr, ctypes.c_int, ctypes.c_int]
        self.lib.spin_contacts_create.restype = ptr
        self.lib.spin_contacts_query.argtypes = [ptr, ptr, ptr]
        self.lib.spin_contacts_query.restype = None
        self.lib.spin_contacts_destroy.argtypes = [ptr]
        self.lib.spin_contacts_destroy.restype = None
        self.model, self.count = model, count
        self.context = self.lib.spin_contacts_create(model._address, count, threads)
        if not self.context:
            raise RuntimeError("Cannot allocate training contact query")
        self.values = np.zeros((count, 2, 4), dtype=np.float64)

    def query(self, states):
        states = np.ascontiguousarray(states, dtype=np.float64)
        expected = (self.count, mujoco.mj_stateSize(self.model, mujoco.mjtState.mjSTATE_INTEGRATION))
        if states.shape != expected:
            raise ValueError("Contact query integration state shape mismatch")
        self.lib.spin_contacts_query(self.context, states.ctypes.data, self.values.ctypes.data)
        return self.values

    def close(self):
        if self.context:
            self.lib.spin_contacts_destroy(self.context)
            self.context = None

    def __del__(self):
        if getattr(self, "context", None):
            self.close()
