"""Import vacuula_server submodules without running the package __init__.

The real __init__ configures logging into the data dir and imports the whole server; the tests only need the
module under test (and, for the glue tests, state/conn).
"""
import sys
import types
from pathlib import Path

SRC = Path(__file__).resolve().parents[1] / "src"

if "vacuula_server" not in sys.modules:
    pkg = types.ModuleType("vacuula_server")
    pkg.__path__ = [str(SRC / "vacuula_server")]
    sys.modules["vacuula_server"] = pkg
