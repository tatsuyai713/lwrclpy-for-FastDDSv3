import sys
import os
import glob
import ctypes

def _python_xy():
    vi = sys.version_info
    return f"{vi.major}.{vi.minor}"

def _prepend_sys_path(path):
    norm = os.path.abspath(path)
    if norm not in sys.path:
        sys.path.insert(0, norm)

def _iter_fastdds_paths():
    explicit = os.environ.get("FASTDDS_PYTHON")
    if explicit:
        yield explicit
    prefixes = []
    env_prefix = os.environ.get("FASTDDS_PREFIX")
    if env_prefix:
        prefixes.append(env_prefix)
    prefixes.append("/opt/fast-dds-v3")
    pyxy = _python_xy()
    for prefix in prefixes:
        for sub in (f"{prefix}/lib/python{pyxy}/site-packages",
                    f"{prefix}/lib/python{pyxy}/dist-packages"):
            if os.path.isdir(sub):
                yield sub

def _preload_libs(paths):
    for p in paths:
        try:
            ctypes.CDLL(p, mode=getattr(ctypes, "RTLD_GLOBAL", os.RTLD_GLOBAL))
        except Exception:
            pass

def ensure_fastdds():
    try:
        import fastdds  # noqa: F401
        return
    except Exception:
        pass

    # Try to add known locations for fastdds bindings
    for base in _iter_fastdds_paths():
        fastdds_pkg = os.path.join(base, "fastdds")
        if os.path.isdir(fastdds_pkg):
            _prepend_sys_path(base)
            # Preload the extension library if present
            for ext in glob.glob(os.path.join(fastdds_pkg, "_fastdds_python.*")):
                _preload_libs([ext])
            break

    # Also add generated message root if present
    for gen_root in (os.environ.get("FASTDDS_TYPES_ROOT"), "/opt/fast-dds-v3-libs/python/src"):
        if gen_root and os.path.isdir(gen_root):
            _prepend_sys_path(gen_root)

    import fastdds  # noqa: F401
    return fastdds
