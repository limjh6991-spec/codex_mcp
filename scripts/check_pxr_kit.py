import sys
try:
    import pxr
    from pxr import Usd, UsdPhysics
    print("OK pxr path:", pxr.__file__)
    print("Usd module:", Usd.__name__)
    print("UsdPhysics module:", UsdPhysics.__name__)
    sys.exit(0)
except Exception as e:
    print("IMPORT ERROR:", repr(e))
    sys.exit(1)
