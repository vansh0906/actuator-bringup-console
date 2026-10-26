"""Actuator bring-up and diagnostics console."""

from .devices import DeviceManager
from .params import PARAMS, ParamClient, ParamError
from .protocol import Codec, Frame, InjectKind, Mode, State
from .transport import LinkError, SlcanLink

__version__ = "1.0.0"


def connect(url: str = "tcp://localhost:29536", start: bool = True):
    """Open a link, start the command loop, and return (link, dm, pc)."""
    dm = DeviceManager()
    link = SlcanLink(url, dm.on_frame).open()
    dm.link = link
    if start:
        dm.start()
    return link, dm, ParamClient(dm)


__all__ = [
    "Codec", "DeviceManager", "Frame", "InjectKind", "LinkError", "Mode", "PARAMS", "ParamClient",
    "ParamError", "SlcanLink", "State", "connect",
]
