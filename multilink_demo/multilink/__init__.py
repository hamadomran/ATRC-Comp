"""Multi-link video for the EDGE demo: two streams per frame, FEC across links."""
from .codec import Receiver, Sender
from .links import Link

__all__ = ["Link", "Receiver", "Sender"]
