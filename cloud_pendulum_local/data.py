"""Data types that mirror the ones of the cloud client, so that code written
against the cloud interface (e.g. the gym tutorial) runs unchanged locally."""
from enum import IntEnum


class CellType(IntEnum):
    """Type of a cell that can be reserved with `Client.start_gym`.

    A local setup only has a single physical cell (whatever is connected to the
    CAN bus), see `Client.start_gym` for how reservations are emulated.
    """
    SINGLE = 1
    DOUBLE = 2
    MOTOR = 3
    COUPLEDOSCILATOR = 4
