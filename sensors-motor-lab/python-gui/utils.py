import threading


def map_value(value, leftMin, leftMax, rightMin=0, rightMax=100):
    # Check extremum cases
    if value > leftMax:
        return rightMax
    if value < leftMin:
        return rightMin

    # Figure out how wide each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range
    return rightMin + (valueScaled * rightSpan)


class StoppableThread(threading.Thread):
    """Thread class with a stop() method. The thread itself has to check
    regularly for the stopped() condition."""

    def __init__(self, target=None, args=None, daemon=True):
        if args:
            super(StoppableThread, self).__init__(
                target=target, args=args, daemon=daemon
            )
        else:
            super(StoppableThread, self).__init__(target=target, daemon=daemon)

        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()
