from readerwriterlock import rwlock

class Bus:
    def __init__(self):
        self.message = None
        self.lock = rwlock.RWLockWriteD()

    def read(self):
        with self.lock.gen_rlock():
            message = self.message
        return message

    def write(self, message):
        with self.lock.gen_wlock():
            self.message = message