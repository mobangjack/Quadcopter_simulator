import threading
import time

class Arbiter():
    def __init__(self, get_time, output, channels=1, tolerance=1):
        self.get_time = get_time
        self.channels = channels
        self.data = [None] * channels
        self.lost_counters = [0] * channels
        self.run = True
        self.cursor = 0
        self.tolerance = tolerance
        self.output = output

    def feed(self, index, value):
        self.data[index] = value
        self.lost_counters[index] = 0

    def is_channel_alive(self, index):
        return self.lost_counters[index] <= self.tolerance

    def update_lost_counter(self):
        for i in range(self.channels):
            if self.lost_counters[i] <= self.tolerance:
                self.lost_counters[i] = self.lost_counters[i] + 1

    def update_cursor(self):
        for i in range(0, self.channels):
            index = (self.cursor + i) % self.channels
            if self.is_channel_alive(index):
                if index != self.cursor:
                    print("channel[%d] is up" % (index))
                    self.cursor = index
                return True
            else:
                print("channel[%d] is down" % (index))
        return False

    def set_output(self):
        self.output(self.data[self.cursor])

    def update(self):
        if self.update_cursor():
            self.set_output()
        self.update_lost_counter()

    def thread_run(self,update_rate,time_scaling):
        update_rate = update_rate*time_scaling
        last_update = self.get_time()
        while(self.run==True):
            time.sleep(0)
            self.time = self.get_time()
            if (self.time - last_update).total_seconds() > update_rate:
                self.update()
                last_update = self.time

    def start_thread(self,update_rate=0.005,time_scaling=1):
        self.thread_object = threading.Thread(target=self.thread_run,args=(update_rate,time_scaling))
        self.thread_object.start()

    def stop_thread(self):
        self.run = False