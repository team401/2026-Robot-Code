from dataclasses import dataclass
import subprocess
import threading
from queue import Queue, Empty
from typing import Callable
from data_manager import DataManager

# TODO: maybe keep a terminal open so that it might be more performant

@dataclass
class StdoutPollResult:
    lines: list[str]
    done: bool

class ProcessStdoutReader:

    def __init__(self, process: subprocess.Popen, stdout_queue: Queue):
        self.process = process
        self.stdout_queue = stdout_queue
        self.thread = None
        self.stop_event = threading.Event()

    def _read_stdout(self):
        while not self.stop_event.is_set():
            line = self.process.stdout.readline()
            if line:
                self.stdout_queue.put(line)
            elif self.process.poll() is not None:
                break
        # Capture current output and queue it before signaling completion
        current_output = self.process.stdout.readlines()
        for line in current_output:
            self.stdout_queue.put(line)
        self.stdout_queue.put(None)  # signal completion

    def start(self):
        self.thread: threading.Thread = threading.Thread(target=self._read_stdout, daemon=True)
        self.thread.start()

    def stop(self):
        if self.thread and self.thread.is_alive():
            self.stop_event.set()
            self.thread.join()
            self.stop_event.clear()
            self.thread = None

    def poll_output(self) -> StdoutPollResult:
        lines = []
        while True:
            try:
                item = self.stdout_queue.get_nowait()
            except Empty:
                break

            if item is None:
                return StdoutPollResult(lines, True)  # done
            lines.append(item)
        return StdoutPollResult(lines, False)  # not done yet

class ProcessHandler:

    def __init__(self, command_supplier: Callable[[], str]):
        self.command_supplier = command_supplier
        self.process = None
        self.stdout_queue = Queue()
        self.stdout_reader = None

    def start(self):
        if self.process is not None:
            return
        
        command = self.command_supplier()

        

        self.process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            stdin=subprocess.PIPE,
            text=True,
            shell=True,
            bufsize=1,
        )
        
        self.stdout_reader = ProcessStdoutReader(self.process, self.stdout_queue)
        self.stdout_reader.stdout_queue.put("Simulator started with command: " + command + "\n")
        self.stdout_reader.start()


    def stop(self):
        if self.process is None:
            if self.stdout_reader:
                self.stdout_reader.stop()
                self.stdout_reader = None
            return
        self.process.terminate()
        self.process.wait()
        self.process = None
        self.stdout_reader.stop()
        self.stdout_reader = None

    def is_running(self) -> bool:
        if self.process is None:
            return False
        polled_status = self.process.poll()
        if polled_status is not None:
            self.stdout_queue.put(f"Simulator process exited with code {polled_status}\n")
            self.process = None
        return polled_status is None  # None means still running

    def poll_output(self) -> StdoutPollResult:
        if self.stdout_reader is None:
            return StdoutPollResult([], True)  # no process, so "done" with empty output
        return self.stdout_reader.poll_output()



class SimManager:

    def __init__(self, data_manager: DataManager, simulator_script_supplier: Callable[[], str]):
        self.data_manager = data_manager
        self.process_handler = ProcessHandler(simulator_script_supplier)

    def start_simulator(self):
        self.process_handler.start()

    def stop_simulator(self):
        self.process_handler.stop()

    def is_simulator_running(self) -> bool:
        return self.process_handler.is_running()

    def get_simulator_output(self) -> list[str]:
        poll_result = self.process_handler.poll_output()
        return poll_result.lines