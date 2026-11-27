import curses
import subprocess
import time

class GazeboTerminalController:
    def __init__(self, model_name="yantra", world="empty"):
        self.model_name = model_name
        self.world = world

    def call_gz_service(self, args):
        try:
            result = subprocess.run(args, capture_output=True, text=True, timeout=2)
            if "true" in result.stdout.lower():
                return True
        except subprocess.TimeoutExpired:
            return False
        return False

    def pause_sim(self):
        return self.call_gz_service([
            "gz", "service", "-s", f"/world/{self.world}/control",
            "--reqtype", "gz.msgs.WorldControl",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req", "pause: true"
        ])

    def unpause_sim(self):
        return self.call_gz_service([
            "gz", "service", "-s", f"/world/{self.world}/control",
            "--reqtype", "gz.msgs.WorldControl",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req", "pause: false"
        ])

    def set_inital_pose(self, x=0, y=0.0, z=0.32):
        orientation = "{x: 0.0, y: 0.0, z: 0, w: 1.0}"
        return self.call_gz_service([
            "gz", "service", "-s", f"/world/{self.world}/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req", f'name: "{self.model_name}", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {orientation}'
        ])

    def run_ui(self):
        curses.wrapper(self._main)

    def _main(self, stdscr):
        stdscr.nodelay(True)
        curses.curs_set(0)
        stdscr.clear()

        stdscr.addstr(0, 0, "Gazebo Controller — [p] Pause  [u] Unpause  [s] Set Intial Pose  [q] Quit")
        stdscr.addstr(2, 0, "Status: Waiting for input...")

        while True:
            key = stdscr.getch()

            if key == ord('p'):
                success = self.pause_sim()
                self._update_status(stdscr, "Paused simulation", success)
            elif key == ord('u'):
                success = self.unpause_sim()
                self._update_status(stdscr, "Unpaused simulation", success)
            elif key == ord('s'):
                success = self.set_inital_pose()
                self._update_status(stdscr, "Set initial pose", success)
            elif key == ord('q'):
                self._update_status(stdscr, "Exiting...", True)
                time.sleep(0.5)
                break

            time.sleep(0.1)

    def _update_status(self, stdscr, action, success):
        msg = f"{action} — {'✅ Success' if success else '❌ Failed'}"
        stdscr.addstr(2, 0, " " * 80)  # Clear old line
        stdscr.addstr(2, 0, f"Status: {msg}")
        stdscr.refresh()

if __name__ == "__main__":
    controller = GazeboTerminalController(model_name="yantra", world="empty")
    controller.run_ui()
