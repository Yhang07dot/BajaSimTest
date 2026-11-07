"""Own the Gazebo process and stop it through its service before sending signals."""
import os
import signal
import subprocess
import sys
import time


def main():
    stopping=False

    def stop(_signum,_frame):
        nonlocal stopping
        stopping=True

    signal.signal(signal.SIGINT,stop)
    signal.signal(signal.SIGTERM,stop)
    process=subprocess.Popen(sys.argv[1:],start_new_session=True)
    while process.poll() is None and not stopping:time.sleep(0.1)
    if not stopping:return process.wait()
    # ServerControl also allows the rendering thread to finish its update.
    try:
        subprocess.run(['gz','service','-s','/server_control','--reqtype','gz.msgs.ServerControl',
                        '--reptype','gz.msgs.Boolean','--timeout','2000','--req','stop: true'],
                       timeout=3,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        for sig,seconds in [(signal.SIGTERM,2),(signal.SIGKILL,2)]:
            if process.poll() is not None:break
            os.killpg(process.pid,sig)
            try:process.wait(timeout=seconds)
            except subprocess.TimeoutExpired:continue
    return 0


if __name__=='__main__':raise SystemExit(main())
