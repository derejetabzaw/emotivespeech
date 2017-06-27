#Author <Temaledegn>
#---Needs Testing---
import subprocess, os, signal

def terminate_process(p):
    ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=subprocess.PIPE)
    ps_output = ps_command.stdout.read()
    retcode = ps_command.wait()
    assert retcode == 0, "ps command returned %d" % retcode
    for pid_str in ps_output.split("\n")[:-1]:
        os.kill(int(pid_str), signal.SIGINT)
    p.terminate()


#pass the process to kill as a parameter to 'terminate_process' function
#this code is not tested, you are gonna have to check it.
# I dont remember what exactly the stop script have to do, 
#this code is for general process killing purpose, just pass it to terminator and it will be gone
