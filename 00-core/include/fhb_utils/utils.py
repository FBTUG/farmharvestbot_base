import subprocess, os, signal

class Consts():
    #ACTID_START group
    ACTID_START_HARVEST=100
    ACTID_START_CLI=200
    ACTID_START_VISION=300
    ACTID_START_ARM=400
    ACTID_START_CAR=500
    ACTID_START_PE=600
    ACTID_START_SIM=700
    
    #ACTID_START_HARVEST
    ACTID_HARVEST_GOAL = ACTID_START_HARVEST+1
    _actname = {ACTID_HARVEST_GOAL:"HarvestGoal"}

    # return "" if not defined, range in ACTID_START_HARVEST->ACTID_START_SIM+100
    def par_to_group(self,act_id):
        grp_name_def=["harvest","cli","vision","arm","car","pe","sim"]
        grp_name =""
        for i in range(7):
            if act_id>= (i+1)*100 and act_id<(i+2)*100:
                grp_name = grp_name_def[i]
        return grp_name
class MyUtils():
    def __init__(self):
        pass

    def terminate_process_and_children(p):
        ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=subprocess.PIPE)
        ps_output = ps_command.stdout.read()
        retcode = ps_command.wait()
        assert retcode == 0, "ps command returned %d" % retcode
        for pid_str in ps_output.split("\n")[:-1]:
                os.kill(int(pid_str), signal.SIGINT)
        p.terminate()

    def get_version(self):
        return "0.0.1"