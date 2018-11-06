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
    def get_version(self):
        return "0.0.1"