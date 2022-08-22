import argparse
from cProfile import run
from objdict import ObjDict
import json

INTERFACE_TYPE = ["VR","2D"]
RUN_TYPE = ["Manipulation", "Navigation"]
ARENA_CONFIG = ["1;2;3", "3;1;2", "2;3;1"]
SA_CONFIG = ["1,2,3;4,1,3;2,4,1", 
             "2,3,4;1,2,4;3,1,2",
             "3,4,1;2,3,1;4,2,3",
             "4,1,2;3,4,2;1,3,4"]
SA_TIME_CONFIG = ["90;135;190",
                  "190;90;135",
                  "135;190;90"]


parser = argparse.ArgumentParser(description="The following parameters are used in this file: ",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)

parser.add_argument("-pc", "--participant_count",  type=int, help="Number of participants to be recruited",  required=True)

args = parser.parse_args()

NUM_PARTICIPANTS = args.participant_count

data = ObjDict()
config = []


for i in range(0, NUM_PARTICIPANTS):
    participant_id = i + 1
    interface = INTERFACE_TYPE[i % 2]
    run_order = []
    run_order.append(RUN_TYPE[(i // (len(INTERFACE_TYPE))) % 2])
    run_order.append(RUN_TYPE[((i // (len(INTERFACE_TYPE))) + 1) % 2])
    arena = ARENA_CONFIG[(i // (len(INTERFACE_TYPE) * len(RUN_TYPE))) % 3]
    sa = SA_CONFIG[i%4]
    print("Participant: " + str(participant_id) + " Interface: " + interface + " Run Type: " + str(run_order) + " Arena config: " + arena + " SA Config: " + sa)
    
    json_entry = ObjDict()
    json_entry['id'] = str(participant_id)
    json_entry['interface'] = interface

    arena_config = arena.split(";")
    sa_config = sa.split(";")
    run_config = []
    raw_config = ''
    for i in range (0, 6):
        if i != 0:
            raw_config += ';'
        run = ObjDict()
        run['id'] = i + 1
        run['type'] = run_order[i//3]
        run['arena_config'] = arena_config[i%3]
        raw_config += run_order[i//3][0] + arena_config[i%3]
        if run_order[i//3] == 'Navigation':
            current_sa = sa_config[i%3].split(",")
            question_config = []
            for j in range (0,3):
                question = ObjDict()
                question['id'] = j+1
                question['index'] = int(current_sa[j])
                question['time'] = 1
                question_config.append(question)
            run['question'] = question_config

        run_config.append(run)

    json_entry['run'] = run_config
    json_entry['raw_config'] = raw_config


    config.append(json_entry)


data['configs'] = config
json_data = data.dumps()
with open('participant_configs.json', 'w') as f:
    txt = json.dumps(data, indent=4, sort_keys=True)
    f.write(txt)